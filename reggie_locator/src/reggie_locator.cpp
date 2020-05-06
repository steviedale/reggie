#include <reggie_locator/reggie_locator.h>
#include <reggie_locator/k_means.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/reconstruction.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/ply_io.h>

#include <tf2_eigen/tf2_eigen.h>

#include <string>
#include <iostream>


ReggieLocator::ReggieLocator()
: nh_()
, camera_topic_("/camera/depth_registered/points")
{
  //ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>("map_cleaned", 1);

  init_empty_map_cloud_ptr();
  init_exclusion_boundaries();
  init_map_frame();

  /*
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cleaned_map_cloud_ptr = remove_exclusion_boundary_points(empty_map_cloud_ptr_); 
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_map_cloud_ptr, cloud_msg);
  pub.publish(cloud_msg);
  */

  ros::spin();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ReggieLocator::getPointCloud()
{
  sensor_msgs::PointCloud2::ConstPtr cloud_msg_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(camera_topic_, nh_);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_msg_ptr, *cloud_ptr);
  return remove_nan_points(cloud_ptr);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ReggieLocator::remove_nan_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
  // remove all points with x, y, or z = NaN
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_ptr, *new_cloud_ptr, indices);

  ROS_INFO_STREAM("Total points: " << new_cloud_ptr->points.size());
  ROS_INFO_STREAM("Remaining points: " << new_cloud_ptr->points.size());

  return new_cloud_ptr;
}

void ReggieLocator::init_empty_map_cloud_ptr()
{
  std::cout << "Clear map and press ENTER";
  std::string input;
  std::getline(std::cin, input);

  empty_map_cloud_ptr_ = getPointCloud();
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr ReggieLocator::segment_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, bool set_negative)
{
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(50);
  seg.setDistanceThreshold(0.05);
  seg.setInputCloud(cloud_ptr);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0)
  {
    // TODO: throw and exception here instead
    ROS_ERROR_STREAM("Could not estimate a planar model for the given dataset.") ;
    exit(1);
  }
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
  extract_indices.setInputCloud(cloud_ptr);
  extract_indices.setIndices(inliers);
  extract_indices.setNegative(set_negative);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  extract_indices.filter(*new_cloud_ptr);
  return new_cloud_ptr;
}

void ReggieLocator::init_exclusion_boundaries()
{
  // remove plane
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr = segment_plane(empty_map_cloud_ptr_, true);

  // cluster groups of point clouds
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> euclidean_cluster;
  euclidean_cluster.setInputCloud(cloud_ptr);
  euclidean_cluster.setClusterTolerance(0.1);
  euclidean_cluster.setMinClusterSize(10);
  euclidean_cluster.setMaxClusterSize(10000);
  std::vector<pcl::PointIndices> cluster_groups;
  euclidean_cluster.extract(cluster_groups);
  
  for (int i = 0; i < cluster_groups.size(); ++i)
  {
    pcl::PointIndices indices = cluster_groups.at(i);

    pcl::PointXYZRGB first_point = cloud_ptr->points.at(indices.indices.at(i));
    ExclusionBoundary boundary;
    boundary.min_x = boundary.max_x = first_point.x;
    boundary.min_y = boundary.max_y = first_point.y;
    boundary.min_z = boundary.max_z = first_point.z;

    for (int i = 0; i < indices.indices.size(); ++i)
    {
      pcl::PointXYZRGB p = cloud_ptr->points.at(indices.indices.at(i));       
      if (p.x < boundary.min_x) boundary.min_x = p.x - EXCLUSION_BOUNDARY_PADDING;
      if (p.y < boundary.min_y) boundary.min_y = p.y - EXCLUSION_BOUNDARY_PADDING;
      if (p.z < boundary.min_z) boundary.min_z = p.z - EXCLUSION_BOUNDARY_PADDING;
      if (p.x > boundary.max_x) boundary.max_x = p.x + EXCLUSION_BOUNDARY_PADDING;
      if (p.y > boundary.max_y) boundary.max_y = p.y + EXCLUSION_BOUNDARY_PADDING;
      if (p.z > boundary.max_z) boundary.max_z = p.z + EXCLUSION_BOUNDARY_PADDING;
    }

    exclusion_boundaries_.push_back(boundary);
  }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ReggieLocator::remove_exclusion_boundary_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*cloud_ptr, *new_cloud_ptr);

  for (int i = 0; i < exclusion_boundaries_.size(); ++i)
  {
    ExclusionBoundary eb = exclusion_boundaries_.at(i);
    pcl::CropBox<pcl::PointXYZRGB> box_filter;
    box_filter.setMin(Eigen::Vector4f(eb.min_x, eb.min_y, eb.min_z, 0));
    box_filter.setMax(Eigen::Vector4f(eb.max_x, eb.max_y, eb.max_z, 0));
    box_filter.setInputCloud(new_cloud_ptr);
    box_filter.setNegative(true);
    box_filter.filter(*new_cloud_ptr);
  }
  return new_cloud_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ReggieLocator::filter_color_range(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, float min_r, float max_r, float min_g, float max_g, float min_b, float max_b)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*cloud_ptr, *new_cloud_ptr);

  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, max_r)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, min_r)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, max_g)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, min_g)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, max_b)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, min_b)));

  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem(color_cond);
  condrem.setInputCloud(cloud_ptr);
  condrem.setKeepOrganized(true);

  condrem.filter(*new_cloud_ptr);
}

void ReggieLocator::init_map_frame()
{

  std::vector<Boundary> cluster_boundaries = k_means_cluster(empty_map_cloud_ptr_, 4);
  for(int i = 0; i < cluster_boundaries.size(); ++i)
  {
    XYZRGB min_p = cluster_boundaries.at(i).min;
    XYZRGB max_p = cluster_boundaries.at(i).max;

    std::cout << "X = [" << min_p.x << ", " << max_p.x << "]" << std::endl;
    std::cout << "Y = [" << min_p.y << ", " << max_p.y << "]" << std::endl;
    std::cout << "Z = [" << min_p.z << ", " << max_p.z << "]" << std::endl;
    std::cout << "R = [" << min_p.r << ", " << max_p.r << "]" << std::endl;
    std::cout << "G = [" << min_p.y << ", " << max_p.g << "]" << std::endl;
    std::cout << "B = [" << min_p.z << ", " << max_p.b << "]" << std::endl << std::endl;
  }
}

void ReggieLocator::get_location()
{
  // remove points inside exclusion boundaries
  //for (int i = 0; i < exclusion_boundaries_.size(); ++i)
  //{
  //}
  // remove points from plane
  // cluster (hopefully there's only one)

  // find points for each sticker on 
  // calculate pose
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reggie_locator_node");
  ReggieLocator reggie_locator;
}
