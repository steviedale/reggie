#include <reggie_locator/reggie_locator.h>
#include <reggie_locator/k_means.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

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

#include <iostream>


float percent_red(pcl::PointXYZRGB p)
{
  return (float)p.r / ((float)p.r + (float)p.g + (float)p.b);
}

float percent_green(pcl::PointXYZRGB p)
{
  return (float)p.g / ((float)p.r + (float)p.g + (float)p.b);
}

ReggieLocator::ReggieLocator()
: nh_()
, camera_topic_("/camera/depth_registered/points")
, cleaned_map_pub(nh_.advertise<sensor_msgs::PointCloud2>("cleaned_map", 1))
, raw_map_pub(nh_.advertise<sensor_msgs::PointCloud2>("raw_map", 1))
, green_marker_pub(nh_.advertise<sensor_msgs::PointCloud2>("green_marker", 1))
, yellow_marker_pub(nh_.advertise<sensor_msgs::PointCloud2>("yellow_marker", 1))
{
  init_empty_map_cloud_ptr();
  init_exclusion_boundaries();
  empty_map_cloud_ptr_ = segment_plane(empty_map_cloud_ptr_, false);
  init_map_frame();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cleaned_map_cloud_ptr = remove_exclusion_boundary_points(empty_map_cloud_ptr_);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cleaned_map_cloud_ptr, cloud_msg);
  cloud_msg.header.frame_id = CAMERA_FRAME;
  cleaned_map_pub.publish(cloud_msg);

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
  /*
  std::cout << "Clear map and press ENTER";
  std::string input;
  std::getline(std::cin, input);
  */
  std::cout << "sleeping..." << std::endl;
  ros::Duration(3.0).sleep();
  std::cout << "Awake!" << std::endl;

  empty_map_cloud_ptr_ = getPointCloud();
}

Eigen::Vector3d ReggieLocator::get_plane_normal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
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

  Eigen::Vector3d plane_normal(coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2));
  std::cout << "Plane Coefficients: " << coefficients->values.at(0) << ", " << coefficients->values.at(1) << ", " << coefficients->values.at(2) << std::endl;

  return plane_normal;
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

    pcl::PointXYZRGB first_point = cloud_ptr->points.at(0);
    ExclusionBoundary boundary;
    boundary.min_x = boundary.max_x = first_point.x;
    boundary.min_y = boundary.max_y = first_point.y;
    boundary.min_z = boundary.max_z = first_point.z;

    for (int j = 0; j < indices.indices.size(); ++j)
    {
      pcl::PointXYZRGB p = cloud_ptr->points.at(indices.indices.at(j));
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ReggieLocator::filter_color_range(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, float min_pr, float max_pr, float min_pg, float max_pg)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointIndices::Ptr keep_indices(new pcl::PointIndices);

  for (int i = 0; i < cloud_ptr->points.size(); ++i)
  {
    pcl::PointXYZRGB p = cloud_ptr->points.at(i);

    float pr = percent_red(p);
    float pg = percent_green(p);

    if (pr >= min_pr && pr <= max_pr && pg >= min_pg && pg <= max_pg)
    {
      keep_indices->indices.push_back(i);
    }
  }
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
  extract_indices.setInputCloud(cloud_ptr);
  extract_indices.setIndices(keep_indices);
  extract_indices.setNegative(false);
  extract_indices.filter(*new_cloud_ptr);

  return new_cloud_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ReggieLocator::filter_biggest_cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
  std::cout << "points in cloud: " << cloud_ptr->size() << std::endl;
  // get groups of clusters
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> euclidean_cluster;
  euclidean_cluster.setInputCloud(cloud_ptr);
  euclidean_cluster.setClusterTolerance(0.1);
  euclidean_cluster.setMinClusterSize(10);
  euclidean_cluster.setMaxClusterSize(10000);
  std::vector<pcl::PointIndices> cluster_groups;
  euclidean_cluster.extract(cluster_groups);

  std::cout << "num_clusters: " << cluster_groups.size() << std::endl;
  // find biggest cluster
  pcl::PointIndices::Ptr biggest_cluster_indices;
  int biggest_cluster_size = 0;
  for (int i = 0; i < cluster_groups.size(); ++i)
  {
    int cluster_size = cluster_groups.at(i).indices.size();
    std::cout << "cluster_size: " << cluster_size << std::endl;
    if (cluster_size > biggest_cluster_size)
    {
      biggest_cluster_size = cluster_size;
      biggest_cluster_indices = boost::make_shared<pcl::PointIndices>(cluster_groups.at(i));
    }
  }

  // filter points from biggest cluster
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
  extract_indices.setInputCloud(cloud_ptr);
  extract_indices.setIndices(biggest_cluster_indices);
  extract_indices.setNegative(false);
  extract_indices.filter(*new_cloud_ptr);

  std::cout << "end check 5" << std::endl;
  return new_cloud_ptr;
}

Eigen::Vector3d ReggieLocator::get_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
  Eigen::Vector3d centroid;
  for (int i = 0; i < cloud_ptr->size(); ++i)
  {
    pcl::PointXYZRGB p = cloud_ptr->points.at(i);
    centroid[0] += (p.x - centroid[0]) / (i+1);
    centroid[1] += (p.y - centroid[1]) / (i+1);
    centroid[2] += (p.z - centroid[2]) / (i+1);
  }
  return centroid;
}

void ReggieLocator::init_map_frame()
{
/*
  float yellow_r_min, yellow_r_max, yellow_g_min, yellow_g_max;
  nh_.getParam("/marker_color_ranges/yellow/r_min", yellow_r_min);
  nh_.getParam("/marker_color_ranges/yellow/r_max", yellow_r_max);
  nh_.getParam("/marker_color_ranges/yellow/g_min", yellow_g_min);
  nh_.getParam("/marker_color_ranges/yellow/g_max", yellow_g_max);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr yellow_marker_cloud_ptr = filter_color_range(empty_map_cloud_ptr_, yellow_r_min,
    yellow_r_max, yellow_g_min, yellow_g_max);
*/
  float padding = 0.01;
  float yellow_r_avg, yellow_g_avg;
  nh_.getParam("/marker_color_ranges/yellow/r_avg", yellow_r_avg);
  nh_.getParam("/marker_color_ranges/yellow/g_avg", yellow_g_avg);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr yellow_marker_cloud_ptr = filter_color_range(empty_map_cloud_ptr_,
    yellow_r_avg - padding,
    yellow_r_avg + padding,
    yellow_g_avg - padding,
    yellow_g_avg + padding
  );
  yellow_marker_cloud_ptr = filter_biggest_cluster(yellow_marker_cloud_ptr);

  sensor_msgs::PointCloud2 yellow_cloud_msg;
  pcl::toROSMsg(*yellow_marker_cloud_ptr, yellow_cloud_msg);
  yellow_cloud_msg.header.frame_id = CAMERA_FRAME;
  yellow_marker_pub.publish(yellow_cloud_msg);

  Eigen::Vector3d yellow_marker_centroid = get_centroid(yellow_marker_cloud_ptr);

  /*
  float green_r_min, green_r_max, green_g_min, green_g_max;
  nh_.getParam("/marker_color_ranges/green/r_min", green_r_min);
  nh_.getParam("/marker_color_ranges/green/r_max", green_r_max);
  nh_.getParam("/marker_color_ranges/green/g_min", green_g_min);
  nh_.getParam("/marker_color_ranges/green/g_max", green_g_max);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr green_marker_cloud_ptr = filter_color_range(empty_map_cloud_ptr_, green_r_min,
    green_r_max, green_g_min, green_g_max);
  */
  float green_r_avg, green_g_avg;
  nh_.getParam("/marker_color_ranges/green/r_avg", green_r_avg);
  nh_.getParam("/marker_color_ranges/green/g_avg", green_g_avg);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr green_marker_cloud_ptr = filter_color_range(empty_map_cloud_ptr_,
    green_r_avg - padding,
    green_r_avg + padding,
    green_g_avg - padding,
    green_g_avg + padding
  );
  green_marker_cloud_ptr = filter_biggest_cluster(green_marker_cloud_ptr);

  sensor_msgs::PointCloud2 green_cloud_msg;
  pcl::toROSMsg(*green_marker_cloud_ptr, green_cloud_msg);
  green_cloud_msg.header.frame_id = CAMERA_FRAME;
  green_marker_pub.publish(green_cloud_msg);

  Eigen::Vector3d green_marker_centroid = get_centroid(green_marker_cloud_ptr);

  Eigen::Vector3d x_vector = (yellow_marker_centroid - green_marker_centroid);
  x_vector = x_vector / x_vector.norm();
//  std::cout << "x_vector: " << x_vector[0] << ", " << x_vector[1] << ", " << x_vector[2] << std::endl;

  Eigen::Vector3d z_vector = get_plane_normal(empty_map_cloud_ptr_);
  z_vector = -1 * z_vector / z_vector.norm();
//  std::cout << "z_vector: " << z_vector[0] << ", " << z_vector[1] << ", " << z_vector[2] << std::endl;

  Eigen::Vector3d y_vector = z_vector.cross(x_vector);
  y_vector = y_vector / y_vector.norm();
//  std::cout << "y_vector: " << y_vector[0] << ", " << y_vector[1] << ", " << y_vector[2] << std::endl;

  Eigen::Matrix3d rotation;
  rotation(0,0) = z_vector[0];
  rotation(1,0) = z_vector[1];
  rotation(2,0) = z_vector[2];
  rotation(0,1) = y_vector[0];
  rotation(1,1) = y_vector[1];
  rotation(2,1) = y_vector[2];
  rotation(0,2) = x_vector[0];
  rotation(1,2) = x_vector[1];
  rotation(2,2) = x_vector[2];

  camera_to_map_tf_ = Eigen::Isometry3d::Identity();
  camera_to_map_tf_.linear() = rotation;
  camera_to_map_tf_.translation() = yellow_marker_centroid;
  geometry_msgs::TransformStamped transform_msg = tf2::eigenToTransform(camera_to_map_tf_);
  transform_msg.header.frame_id = CAMERA_FRAME;
  transform_msg.child_frame_id = MAP_FRAME;
  static_broadcaster_.sendTransform(transform_msg);

/*
  Eigen::Isometry3d green_tf = Eigen::Isometry3d::Identity();
  green_tf.linear() = rotation();
  green_tf.translation() = green_marker_centroid;
  geometry_msgs::TransformStamped green_tf_msg = tf2::eigenToTransform(green_tf);
  green_tf_msg.header.frame_id = CAMERA_FRAME;
  green_tf_msg.child_frame_id = "green_marker";
  static_broadcaster_.sendTransform(green_tf_msg);
*/
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
