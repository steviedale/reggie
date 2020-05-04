#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>

#include <tf2_eigen/tf2_eigen.h>

#include <string>


int main(int argc, char **argv)
{
  std::string camera_topic = "/camera/depth_registered/points";
  ros::init(argc, argv, "reggie_locator_node");
  ros::NodeHandle nh;

  ros::Publisher toRemove_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("toRemove_cloud", 1);

  std::cout << "Clear map and press ENTER";
  std::string input;
  std::getline(std::cin, input);

  sensor_msgs::PointCloud2::ConstPtr empty_env_cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(camera_topic, nh);
  ROS_INFO_STREAM("Received cloud.");

  pcl::PointCloud<pcl::PointXYZRGB> empty_env_cloud;
  pcl::fromROSMsg(*empty_env_cloud_msg, empty_env_cloud);
  ROS_INFO_STREAM("Converted cloud message.");

  // Extract non-plane points
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_env_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(empty_env_cloud));

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(50);
  seg.setDistanceThreshold(0.05);

  seg.setInputCloud(empty_env_cloud_ptr);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0)
  {
    // TODO: throw and exception here instead
    ROS_ERROR_STREAM("Could not estimate a planar model for the given dataset.") ;
    exit(1);
  }

  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
  extract_indices.setInputCloud(empty_env_cloud_ptr);
  extract_indices.setIndices(inliers);
  extract_indices.setNegative(true);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr toRemove_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  extract_indices.filter(*toRemove_cloud_ptr);

  ROS_INFO_STREAM("Total points: " << empty_env_cloud.points.size());
  ROS_INFO_STREAM("ToRemove points: " << toRemove_cloud_ptr->points.size());

  sensor_msgs::PointCloud2 toRemove_cloud_msg; 
  pcl::toROSMsg(*toRemove_cloud_ptr, toRemove_cloud_msg);

  toRemove_cloud_pub.publish(toRemove_cloud_msg);

  // cluster groups of point clouds
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> euclidean_cluster;
  std::vector<pcl::PointIndices> cluster_groups;
  euclidean_cluster.setInputCloud(toRemove_cloud_ptr);
  euclidean_cluster.setClusterTolerance(0.01);
  euclidean_cluster.setMinClusterSize(10);
  euclidean_cluster.setMaxClusterSize(10000);
  euclidean_cluster.extract(cluster_groups);
  
  // make a convex hull for each group
  std::vector<std::vector<pcl::Vertices>> hull_polygons_list;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>> hull_cloud_list;
  for (int i = 0; i < cluster_groups.size(); ++i)
  {
    // convert indices to cloud (for this group)
    pcl::PointIndices::Ptr indices_ptr = boost::make_shared<pcl::PointIndices>(cluster_groups[i]);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices_2; 
    extract_indices_2.setInputCloud(toRemove_cloud_ptr);
    extract_indices_2.setIndices(indices_ptr);
    extract_indices_2.setNegative(false);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud_ptr;
    extract_indices.filter(*hull_cloud_ptr);

    // create convex hull (for this group)
    pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;
    std::vector<pcl::Vertices> hull_polygons;
    pcl::PointCloud<pcl::PointXYZRGB> hull_points;
    convex_hull.setInputCloud(hull_cloud_ptr);
    convex_hull.reconstruct(hull_points, hull_polygons); 
    hull_cloud_list.push_back(hull_points);
    hull_polygons_list.push_back(hull_polygons);
  }

  ros::spin();
}
