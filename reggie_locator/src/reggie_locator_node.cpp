#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
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

  ros::spin();
}
