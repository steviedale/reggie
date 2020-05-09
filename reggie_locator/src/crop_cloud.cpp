#include <ros/ros.h>

#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <iostream>

float percent_red(pcl::PointXYZRGB p)
{
  return (float)p.r / ((float)p.r + (float)p.g + (float)p.b);
}

float percent_green(pcl::PointXYZRGB p)
{
  return (float)p.g / ((float)p.r + (float)p.g + (float)p.b);
}

int main(int argc, char **argv)
{
  std::string CAMERA_TOPIC = "/camera/depth_registered/points";
  std::string FRAME_ID = "camera_rgb_optical_frame";

  ros::init(argc, argv, "crop_cloud_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);

  double x_min, x_max, y_min, y_max, z_min, z_max;
  nh.getParam("/crop_cloud_node/x_min", x_min);
  nh.getParam("/crop_cloud_node/x_max", x_max);
  nh.getParam("/crop_cloud_node/y_min", y_min);
  nh.getParam("/crop_cloud_node/y_max", y_max);
  nh.getParam("/crop_cloud_node/z_min", z_min);
  nh.getParam("/crop_cloud_node/z_max", z_max);

  std::cout << "min x: " << x_min << std::endl;
  std::cout << "max x: " << x_max << std::endl;
  std::cout << "min y: " << y_min << std::endl;
  std::cout << "max y: " << y_max << std::endl;
  std::cout << "min z: " << z_min << std::endl;
  std::cout << "max z: " << z_max << std::endl;

  std::cout << "press ENTER";
  std::string input;
  std::getline(std::cin, input);

  sensor_msgs::PointCloud2::ConstPtr cloud_msg_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(CAMERA_TOPIC, nh);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_msg_ptr, *cloud_ptr);

  pcl::CropBox<pcl::PointXYZRGB> box_filter;
  box_filter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
  box_filter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
  box_filter.setInputCloud(cloud_ptr);
  box_filter.filter(*cloud_ptr);

  std::cout << "SIZE: " << cloud_ptr->size() << std::endl;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_ptr, cloud_msg);
  cloud_msg.header.frame_id = FRAME_ID;
  pub.publish(cloud_msg);

  float r_min, r_max, g_min, g_max;
  float pr_total, pg_total;
  pr_total = pg_total = 0.0;

  r_min = percent_red(cloud_ptr->points.at(0));
  r_max = percent_red(cloud_ptr->points.at(0));
  g_min = percent_green(cloud_ptr->points.at(0));
  g_max = percent_green(cloud_ptr->points.at(0));

  for(int i = 0; i < cloud_ptr->size(); ++i)
  {
    pcl::PointXYZRGB p = cloud_ptr->points.at(i);

    float pr = percent_red(p);
    float pg = percent_green(p);

    pr_total += pr;
    pg_total += pg;

    if (pr < r_min) r_min = pr;
    if (pr > r_max) r_max = pr;
    if (pg < g_min) g_min = pg;
    if (pg > g_max) g_max = pg;

    std::cout << pr << ", " << pg << std::endl;
  }
  std::cout << "R: [ " << r_min << ",  " << r_max << " ]" << std::endl;
  std::cout << "G: [ " << g_min << ",  " << g_max << " ]" << std::endl;

  std::cout << "Avg(pr): " << pr_total / cloud_ptr->size() << std::endl;
  std::cout << "Avg(pg): " << pg_total / cloud_ptr->size() << std::endl;

  ros::spin();
}
