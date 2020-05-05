#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>


float EXCLUSION_BOUNDARY_PADDING = 0.02;

struct ExclusionBoundary {
  float min_x;
  float max_x;
  float min_y;
  float max_y;
  float min_z;
  float max_z;
};

class ReggieLocator
{
public:
  ReggieLocator();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr remove_nan_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segment_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, bool set_negative=false);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr remove_exclusion_boundary_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);

  void init_empty_map_cloud_ptr();

  void init_exclusion_boundaries();

  void init_map_frame();

  void get_location();

  void camera_callback(sensor_msgs::PointCloud2 msg);

  ros::NodeHandle nh_;
  std::string camera_topic_;
  std::vector<ExclusionBoundary> exclusion_boundaries_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_map_cloud_ptr_;
};
