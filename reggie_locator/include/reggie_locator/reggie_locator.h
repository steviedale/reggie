#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <Eigen/Dense>
#include <string>
#include <tf2_ros/static_transform_broadcaster.h>

std::string CAMERA_FRAME = "camera_link";
std::string MAP_FRAME = "map_frame";
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

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter_color_range(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr,
    float min_pr, float max_pr, float min_pg, float max_pg);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter_biggest_cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);

  Eigen::Vector3d get_plane_normal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);

  Eigen::Vector3d get_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);

  void init_empty_map_cloud_ptr();

  void init_exclusion_boundaries();

  void init_map_frame();

  void get_location();

  void camera_callback(sensor_msgs::PointCloud2 msg);

  ros::NodeHandle nh_;
  ros::Publisher cleaned_map_pub;
  ros::Publisher raw_map_pub;
  ros::Publisher yellow_marker_pub;
  ros::Publisher green_marker_pub;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  std::string camera_topic_;
  std::vector<ExclusionBoundary> exclusion_boundaries_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_map_cloud_ptr_;
  Eigen::Isometry3d camera_to_map_tf_;
};
