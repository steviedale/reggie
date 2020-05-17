#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
//#include <Eigen/Dense>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <exception>
#include <string>


std::string CAMERA_FRAME = "camera_rgb_optical_frame";
std::string MAP_FRAME = "map_frame";
std::string ROBOT_FRAME = "base_link";
float EXCLUSION_BOUNDARY_PADDING = 0.05;
float MARKER_BOUNDARY_PADDING = 0.02;


struct Boundary {
  float min_x;
  float max_x;
  float min_y;
  float max_y;
  float min_z;
  float max_z;
};

struct ClusterNotFoundException : public std::exception
{
	const char * what () const throw ()
    {
    	return "Could not find cluster.";
    }
};

struct EmptyCloudException : public std::exception
{
	const char * what () const throw ()
    {
    	return "Cloud has size == 0.";
    }
};

class ReggieLocalize
{
public:
  ReggieLocalize();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_point_cloud(int num_clouds=1);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr remove_nan_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segment_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, bool set_negative=false, float distance_thresh=0.05);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segment_boundary(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, Boundary boundary, bool set_negative=false);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter_color_range(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr,
    float min_pr, float max_pr, float min_pg, float max_pg, float min_pa, float max_pa);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter_biggest_cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);

  void init_plane_normal();

  Eigen::Vector3d get_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);

  Boundary create_boundary(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);

  void init_map_frame();

  bool update_robot_pose(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

  // Variables //
  ros::NodeHandle nh_;

  ros::Publisher cleaned_map_pub_;
  ros::Publisher raw_map_pub_;
  ros::Publisher yellow_map_marker_pub_;
  ros::Publisher green_map_marker_pub_;
  ros::Publisher blue_map_marker_pub_;
  ros::Publisher blue_robot_marker_pub_;
  ros::Publisher green_robot_marker_pub_;
  ros::Publisher no_map_markers_pub_;

  ros::ServiceServer update_robot_pose_srv_;

  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  tf2_ros::TransformBroadcaster dynamic_broadcaster_;

  std::string camera_topic_;

  Eigen::Isometry3d camera_to_map_tf_;

  Eigen::Vector3d plane_normal_;

  std::vector<Boundary> marker_boundaries_;

  float map_x_length_, map_y_length_;
  bool map_frame_initialized_ = false;
  float color_tolerance_;
  float alpha_tolerance_;
  bool cluster_markers_;
};
