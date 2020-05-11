#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

class ReggieRandomWalk
{
public:
  ReggieRandomWalk();

  void send_to_center();
  void call_update_robot_pose();
  void set_orientation(float theta);
  Eigen::Isometry3d get_robot_transform();

  // Variables //
  ros::NodeHandle nh_;
  ros::ServiceClient update_robot_pose_client_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  float map_x_length_, map_y_length_;
  Eigen::Vector3d center_point_;
};
