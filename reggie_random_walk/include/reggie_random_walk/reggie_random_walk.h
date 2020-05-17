#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>


float TF_TIMEOUT = 2.0;
int FORWARD_COMMAND = 0;
int BACKWARD_COMMAND = 1;
int LEFT_COMMAND = 2;
int RIGHT_COMMAND = 3;

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
  ros::ServiceClient update_robot_pose_client_, simple_wheel_command_client_, add_episode_element_client_, write_episode_client_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  float map_x_length_, map_y_length_;
  // this will determine at what point we should stop taking long movements and start taking short ones
  float switch_to_short_rotation_threshold_radians_, switch_to_short_translation_threshold_meters_;
  // this will determine when we are close enough to the target to stop moving
  float radians_from_goal_tolerance_, meters_from_goal_tolerance_;
  // this will be used to estimate what the pose should be after a give action
  float long_delta_rotation_estimate_radians_, short_delta_rotation_estimate_radians_, long_delta_translation_estimate_meters_, short_delta_translation_estimate_meters_;
  // if the measured pose is off (differs from estimate) by more than this setting, we ignore the measurement (or get another measurement)
  float rotation_measurement_tolerance_radians_, translation_measurement_tolerance_meters_;
  // the number of bad measurements can be taken before we stop measuring and go with the estimate
  int allowed_measurement_attempts_ = 5;

  bool record_episode_;

  Eigen::Vector3d center_point_;
  Eigen::Isometry3d robot_transform_;
};
