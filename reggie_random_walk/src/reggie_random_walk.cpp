#include <reggie_random_walk/reggie_random_walk.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Trigger.h>
#include <math.h>


float get_theta(float x, float y){
  if (x == 0.0) {
    if (y > 0.0) return 90.0;
    else return 270.0;
  }
  else if (y == 0.0) {
    if (x > 0.0) return 0.0;
    else return 180.0;
  }
  else {
    if (x > 0.0 && y > 0.0) return atan(y / x);
    else if (x < 0.0 && y > 0.0) return 180.0 - atan(-1 * y / x);
    else if (x < 0.0 && y < 0.0) return 180.0 + atan(y / x);
    else if (x > 0.0 && y < 0.0) return 360.0 - atan(-1 * y / x);
    else { ROS_ERROR_STREAM("Unexpected case in get_theta() function."); exit(1); }
  }
}

float get_theta(Eigen::Isometry3d transform){
  float x = transform.linear()(0,0);
  float y = transform.linear()(1,0);
  return get_theta(x, y);
}

ReggieRandomWalk::ReggieRandomWalk()
: nh_()
, tf_buffer_()
, tf_listener_(tf_buffer_)
, update_robot_pose_client_(nh_.serviceClient<std_srvs::Trigger>("update_robot_pose"))
{
  while (!nh_.hasParam("/map_x_length") || !nh_.hasParam("/map_y_length")) { ros::Duration(0.5).sleep(); }
  nh_.getParam("/map_x_length", map_x_length_);
  nh_.getParam("/map_y_length", map_y_length_);

  center_point_ = Eigen::Vector3d(map_x_length_/2, map_y_length_/2, 0.0);

  send_to_center();
}

void ReggieRandomWalk::call_update_robot_pose()
{
  // update robot position
  std_srvs::Trigger trigger;
  int attempt_num = 1;
  while(true) {
    update_robot_pose_client_.call(trigger);
    if (trigger.response.success) break;
    else {
      ROS_WARN_STREAM("Failed to update robot pose (attempt " << attempt_num << " of 5).");
      attempt_num += 1;
      if (attempt_num > 5) {
        ROS_ERROR_STREAM("Failed to update robot pose. Exiting.");
        exit(1);
      }
    }
  }
}

Eigen::Isometry3d ReggieRandomWalk::get_robot_transform()
{
  geometry_msgs::TransformStamped transform_msg = tf_buffer_.lookupTransform("map_frame", "base_link", ros::Time(0), ros::Duration(2.0));
  return tf2::transformToEigen(transform_msg);
}

void ReggieRandomWalk::set_orientation(float goal_theta)
{
  Eigen::Isometry3d transform;
  float actual_theta;
  while (true) {
    call_update_robot_pose();

    //geometry_msgs::TransformStamped transform_msg = tf_buffer_.lookupTransform("map_frame", "base_link", ros::Time(0));
    //Eigen::Isometry3d transform = tf2::transformToEigen(transform_msg);
    transform = get_robot_transform();

    actual_theta = get_theta(transform);
    ROS_WARN_STREAM("Robot theta: " << actual_theta);
  }
}

void ReggieRandomWalk::send_to_center()
{
  Eigen::Isometry3d transform = get_robot_transform();
  Eigen::Vector3d to_center_vector = transform.translation() - center_point_;
  float goal_orientation = get_theta(to_center_vector(0), to_center_vector(1));
  ROS_INFO_STREAM("Goal orientation: " << goal_orientation);
  set_orientation(goal_orientation);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reggie_random_walk_node");
  ReggieRandomWalk reggie_random_walk;
  ros::spin();
}
