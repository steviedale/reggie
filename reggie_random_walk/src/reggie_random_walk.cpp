#include <reggie_random_walk/reggie_random_walk.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Trigger.h>
#include <math.h>
#include <reggie_support/WheelCommand.h>
#include <iostream>
#include <string>


float PI = 3.141592;


float rad_to_deg(float radians){
  return 180.0 * radians / PI;
}

float get_rotation(float x, float y){
  if (x == 0.0) {
    if (y > 0.0) return (1 / 2) * PI;
    else return (3 / 2) * PI;
  }
  else if (y == 0.0) {
    if (x > 0.0) return 0.0;
    else return PI;
  }
  else {
    // quadrant I
    if (x > 0.0 && y > 0.0) return atan(y / x);
    // quadrant II
    else if (x < 0.0 && y > 0.0) return PI - atan(-1 * y / x);
    // quadrant III
    else if (x < 0.0 && y < 0.0) return PI + atan(y / x);
    // quadrant IV
    else if (x > 0.0 && y < 0.0) return 2 * PI - atan(-1 * y / x);
    else { ROS_ERROR_STREAM("Unexpected case in get_rotation() function."); exit(1); }
  }
}

float get_rotation(Eigen::Isometry3d transform){
  float x = transform.linear()(0,0);
  float y = transform.linear()(1,0);
  return get_rotation(x, y);
}

ReggieRandomWalk::ReggieRandomWalk()
: nh_()
, tf_buffer_()
, tf_listener_(tf_buffer_)
, update_robot_pose_client_(nh_.serviceClient<std_srvs::Trigger>("update_robot_pose"))
, simple_wheel_command_client_(nh_.serviceClient<reggie_support::WheelCommand>("simple_wheel_command"))
{
  while (
    !nh_.hasParam("/map_x_length") ||
    !nh_.hasParam("/map_y_length") ||
    !nh_.hasParam("/switch_to_short_rotation_threshold_radians") ||
    !nh_.hasParam("/switch_to_short_translation_threshold_meters") ||
    !nh_.hasParam("/radians_from_goal_tolerance") ||
    !nh_.hasParam("/meters_from_goal_tolerance") ||
    !nh_.hasParam("/long_delta_rotation_estimate_radians") ||
    !nh_.hasParam("/short_delta_rotation_estimate_radians") ||
    !nh_.hasParam("/long_delta_translation_estimate_meters") ||
    !nh_.hasParam("/short_delta_translation_estimate_meters") ||
    !nh_.hasParam("/rotation_measurement_tolerance_radians") ||
    !nh_.hasParam("/translation_measurement_tolerance_meters")
  ) { ROS_ERROR_STREAM("Waiting for parameters..."); ros::Duration(1.0).sleep(); }

  // dimensions of map
  nh_.getParam("/map_x_length", map_x_length_);
  nh_.getParam("/map_y_length", map_y_length_);

  // this will determine at what point we should stop taking long movements and start taking short ones
  nh_.getParam("/switch_to_short_rotation_threshold_radians", switch_to_short_rotation_threshold_radians_);
  nh_.getParam("/switch_to_short_translation_threshold_meters", switch_to_short_translation_threshold_meters_);

  // this will determine when we are close enough to the target to stop moving
  nh_.getParam("/radians_from_goal_tolerance", radians_from_goal_tolerance_);
  nh_.getParam("/meters_from_goal_tolerance", meters_from_goal_tolerance_);

  // this will be used to estimate what the pose should be after a give action
  nh_.getParam("/long_delta_rotation_estimate_radians", long_delta_rotation_estimate_radians_);
  nh_.getParam("/short_delta_rotation_estimate_radians", short_delta_rotation_estimate_radians_);
  nh_.getParam("/long_delta_translation_estimate_meters", long_delta_translation_estimate_meters_);
  nh_.getParam("/short_delta_translation_estimate_meters", short_delta_translation_estimate_meters_);

  // if the predicted pose is off (differs from estimate) by more than this setting, we ignore the prediction (or get another one)
  nh_.getParam("/rotation_measurement_tolerance_radians", rotation_measurement_tolerance_radians_);
  nh_.getParam("/translation_measurement_tolerance_meters", translation_measurement_tolerance_meters_);

  center_point_ = Eigen::Vector3d(map_x_length_/2, map_y_length_/2, 0.0);
  Eigen::Isometry3d center_point_tf = Eigen::Isometry3d::Identity();
  center_point_tf.translation() = center_point_;
  geometry_msgs::TransformStamped transform_msg = tf2::eigenToTransform(center_point_tf);
  transform_msg.header.frame_id = "map_frame";
  transform_msg.child_frame_id = "center_point";
  static_broadcaster_.sendTransform(transform_msg);

  // get initial pose
  std::string input;
  do {
    call_update_robot_pose();
    robot_transform_ = get_robot_transform();
    const Eigen::Vector3d position = robot_transform_.translation();
    const float rotation = get_rotation(robot_transform_);

    ROS_ERROR_STREAM("X: " << position.x());
    ROS_ERROR_STREAM("Y: " << position.y());
    ROS_ERROR_STREAM("Theta: " << rad_to_deg(rotation));

    ROS_ERROR_STREAM("Enter 'y' to approve initial pose:");
    std::getline(std::cin, input);
  } while (input != "y");

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
  geometry_msgs::TransformStamped transform_msg = tf_buffer_.lookupTransform("map_frame", "base_link", ros::Time(0), ros::Duration(TF_TIMEOUT));
  return tf2::transformToEigen(transform_msg);
}

void ReggieRandomWalk::set_orientation(float goal_rotation)
{
  ROS_WARN_STREAM("Goal rotation: " << rad_to_deg(goal_rotation));
  int consecutive_estimates = 0;
  while (true) {
    float measured_rotation = get_rotation(robot_transform_);

    if (abs(measured_rotation - goal_rotation) < radians_from_goal_tolerance_) {
      ROS_ERROR_STREAM("GOAL MET.");
      break;
    }

    float turn_radians = goal_rotation - measured_rotation;
    bool turn_left = true;
    if (turn_radians > 0) {
      if (turn_radians > PI) {
        turn_left = false;
        turn_radians -= PI;
      }
      else {
        turn_left = true;
      }
    }
    // turn_radians < 0
    else {
       turn_radians = abs(turn_radians);
       if (turn_radians > PI) {
         turn_radians -= PI;
         turn_left = true;
       }
       else {
         turn_left = false;
       }
    }
    ROS_ERROR_STREAM("turn radians: " << rad_to_deg(turn_radians));
    if (turn_left){ ROS_ERROR_STREAM("turning left"); }
    else { ROS_ERROR_STREAM("turning right"); }

    bool long_move = turn_radians > switch_to_short_rotation_threshold_radians_;
    if (long_move) ROS_ERROR_STREAM("Long move");
    else ROS_ERROR_STREAM("Short move");

    std::string input;
    ROS_ERROR_STREAM("Press ENTER to execute move.");
    std::getline(std::cin, input);

    reggie_support::WheelCommand wheel_command;
    wheel_command.request.long_move = long_move;
    wheel_command.request.command = (turn_left) ? LEFT_COMMAND : RIGHT_COMMAND;

    int attempt_num = 1;
    while (true) {
      simple_wheel_command_client_.call(wheel_command);
      if (wheel_command.response.success){
        break;
      }
      else {
        ROS_WARN_STREAM("Simple wheel command was unsuccessful (attempt " << attempt_num << " of 5)");
        ++attempt_num;
      }
      if (attempt_num > 5){
        ROS_ERROR_STREAM("Simple wheel command failed. Exiting.");
        exit(1);
      }
    }
    // update the rotation of the robot either via the measurement or estimate (if the measurement is bad)
    for (int i = 0; i < allowed_measurement_attempts_; ++i) {
      call_update_robot_pose();
      Eigen::Isometry3d next_measured_transform = get_robot_transform();
      float next_measured_rotation = get_rotation(next_measured_transform);
      float measured_delta_rotation = next_measured_rotation - measured_rotation;

      float estimated_delta_rotation = (long_move) ? long_delta_rotation_estimate_radians_ : short_delta_rotation_estimate_radians_;
      if (!turn_left) estimated_delta_rotation *= -1.0;

      ROS_WARN_STREAM("measured_delta_rotation: " << rad_to_deg(measured_delta_rotation) << " deg");
      ROS_WARN_STREAM("estimated_delta_rotation: " << rad_to_deg(estimated_delta_rotation) << " deg");
      ROS_WARN_STREAM("measured_delta_rotation: " << measured_delta_rotation);
      ROS_WARN_STREAM("estimated_delta_rotation: " << estimated_delta_rotation);

      // if the predicted change in rotation was intolerably more than estimated
      if (abs(measured_delta_rotation - estimated_delta_rotation) > rotation_measurement_tolerance_radians_ * (consecutive_estimates + 1)) {
        ROS_WARN_STREAM("Declined delta rotation measurement.");
        // if this was the final bad attempt
        if (i+1 == allowed_measurement_attempts_) {
          // manually adjust transform based on estimate
          ROS_WARN_STREAM("Failed to measure acceptable delta rotation. Setting robot rotation based on estimate.");
          robot_transform_.rotate(Eigen::AngleAxisd(estimated_delta_rotation, Eigen::Vector3d::UnitZ()));
          ++consecutive_estimates;
          ROS_WARN_STREAM("Consecutive estimates: " << consecutive_estimates);
        }
      }
      // if the rotation measurement was good (within tolerance)
      else {
        ROS_WARN_STREAM("Accepted rotation measurement.");
        robot_transform_ = next_measured_transform;
        consecutive_estimates = 0;
        break;
      }
    }
  }
}

void ReggieRandomWalk::send_to_center()
{
  Eigen::Isometry3d transform = get_robot_transform();
  Eigen::Vector3d to_center_vector = center_point_ - transform.translation();
  ROS_ERROR_STREAM("to_center_vector: X=" << to_center_vector.x() << " Y=" << to_center_vector.y());
  float goal_orientation = get_rotation(to_center_vector(0), to_center_vector(1));
  set_orientation(goal_orientation);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reggie_random_walk_node");
  ReggieRandomWalk reggie_random_walk;
  ros::spin();
}
