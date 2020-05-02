cmake_minimum_required(VERSION 3.0.2)
project(reggie_wheel_controller)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(DIRECTORY msg FILES WheelVelocity.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

# do not wildcard install files since the root folder of the package will contain a debian folder for releasing
catkin_install_python(PROGRAMS
  scripts/wheel_controller.py
  scripts/talker.py
  scripts/listener.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/reggie_wheel_controller
)
#install(FILES
#  001_talker_listener/README
#  001_talker_listener/talker_listener.launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/001_talker_listener
#)
