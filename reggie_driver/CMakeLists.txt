cmake_minimum_required(VERSION 3.0.2)
project(reggie_driver)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS)

catkin_install_python(PROGRAMS
  scripts/wheel_controller.py
  scripts/simple_wheel_controller.py
  scripts/simple_wheel_command_listener.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${PROJECT_NAME}
)
