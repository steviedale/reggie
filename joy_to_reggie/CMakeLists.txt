cmake_minimum_required(VERSION 3.0.2)
project(joy_to_reggie)

find_package(catkin REQUIRED COMPONENTS 
  reggie_driver
  joy
)

catkin_package(CATKIN_DEPENDS reggie_driver joy)

catkin_install_python(PROGRAMS
  scripts/joy_to_reggie_node.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${PROJECT_NAME}
)
