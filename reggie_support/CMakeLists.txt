cmake_minimum_required(VERSION 2.8.3)
project(reggie_support)

find_package(catkin REQUIRED)

install(DIRECTORY launch/ config/ rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
