cmake_minimum_required(VERSION 2.8.3)
project(reggie_kinect)

find_package(catkin REQUIRED)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
