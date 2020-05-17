cmake_minimum_required(VERSION 2.8.3)
project(reggie_random_walk)
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  geometry_msgs
  tf2_ros
  tf2_eigen
  reggie_support
)
find_package(Eigen3 REQUIRED)

catkin_package(
INCLUDE_DIRS include
#  LIBRARIES lesson_perception
CATKIN_DEPENDS
  roscpp
  std_srvs
  geometry_msgs
  tf2_ros
  tf2_eigen
  reggie_support
DEPENDS
  Eigen3
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node src/reggie_random_walk.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}) 
#############
## Install ##
#############

#############
## Testing ##
#############
