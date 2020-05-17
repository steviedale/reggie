cmake_minimum_required(VERSION 2.8.3)
project(reggie_localize)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf2_ros
  pcl_msgs
  pcl_conversions
  visualization_msgs
  std_srvs
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
INCLUDE_DIRS include
#  LIBRARIES lesson_perception
#  CATKIN_DEPENDS roscpp sensor_msgs tf
DEPENDS Eigen3
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

#add_library(k_means src/k_means.cpp)
#target_link_libraries(k_means ${catkin_LIBRARIES} ${PCL_LIBRARIES})

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node src/reggie_localize.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(crop_cloud src/crop_cloud.cpp)
target_link_libraries(crop_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#############
## Install ##
#############
install(DIRECTORY launch/ config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

#############
## Testing ##
#############
