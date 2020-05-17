cmake_minimum_required(VERSION 2.8.3)
project(reggie_episode)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  reggie_support
  std_srvs
)

catkin_package(
INCLUDE_DIRS include
#  LIBRARIES lesson_perception
  CATKIN_DEPENDS 
    roscpp 
    reggie_support
    std_srvs
  DEPENDS 
    Eigen3
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/reggie_episode.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp stdc++fs)

#############
## Install ##
#############
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

#############
## Testing ##
#############
