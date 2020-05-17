cmake_minimum_required(VERSION 2.8.3)
project(reggie_support)

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
)

add_service_files(
  DIRECTORY srv 
  FILES 
    WheelCommand.srv
    EpisodeElement.srv
)
generate_messages(DEPENDENCIES)
catkin_package(CATKIN_DEPENDS message_runtime)

install(DIRECTORY launch/ rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
