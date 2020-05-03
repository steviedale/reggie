cmake_minimum_required(VERSION 3.0.2)
project(reggie_driver)

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
)

add_service_files(DIRECTORY srv FILES WheelCommand.srv)
generate_messages(DEPENDENCIES)
catkin_package(CATKIN_DEPENDS message_runtime)

catkin_install_python(PROGRAMS
  scripts/wheel_controller.py
  scripts/simple_wheel_controller.py
  scripts/simple_wheel_command_listener.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${PROJECT_NAME}
)
