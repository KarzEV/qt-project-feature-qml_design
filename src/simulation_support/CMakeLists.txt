cmake_minimum_required(VERSION 3.10)

get_filename_component(PACKAGE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message("Creating Package: ${PACKAGE_NAME}")

project(${PACKAGE_NAME})

find_package(
  catkin REQUIRED
  COMPONENTS
    rospy
    gazebo_msgs
    tf2
)

catkin_package()

catkin_install_python(PROGRAMS nodes/keyboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
