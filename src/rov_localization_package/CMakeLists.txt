cmake_minimum_required(VERSION 3.10)

get_filename_component(PACKAGE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message("Creating Package: ${PACKAGE_NAME}")

project(${PACKAGE_NAME})

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
	add_compile_options(-w)
endif()

set(ROS_DEPENDS
  roscpp
  roslib
  std_msgs
  std_srvs
  sensor_msgs
  rov_msgs_package
)

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDS})

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    ${ROS_DEPENDS}
  DEPENDS
)

add_subdirectory(hydroacoustic_localization)

set(SOURCES
)

set(NODE
    nodes/hydroacoustic_localization_node.cpp
)

get_filename_component(NODE_NAME ${NODE} NAME_WE)
message("bilding - " ${NODE_NAME})

add_executable(${NODE_NAME} ${NODE} ${SOURCES})

target_include_directories(${NODE_NAME}
        PRIVATE
		include
		${catkin_INCLUDE_DIRS}
)

target_link_libraries(${NODE_NAME}
        PRIVATE
		hydroacoustic_localization
		${catkin_LIBRARIES}
)

target_compile_definitions(
		${NODE_NAME}
		PRIVATE ROS_PACKAGE_NAME="${PROJECT_NAME}"
)

install(TARGETS ${NODE_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY map/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/map
)
