cmake_minimum_required(VERSION 3.10)

get_filename_component(PACKAGE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message("Creating Package: ${PACKAGE_NAME}")

project(${PACKAGE_NAME})

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)


FILE(GLOB rosMsgFiles RELATIVE "${${PROJECT_NAME}_SOURCE_DIR}/msg"
    "${${PROJECT_NAME}_SOURCE_DIR}/msg/*.msg")

FOREACH(rosMsg ${rosMsgFiles})
    MESSAGE(STATUS "Processing message file: ${rosMsg}" )
    add_message_files(FILES ${rosMsg})
ENDFOREACH(rosMsg)

FILE(GLOB rosSrvFiles RELATIVE "${${PROJECT_NAME}_SOURCE_DIR}/srv"
    "${${PROJECT_NAME}_SOURCE_DIR}/srv/*.srv")

FOREACH(rosSrv ${rosSrvFiles})
    MESSAGE(STATUS "Processing message file: ${rosSrv}" )
    add_service_files(FILES ${rosSrv})
ENDFOREACH(rosMsg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
