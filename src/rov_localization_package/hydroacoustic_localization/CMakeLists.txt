cmake_minimum_required(VERSION 3.10)

message("Creating hydracoustic localization library")

project(hydroacoustic_localization)

find_package(PCL 1.12 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
        include
)

set(SOURCES
        src/transform.cpp
        src/mather_shell.cpp
        src/diagnostic_extractor.cpp
        src/scan_mather_manager.cpp
        src/utils.cpp
)

add_library(${PROJECT_NAME} STATIC ${SOURCES})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        include
)

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        ${PCL_LIBRARIES}
        Eigen3::Eigen
        ${YAML_CPP_LIBRARIES}
        stdc++fs
)
