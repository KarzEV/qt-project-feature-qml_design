cmake_minimum_required(VERSION 3.12)

message("Creating hydracoustic localization library")

project(hydroacoustic_localization)

find_package(PCL 1.12 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

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
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        ${PCL_LIBRARIES}
        Eigen3::Eigen
        ${YAML_CPP_LIBRARIES}
        stdc++fs
)

target_compile_definitions(
        ${PROJECT_NAME}
        PUBLIC ${PCL_DEFINITIONS}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${PCL_COMPILE_OPTIONS})

