cmake_minimum_required(VERSION 3.22)
project(qt-gui)

set(CMAKE_AUTORCC ON)

find_package(Qt5 COMPONENTS COMPONENTS Widgets Qml Quick Charts REQUIRED)

include_directories(${Qt5Widgets_INCLUDE_DIRS} ${QtQml_INCLUDE_DIRS})
add_definitions(${Qt5Widgets_DEFINITIONS} ${QtQml_DEFINITIONS} ${${Qt5Quick_DEFINITIONS}})


add_executable(${PROJECT_NAME}
        src/main.cpp
        resources/qml.qrc)

target_include_directories(${PROJECT_NAME}
        PRIVATE
        ${PROJECT_BINARY_DIR}
)

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        Qt5::Core
        Qt5::Qml
        Qt5::Quick
        Qt5::Charts
)


