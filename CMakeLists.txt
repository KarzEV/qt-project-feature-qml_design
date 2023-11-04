cmake_minimum_required(VERSION 3.22)
project(qt-gui)

set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOMOC ON)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Qt5 COMPONENTS COMPONENTS Widgets Qml Quick Charts OpenGL REQUIRED)

include_directories(${Qt5Widgets_INCLUDE_DIRS} ${QtQml_INCLUDE_DIRS})
add_definitions(${Qt5Widgets_DEFINITIONS} ${QtQml_DEFINITIONS} ${${Qt5Quick_DEFINITIONS}})

add_executable(${PROJECT_NAME}
        src/main.cpp
        src/cloudsvisualizer.cpp
        src/cloudsvisualizer.h
        src/cloudsdrawler.cpp
        src/cloudsdrawler.h
        src/modeldrawler.cpp
        src/modeldrawler.h
        src/basedrawler.cpp
        src/basedrawler.h
        src/utils.cpp
        src/utils.h
        src/types.h
)

target_link_libraries(${PROJECT_NAME}
        PRIVATE
        Qt5::Core
        Qt5::Qml
        Qt5::Quick
        Qt5::Charts
        Qt5::OpenGL
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
)


