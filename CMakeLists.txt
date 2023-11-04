cmake_minimum_required(VERSION 3.22)
project(qt-gui)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Qt5 COMPONENTS COMPONENTS Widgets Qml Quick Charts OpenGL REQUIRED)

include_directories(${Qt5Widgets_INCLUDE_DIRS} ${QtQml_INCLUDE_DIRS})
add_definitions(${Qt5Widgets_DEFINITIONS} ${QtQml_DEFINITIONS} ${${Qt5Quick_DEFINITIONS}})

set(SOURCES
    src/main.cpp
    src/cloudsvisualizer.cpp
    src/cloudsdrawler.cpp
    src/modeldrawler.cpp
    src/basedrawler.cpp
    src/utils.cpp
    src/mainwindow.cpp
)

set(MOC_HEADERS
    src/cloudsvisualizer.h
    src/cloudsdrawler.h
    src/modeldrawler.h
    src/basedrawler.h
    src/utils.h
    src/types.h
    src/mainwindow.h
)

set(UIS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mainwindow.ui
)

qt5_wrap_ui(SOURCES ${UIS})

add_executable(${PROJECT_NAME} ${MOC_HEADERS} ${SOURCES} ${MOC_SRCS} ${UIS})

target_include_directories(${PROJECT_NAME}
        PRIVATE
        ${PROJECT_BINARY_DIR}
)

target_link_libraries(${PROJECT_NAME}
        PRIVATE
        Qt5::Core
        Qt5::Qml
        Qt5::Quick
        Qt5::Charts
        Qt5::OpenGL
        Qt5::Widgets
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
)
