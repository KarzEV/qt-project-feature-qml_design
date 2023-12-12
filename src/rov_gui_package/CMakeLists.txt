cmake_minimum_required(VERSION 3.10)

get_filename_component(PACKAGE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message("Creating Package: ${PACKAGE_NAME}")

project(${PACKAGE_NAME})

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

set(ROS_DEPENDS
  roscpp
  std_msgs
  std_srvs
)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Qt5 COMPONENTS COMPONENTS Widgets Qml Quick Charts OpenGL REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDS})

add_definitions(${Qt5Widgets_DEFINITIONS} ${QtQml_DEFINITIONS} ${${Qt5Quick_DEFINITIONS}})

catkin_package(
  INCLUDE_DIRS
    include
    ${OPENGL_INCLUDE_DIR}
  LIBRARIES
    ${OPENGL_LIBRARIES}
  CATKIN_DEPENDS
    ${ROS_DEPENDS}
  DEPENDS
    Qt5Widgets
    QtQml	
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${QtQml_INCLUDE_DIRS}
)

set(SOURCES
    src/cloudsvisualizer.cpp
    src/cloudsdrawler.cpp
    src/modeldrawler.cpp
    src/basedrawler.cpp
    src/utils.cpp
    src/mainwindow.cpp
    src/vehiclestatetable.cpp
    src/attitudeindicator.cpp
    src/compass.cpp
    src/testwidget.cpp
)

set(MOC_HEADERS
    include/rov_gui_package/cloudsvisualizer.h
    include/rov_gui_package/cloudsdrawler.h
    include/rov_gui_package/modeldrawler.h
    include/rov_gui_package/basedrawler.h
    include/rov_gui_package/utils.h
    include/rov_gui_package/types.h
    include/rov_gui_package/mainwindow.h
    include/rov_gui_package/vehiclestatetable.h
    include/rov_gui_package/attitudeindicator.h
    include/rov_gui_package/compass.h
    include/rov_gui_package/testwidget.h
)

set(UIS
    src/mainwindow.ui
    src/vehiclestatetable.ui
    src/testwidget.ui
)

set(RESOURCES
    src/models.qrc
)

set(NODE
    nodes/gui_node.cpp
)

qt5_wrap_ui(SOURCES ${UIS})
qt5_add_resources(RCC_SOURCES ${RESOURCES})

get_filename_component(NODE_NAME ${NODE} NAME_WE)
message("bilding - " ${NODE_NAME})

add_executable(${NODE_NAME} ${NODE} ${MOC_HEADERS} ${SOURCES} ${MOC_SRCS} ${UIS} ${RCC_SOURCES})

target_include_directories(${NODE_NAME}
        PRIVATE
        ${PROJECT_BINARY_DIR}
)

target_link_libraries(${NODE_NAME}
        PRIVATE
        Qt5::Core
        Qt5::Qml
        Qt5::Quick
        Qt5::Charts
        Qt5::OpenGL
        Qt5::Widgets
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
	${catkin_LIBRARIES}
)

install(TARGETS ${NODE_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
