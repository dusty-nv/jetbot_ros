cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(Qt5Core REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${Qt5Core_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(user_camera_control SHARED user_camera_control.cpp)
target_link_libraries(user_camera_control ${GAZEBO_LIBRARIES})