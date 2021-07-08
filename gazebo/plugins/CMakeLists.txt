cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_AUTOMOC ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

find_package(gazebo REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED) 

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${Qt5Core_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_subdirectory(user_camera_control)
add_subdirectory(user_camera_control_system)