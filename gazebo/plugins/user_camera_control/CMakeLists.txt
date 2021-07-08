
add_library(user_camera_control SHARED ${headers_MOC} user_camera_control.cpp)
target_link_libraries(user_camera_control ${GAZEBO_LIBRARIES} Qt5::Widgets)