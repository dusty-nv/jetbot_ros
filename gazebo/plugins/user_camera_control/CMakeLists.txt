
add_library(gazebo_user_camera_control SHARED ${headers_MOC} user_camera_control.cpp)
target_link_libraries(gazebo_user_camera_control ${GAZEBO_LIBRARIES} Qt5::Widgets)
install(TARGETS gazebo_user_camera_control DESTINATION lib)