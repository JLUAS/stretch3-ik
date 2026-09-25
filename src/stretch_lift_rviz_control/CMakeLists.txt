cmake_minimum_required(VERSION 3.16)
project(stretch_lift_rviz_control)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(stretch_robot_interface REQUIRED)
find_package(stretch_custom_kinematics REQUIRED)
find_package(stretch_guarded_executor REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(lift_marker src/lift_marker.cpp)
ament_target_dependencies(lift_marker rclcpp interactive_markers visualization_msgs
  geometry_msgs sensor_msgs std_msgs std_srvs ament_index_cpp
  stretch_robot_interface stretch_custom_kinematics)

add_library(lift_control_panel SHARED
  src/lift_control_panel.cpp
  include/stretch_lift_rviz_control/lift_control_panel.hpp)
set_target_properties(lift_control_panel PROPERTIES AUTOMOC ON)
target_include_directories(lift_control_panel PRIVATE include)
target_link_libraries(lift_control_panel Qt5::Widgets)
ament_target_dependencies(lift_control_panel rviz_common pluginlib rclcpp
  rclcpp_action std_msgs std_srvs stretch_guarded_executor)
pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)

install(TARGETS lift_marker DESTINATION lib/${PROJECT_NAME})
install(TARGETS lift_control_panel LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz config DESTINATION share/${PROJECT_NAME})
ament_package()
