cmake_minimum_required(VERSION 3.16)
project(stretch_collision_planning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(stretch_custom_kinematics REQUIRED)
find_package(stretch_robot_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/collision_geometry.cpp
  src/octomap_environment.cpp
  src/octomap_scene_store.cpp
  src/primitive_environment.cpp
  src/robot_collision_model.cpp
  src/rrt_connect.cpp
  src/state_validity.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen ${OCTOMAP_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}
  octomap_msgs
  stretch_custom_kinematics
  stretch_robot_interface)

add_executable(cspace_slice src/cspace_slice.cpp)
target_link_libraries(cspace_slice ${PROJECT_NAME})

add_executable(planning_demo src/planning_demo.cpp)
target_link_libraries(planning_demo ${PROJECT_NAME})
ament_target_dependencies(planning_demo ament_index_cpp)

add_executable(octomap_builder src/octomap_builder_node.cpp)
target_link_libraries(octomap_builder ${PROJECT_NAME})
ament_target_dependencies(octomap_builder
  nav_msgs
  octomap_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS cspace_slice planning_demo octomap_builder
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_collision_validity test/test_collision_validity.cpp)
  target_link_libraries(test_collision_validity ${PROJECT_NAME})
  target_compile_definitions(test_collision_validity PRIVATE
    TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../stretch_custom_kinematics/test/robot.urdf"
    TEST_LIMITS="${CMAKE_CURRENT_SOURCE_DIR}/../stretch_custom_kinematics/config/hardware_limits.csv")
  ament_add_gtest(test_rrt_connect test/test_rrt_connect.cpp)
  target_link_libraries(test_rrt_connect ${PROJECT_NAME})
  target_compile_definitions(test_rrt_connect PRIVATE
    TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../stretch_custom_kinematics/test/robot.urdf"
    TEST_LIMITS="${CMAKE_CURRENT_SOURCE_DIR}/../stretch_custom_kinematics/config/hardware_limits.csv")
  ament_add_gtest(test_octomap_environment test/test_octomap_environment.cpp)
  target_link_libraries(test_octomap_environment ${PROJECT_NAME})
  target_compile_definitions(test_octomap_environment PRIVATE
    TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../stretch_custom_kinematics/test/robot.urdf"
    TEST_LIMITS="${CMAKE_CURRENT_SOURCE_DIR}/../stretch_custom_kinematics/config/hardware_limits.csv")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 ament_index_cpp octomap octomap_msgs stretch_custom_kinematics stretch_robot_interface)
ament_package()
