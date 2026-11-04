cmake_minimum_required(VERSION 3.16)
project(stretch_custom_manipulation)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraspTarget.msg"
  "msg/GripperState.msg"
  "msg/ManipulationStatus.msg"
  "msg/ObjectObservation.msg"
  "action/GraspObject.action"
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs
)

install(DIRECTORY launch config rviz docs scripts worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/camera_scan.py scripts/autonomous_mission.py
  scripts/sim_gripper.py scripts/pick_state_machine.py scripts/place_state_machine.py
  scripts/grasp_mission.py scripts/grasp_planning.py scripts/gripper_bridge.py
  scripts/object_localizer.py scripts/sam2_segmenter.py
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(test_autonomous_mission test/test_autonomous_mission.py)
  ament_add_pytest_test(test_pick_state_machine test/test_pick_state_machine.py)
  ament_add_pytest_test(test_place_state_machine test/test_place_state_machine.py)
  ament_add_pytest_test(test_grasp_planning test/test_grasp_planning.py)
  ament_add_pytest_test(test_sam2_segmenter test/test_sam2_segmenter.py)
  ament_add_pytest_test(test_grasp_mission test/test_grasp_mission.py)
  ament_add_pytest_test(test_perception_chain test/test_perception_chain.py)
  add_launch_test(test/test_launch_sim.py)
endif()
ament_package()
