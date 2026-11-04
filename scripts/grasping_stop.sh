#!/bin/bash
# Stop the grasping bringup like Ctrl-C in its terminal: SIGINT to ros2 launch,
# which shuts every node down in order (the lidar driver stops its motor).
# SIGTERM only if it is still running after 25 s.
pkill -INT -f "ros2 launch stretch_custom_manipulation bringup.launch.py"
for i in $(seq 1 25); do
  pgrep -f "ros2 launch stretch_custom_manipulation bringup.launch.py" > /dev/null || break
  sleep 1
done
pkill -TERM -f "ros2 launch stretch_custom_manipulation bringup.launch.py"
sleep 3
echo "nodes still running: $(pgrep -fc 'sllidar_node|stretch_driver|realsense2_camera_node|guarded_executor|controller_server|ik_preview')"
