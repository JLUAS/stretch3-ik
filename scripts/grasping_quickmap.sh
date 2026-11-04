#!/bin/bash
# Map of what the lidar sees from where the robot stands (no driving).
# The map origin is the robot's current pose, so the grasping bringup's
# default AMCL pose (0, 0, 0) is right when it starts from this same spot.
#   ~/ament_ws/scripts/grasping_quickmap.sh [~/maps/grasping_area]
OUT=${1:-$HOME/maps/grasping_area}
export HELLO_FLEET_PATH=${HELLO_FLEET_PATH:-$HOME/stretch_user} HELLO_FLEET_ID=${HELLO_FLEET_ID:?set HELLO_FLEET_ID to the robot serial, e.g. stretch-se3-xxxx} PATH=$HOME/.local/bin:$PATH
source /opt/ros/humble/setup.bash
source ~/ament_ws/install/setup.bash
export ROS_DOMAIN_ID=42
# Save while SLAM runs: ~20 s to start, then ~25 s of stationary scans.
( sleep 45
  ros2 run nav2_map_server map_saver_cli -f "$OUT" --ros-args -p save_map_timeout:=10.0
) &
# SIGINT after 60 s = Ctrl-C: orderly shutdown, the lidar motor stops.
timeout -s INT -k 20 60 ros2 launch "$(dirname "$0")/quickmap.launch.py"
wait
ls -la "$OUT".yaml "$OUT".pgm && cat "$OUT".yaml
