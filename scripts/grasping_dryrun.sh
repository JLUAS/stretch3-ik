#!/bin/bash
# Grasping bringup in DRY RUN (no arm/gripper/head/base motion). Ctrl-C stops it.
export HELLO_FLEET_PATH=${HELLO_FLEET_PATH:-$HOME/stretch_user} HELLO_FLEET_ID=${HELLO_FLEET_ID:?set HELLO_FLEET_ID to the robot serial, e.g. stretch-se3-xxxx} PATH=$HOME/.local/bin:$PATH
cd ~/ament_ws
# Every run keeps its full output for later diagnosis.
mkdir -p ~/grasping_logs
LOG=~/grasping_logs/$(date +%Y%m%d_%H%M%S)_$(basename "$0" .sh).log
./src/stretch_custom_manipulation/scripts/run_manipulation.sh sim:=false use_slam:=false \
  map:=$HOME/maps/grasping_area.yaml dry_run:=true rviz:=true 2>&1 | tee "$LOG"
