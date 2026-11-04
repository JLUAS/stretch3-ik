#!/usr/bin/env bash
# Unified manipulation bringup wrapper.
# Usage: ./run_manipulation.sh sim:=true   (or sim:=false)
#
# Sets ROS_DOMAIN_ID before invoking the launch system so that
# the ros2 daemon, parameter server, and all child nodes share
# the same DDS domain.
set -euo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"

sim=true
scenario=""
dry_run_set=false
collision_environment_set=false
passthrough=()
for arg in "$@"; do
  case "$arg" in
    sim:=true)  sim=true ;;
    sim:=false) sim=false ;;
    scenario:=*) scenario="${arg#scenario:=}" ;;
    dry_run:=*) dry_run_set=true; passthrough+=("$arg") ;;
    collision_environment:=*) collision_environment_set=true; passthrough+=("$arg") ;;
    *)          passthrough+=("$arg") ;;
  esac
done

if [ "$sim" = true ] && [ -z "$scenario" ]; then
  scenario=small_house
fi

if [ "$sim" = true ]; then
  export ROS_DOMAIN_ID=20
  export ROS_LOCALHOST_ONLY=1
  export STRETCH_RUNTIME=sim
else
  export ROS_DOMAIN_ID=42
  export ROS_LOCALHOST_ONLY=0
  export STRETCH_RUNTIME=real
fi

echo "=== Stretch Custom Manipulation ==="
echo "Mode:           $([ "$sim" = true ] && echo SIMULATION || echo REAL)"
if [ "$sim" = true ]; then echo "Scenario:       $scenario"; fi
echo "ROS_DOMAIN_ID:  $ROS_DOMAIN_ID"
echo "==================================="

set +u
source /opt/ros/humble/setup.bash
if [ "$sim" = false ]; then
  robot_workspace="${STRETCH_ROS_WS:-${HOME}/ament_ws}"
  if [ -f "${robot_workspace}/install/setup.bash" ]; then
    source "${robot_workspace}/install/setup.bash"
  fi
fi
if [ "$sim" = true ] && [ "$scenario" = small_house ]; then
  simulation_workspace="${STRETCH_SMALL_HOUSE_WS:-$(dirname "$workspace")/stretch3-sim-to-real-phase1}"
  if [ ! -f "${simulation_workspace}/install/setup.bash" ]; then
    echo "Missing Stretch Small House install: ${simulation_workspace}/install/setup.bash" >&2
    exit 2
  fi
  export STRETCH_SMALL_HOUSE_REPO="$simulation_workspace"
  source "${simulation_workspace}/install/setup.bash"
fi
if [ -f "${workspace}/install/setup.bash" ]; then
  source "${workspace}/install/setup.bash"
fi
set -u

# Kill any ros2 daemon on the wrong domain to avoid cross-domain leakage.
ros2 daemon stop 2>/dev/null || true

launch_args=("sim:=$sim")
if [ -n "$scenario" ]; then launch_args+=("scenario:=$scenario"); fi
if [ "$sim" = false ] && [ "$dry_run_set" = false ]; then
  launch_args+=("dry_run:=true")
  echo "Real-hardware default: dry_run:=true (pass dry_run:=false explicitly to arm execution)"
fi
if [ "$sim" = false ] && [ "$collision_environment_set" = false ]; then
  launch_args+=("collision_environment:=octomap")
  echo "Real-hardware default: collision_environment:=octomap"
fi
exec ros2 launch stretch_custom_manipulation bringup.launch.py \
  "${launch_args[@]}" "${passthrough[@]+"${passthrough[@]}"}"
