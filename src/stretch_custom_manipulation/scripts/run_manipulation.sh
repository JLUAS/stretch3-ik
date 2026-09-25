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
passthrough=()
for arg in "$@"; do
  case "$arg" in
    sim:=true)  sim=true ;;
    sim:=false) sim=false ;;
    *)          passthrough+=("$arg") ;;
  esac
done

if [ "$sim" = true ]; then
  export ROS_DOMAIN_ID=20
  export STRETCH_RUNTIME=sim
else
  export ROS_DOMAIN_ID=42
  export STRETCH_RUNTIME=real
fi

# Kill any ros2 daemon on the wrong domain to avoid cross-domain leakage.
ros2 daemon stop 2>/dev/null || true

echo "=== Stretch Custom Manipulation ==="
echo "Mode:           $([ "$sim" = true ] && echo SIMULATION || echo REAL)"
echo "ROS_DOMAIN_ID:  $ROS_DOMAIN_ID"
echo "==================================="

set +u
source /opt/ros/humble/setup.bash
if [ -f "${workspace}/install/setup.bash" ]; then
  source "${workspace}/install/setup.bash"
fi
set -u

exec ros2 launch stretch_custom_manipulation bringup.launch.py \
  sim:="$sim" "${passthrough[@]+"${passthrough[@]}"}"
