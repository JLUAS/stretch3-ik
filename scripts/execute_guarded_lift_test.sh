#!/usr/bin/env bash
# One explicit operator command; never invoked automatically by a launch file.
set -e
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"
export ROS_DOMAIN_ID=42
export STRETCH_RUNTIME=real
export ROS_LOCALHOST_ONLY=0
exec /usr/bin/python3 "${workspace_dir}/scripts/guarded_lift_test_once.py" "$@"
