#!/bin/bash
# Start the isolated grasp stack and run seeded trials inside one private
# network namespace. Usage:
#   run_sim.sh <log_dir> <mission_script> <trials> [run_trials.py args...]
# Must be invoked through ws/scripts/sim_isolated.sh.
set -eo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
log="$1"; script="$2"; trials="$3"; shift 3
[ "$ROS_DOMAIN_ID" = 20 ] && [ "$ROS_LOCALHOST_ONLY" = 1 ] || { echo "not isolated" >&2; exit 3; }
mkdir -p "$log"
export ROS_LOG_DIR="$log/ros"
ros2 launch "$here/grasp_sim.launch.py" mission_script:="$script" \
  ${TOLERANCE:+grasp_approach_tolerance:=$TOLERANCE} ${MISSION_PARAMS:+mission_params:=$MISSION_PARAMS} ${IK_PREFIX:+ik_preview_prefix:=$IK_PREFIX} \
  > "$log/stack.log" 2>&1 &
stack=$!
trap 'kill -INT $stack 2>/dev/null; sleep 3; kill -TERM $stack 2>/dev/null; wait' EXIT
python3 "${RUNNER:-$here/run_trials.py}" --trials "$trials" --out "$log/trials.jsonl" "$@" 2>&1 | tee "$log/runner.log"
