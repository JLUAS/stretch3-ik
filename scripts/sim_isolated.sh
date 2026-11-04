#!/bin/bash
# Run a command where the physical robot is unreachable: private network
# namespace (loopback only), private /dev/shm (no Fast DDS shared-memory
# discovery with host processes), ROS domain 20, localhost-only discovery.
# Refuses to run the command if any of that cannot be proven inside.
# Usage: ws/scripts/sim_isolated.sh <command> [args...]
set -euo pipefail
ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec unshare --user --map-root-user --net --mount bash -c '
set -eo pipefail
ws="$1"; shift
ip link set lo up
mount -t tmpfs tmpfs /dev/shm
mount -t sysfs sysfs /sys  # /sys/class/net must show this namespace
fail() { echo "ISOLATION FAILED: $*" >&2; exit 3; }
[ "$(ip -o link | grep -vc " lo:")" = 0 ] && [ "$(ls /sys/class/net)" = lo ] || fail "non-loopback interface present"
ip route get "${ROBOT_IP:-192.168.12.3}" >/dev/null 2>&1 && fail "route to the robot exists"
ip route get 10.0.0.1 >/dev/null 2>&1 && fail "default route exists"
[ -z "$(ls -A /dev/shm)" ] || fail "/dev/shm is shared"
export ROS_DOMAIN_ID=20 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI ROS_DISCOVERY_SERVER
echo "ISOLATED: netns=$(readlink /proc/self/ns/net) links=$(ip -o link | cut -d: -f2 | xargs) ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY RMW=$RMW_IMPLEMENTATION" >&2
set +u
source /opt/ros/humble/setup.bash
source "$ws/install/setup.bash"
exec "$@"
' bash "$ws" "$@"
