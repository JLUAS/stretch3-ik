#!/bin/bash
# Copy this repository's packages and scripts into the robot workspace and
# build them there. Starts nothing on the robot and sends no commands to the
# driver. Only the packages in ./src are replaced; every other package in the
# robot workspace (driver, Nav2, cameras) is left alone.
#
# Refuses to overwrite robot copies that changed since BASELINE, a local copy
# of the robot workspace taken before your last deploy (same layout: src/<pkg>,
# scripts/). Without BASELINE it refuses unless FORCE=1.
#
#   ROBOT=hello-robot@<robot-ip> BASELINE=~/robot_snapshot ./scripts/deploy_to_robot.sh
set -euo pipefail
ROBOT=${ROBOT:?set ROBOT=user@robot-ip}
WS=${WS:-ament_ws}
HERE=$(cd "$(dirname "$0")/.." && pwd)
STAMP=$(date +%Y%m%d_%H%M)
pkgs=$(cd "$HERE/src" && ls -d */ | tr -d / | xargs)
dirs="scripts $(for p in $pkgs; do printf 'src/%s ' "$p"; done)"

list="find $dirs -type f ! -path '*/__pycache__/*' -exec md5sum {} + 2>/dev/null | sort -k2"
if [ -n "${BASELINE:-}" ]; then
  echo "Checking the robot copies against $BASELINE..."
  robot_sums=$(ssh "$ROBOT" "cd ~/$WS && $list" || true)
  base_sums=$(cd "$BASELINE" && eval "$list" || true)
  if [ "$robot_sums" != "$base_sums" ]; then
    diff <(echo "$base_sums") <(echo "$robot_sums") | head -20
    echo "Robot copies changed since the baseline; merge them first." >&2
    exit 1
  fi
elif [ "${FORCE:-0}" != 1 ]; then
  echo "No BASELINE given: set BASELINE=<local copy of the robot workspace> or FORCE=1." >&2
  exit 1
fi

echo "Backing up ~/$WS/{$dirs} on the robot..."
ssh "$ROBOT" "cd ~/$WS && tar czf ~/${WS}_backup_$STAMP.tgz --ignore-failed-read $dirs"
for d in $dirs; do
  # Staging and old copies live outside the workspace: a second package.xml
  # under src/ would break the colcon build.
  new="\$HOME/${WS}_new_$STAMP/$d" old="\$HOME/${WS}_old_$STAMP/$d"  # expanded on the robot
  ssh "$ROBOT" "mkdir -p $new $(dirname "$old")"
  tar czf - -C "$HERE/$d" --exclude=__pycache__ . | ssh "$ROBOT" "tar xzf - -C $new"
  ssh "$ROBOT" "cd ~/$WS && { [ ! -e $d ] || mv $d $old; } && mv $new $d"
done
ssh "$ROBOT" "rm -rf \$HOME/${WS}_new_$STAMP"

echo "Building on the robot..."
ssh "$ROBOT" "cd ~/$WS && source /opt/ros/humble/setup.bash &&
  colcon build --symlink-install --packages-select $pkgs --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
echo "Deployed. Backup: ~/${WS}_backup_$STAMP.tgz; old copies: ~/${WS}_old_$STAMP"
