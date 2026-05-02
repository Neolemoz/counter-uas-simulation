#!/usr/bin/env bash
# รัน C-UAS Gazebo + world ให้ถูกต้อง (source ROS + workspace อัตโนมัติ)
# Usage:
#   ./scripts/run_gazebo_cuas.sh
#   ./scripts/run_gazebo_cuas.sh --multi
#   ./scripts/run_gazebo_cuas.sh use_gazebo_gui:=false
#   ./scripts/run_gazebo_cuas.sh --multi use_rviz:=true
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$WS_ROOT"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
else
  echo "ERROR: ไม่พบ /opt/ros/jazzy หรือ humble — ติดตั้ง ROS 2 ก่อน" >&2
  exit 1
fi

if [[ ! -f "$WS_ROOT/install/setup.bash" ]]; then
  echo "ERROR: ยังไม่มี install/setup.bash — รัน: cd $WS_ROOT && colcon build --symlink-install" >&2
  exit 1
fi
# shellcheck source=/dev/null
source "$WS_ROOT/install/setup.bash"

MULTI=false
LAUNCH_ARGS=()
for arg in "$@"; do
  if [[ "$arg" == "--multi" ]]; then
    MULTI=true
  else
    LAUNCH_ARGS+=("$arg")
  fi
done

if $MULTI; then
  exec ros2 launch gazebo_target_sim gazebo_target_multi.launch.py "${LAUNCH_ARGS[@]}"
else
  exec ros2 launch gazebo_target_sim gazebo_target.launch.py "${LAUNCH_ARGS[@]}"
fi
