#!/usr/bin/env bash
# Optional ROS 2 evaluation recording (run in a separate terminal alongside Gazebo launches).
set -eo pipefail
if [[ ! -x "$(command -v ros2)" ]]; then
  echo "ERROR: ros2 CLI not available (source ROS + workspace)." >&2
  exit 1
fi

OUT_REL="${EVAL_ROSBAG_DIR:-runs/evaluation/bags/active}"
stamp="$(date -u +%Y-%m-%dT%H-%M-%SZ)"
OUT_PATH="${EVAL_ROSBAG_PREFIX:-$OUT_REL}_$stamp"

topics=(
  /clock
  /drone/position
  /interception/impact_event
  /interceptor_0/cmd_velocity /interceptor_1/cmd_velocity /interceptor_2/cmd_velocity
  /interceptor_0/position /interceptor_1/position /interceptor_2/position
  /interceptor/selected_id
)
exec ros2 bag record "${topics[@]}" -o "${OUT_PATH}"
