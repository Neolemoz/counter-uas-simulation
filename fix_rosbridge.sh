#!/usr/bin/env bash
# Clean ROS 2 Jazzy + rosbridge (no overlay pollution).
#
# Default: NO apt/sudo — env + ABI check + launch.
# One-time fix (FastCDR vs rosbridge ABI):  ./fix_rosbridge.sh --upgrade
# Upgrade only, no launch:                 ./fix_rosbridge.sh --upgrade-only
#
# Typical failures:
# - libfastcdr too old for rosbridge-msgs → undefined symbol …serializeEj
# - RuntimeError: This member is not been selected → from libfastrtps (Fast DDS);
#   usually partial upgrades (fastcdr/rosbridge newer than fastrtps/rmw/rclpy).
#   Run --upgrade so apt refreshes the whole Fast DDS + rmw + rclpy chain.

set -eo pipefail

WITH_UPGRADE=0
DO_LAUNCH=1
for a in "$@"; do
  case "$a" in
    --upgrade|--fix-packages|--apt) WITH_UPGRADE=1 ;;
    --upgrade-only|--apt-only) WITH_UPGRADE=1; DO_LAUNCH=0 ;;
    --help|-h)
      cat <<'EOF'
Usage: fix_rosbridge.sh [OPTION]

  (none)          Clean env, check FastCDR/rosbridge ABI, launch WebSocket bridge.
  --upgrade       Same + apt upgrade fastcdr + rosbridge (needs sudo), then launch.
  --upgrade-only  Only run apt upgrade + ABI check; do not launch.

If launch dies with "undefined symbol" in librosbridge_msgs__rosidl_typesupport_fastrtps_c.so,
your ros-jazzy-fastcdr is older than the rosbridge-msgs build — use --upgrade once.

If rclpy raises "This member is not been selected", Fast DDS (libfastrtps) is out of sync
with the rest of the stack — run --upgrade once (it upgrades fastrtps + rmw + rclpy too).
EOF
      exit 0
      ;;
  esac
done

rosbridge_typesupport_ok() {
  # Same load order as the bridge: fastcdr then rosbridge_msgs FastRTPS typesupport.
  python3 <<'PY'
import ctypes
import sys

def main() -> int:
    try:
        ctypes.CDLL("/opt/ros/jazzy/lib/libfastcdr.so")
        ctypes.CDLL("/opt/ros/jazzy/lib/librosbridge_msgs__rosidl_typesupport_fastrtps_c.so")
    except OSError:
        return 1
    return 0

raise SystemExit(main())
PY
}

echo "==> Clearing overlay-related env"
unset AMENT_PREFIX_PATH || true
unset CMAKE_PREFIX_PATH || true
unset COLCON_PREFIX_PATH || true
unset LD_LIBRARY_PATH || true
unset LD_PRELOAD || true
unset PYTHONPATH || true
unset RMW_IMPLEMENTATION || true

echo "==> Sourcing ONLY /opt/ros/jazzy/setup.bash"
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash

# Let ros-jazzy-rmw-implementation pick the default; avoids stale RMW_* from the shell.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
echo "==> RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"

if [[ "$WITH_UPGRADE" -eq 1 ]]; then
  echo "==> Upgrading packages (sudo; can take a few minutes — progress is shown)"
  echo "    If it seems stuck: enter your sudo password, or run 'sudo -v' first."
  sudo -v
  sudo DEBIAN_FRONTEND=noninteractive apt-get update
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
    -o Dpkg::Use-Pty=0 \
    -o Dpkg::Options::=--force-confdef \
    -o Dpkg::Options::=--force-confold \
    ros-jazzy-fastcdr \
    ros-jazzy-fastrtps \
    ros-jazzy-fastrtps-cmake-module \
    ros-jazzy-rmw-dds-common \
    ros-jazzy-rmw-fastrtps-cpp \
    ros-jazzy-rmw-fastrtps-shared-cpp \
    ros-jazzy-rmw-implementation \
    ros-jazzy-rcl \
    ros-jazzy-rclpy \
    ros-jazzy-rosapi \
    ros-jazzy-rosbridge-msgs \
    ros-jazzy-rosbridge-library \
    ros-jazzy-rosbridge-server
  sudo ldconfig
  echo "==> Installed versions:"
  dpkg-query -W -f='${Package}\t${Version}\n' \
    ros-jazzy-fastcdr \
    ros-jazzy-fastrtps \
    ros-jazzy-rmw-fastrtps-cpp \
    ros-jazzy-rclpy \
    ros-jazzy-rosbridge-msgs \
    ros-jazzy-rosbridge-server \
    2>/dev/null || true
else
  echo "==> Skipping apt (fast). If ABI check fails below, run once:"
  echo "    $0 --upgrade"
fi

export PYTHONNOUSERSITE=1

echo "==> Checking FastCDR vs rosbridge_msgs (FastRTPS typesupport)…"
if ! rosbridge_typesupport_ok; then
  echo ""
  echo "ERROR: ABI mismatch — librosbridge_msgs FastRTPS typesupport cannot load against libfastcdr."
  echo "This is not a hang: rosbridge would exit immediately with undefined symbol."
  echo ""
  echo "Installed (for reference):"
  dpkg-query -W -f='  ${Package} ${Version}\n' \
    ros-jazzy-fastcdr ros-jazzy-rosbridge-msgs 2>/dev/null || true
  echo ""
  echo "Fix (needs sudo, one time):"
  echo "  $0 --upgrade"
  echo "Or upgrade only without launching:"
  echo "  $0 --upgrade-only"
  echo ""
  exit 1
fi

echo "==> rosbridge packages in this shell:"
ros2 pkg list | grep -i rosbridge || true

if [[ "$DO_LAUNCH" -eq 0 ]]; then
  echo "==> ABI OK — exiting (--upgrade-only, no launch)."
  exit 0
fi

echo "==> Launching WebSocket bridge (Ctrl+C to stop) — ws://127.0.0.1:9090 (all interfaces: 0.0.0.0)"
exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9090
