#!/usr/bin/env bash
# Read-only helper for HIT -> explosion debugging.
# Full guide: research/debug_hit_explosion_gazebo.md
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="${1:-}"

echo "=== HIT / Gazebo explosion debug (read-only) ==="
echo "Guide: ${ROOT}/research/debug_hit_explosion_gazebo.md"
echo

if [[ -n "$LOG" && -f "$LOG" ]]; then
  echo "=== Grep: ${LOG} ==="
  grep -E '\[HIT\]|\[RESULT\]|HIT_BLOCKED|GUIDANCE_WARN|HIT: publishing|Received .*stop.*True|EXPLOSION|Impact sequence' "$LOG" || true
  echo
elif [[ -n "$LOG" ]]; then
  echo "WARN: log file not found: $LOG"
  echo
fi

echo "=== Manual grep (paste log path) ==="
echo "  grep -E '\\\\[HIT\\\\]|\\\\[RESULT\\\\]|HIT_BLOCKED|EXPLOSION|Impact sequence' your.log"
echo

if command -v ros2 >/dev/null 2>&1; then
  echo "=== ros2 topic list (stop + hit markers) ==="
  ros2 topic list 2>/dev/null | grep -E '/(target/stop|target_[0-9]+/stop|interception/hit_markers)$' || true
  echo
  echo "Optional:"
  echo "  ros2 topic echo /target/stop --once"
  echo "  ros2 topic echo /interception/hit_markers --once"
else
  echo "ros2 not in PATH — source /opt/ros/<distro>/setup.bash and install/setup.bash for topic checks."
fi
