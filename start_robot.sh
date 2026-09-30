#!/usr/bin/env bash
# Start the robot in a mode:
#   ./start_robot.sh            command mode (voice + drivetrain) — what systemd runs
#   ./start_robot.sh command    same
#   ./start_robot.sh explore    explore mode (SLAM + plant photos, no voice)
# Anything after the mode is passed to `ros2 launch`, e.g. `voice:=false`.
#
# Exists because systemd cannot `source` a setup file, and ROS 2 is entirely
# built around sourcing two of them: the distro overlay and this workspace's.
# Without both, `ros2 launch` is not on PATH and robot_interfaces cannot be
# imported.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MODE=command
case "${1:-}" in
  command|explore) MODE="$1"; shift ;;
  -h|--help) sed -n '2,6p' "$0"; exit 0 ;;
esac

source /opt/ros/jazzy/setup.bash
source "$REPO/ros2_ws/install/setup.bash"

# Keep DDS off the wifi. Without this every ROS process multicasts to the whole
# subnet looking for peers, which costs real CPU on a Pi and finds your laptop.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export PYTHONUNBUFFERED=1

cd "$REPO"
exec ros2 launch robot_bringup "$MODE.launch.py" "$@"
