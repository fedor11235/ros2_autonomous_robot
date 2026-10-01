#!/usr/bin/env bash
# One-command launcher for a native (non-Docker) ROS 2 Humble install.
# Builds the workspace if needed, sources it, and starts the full simulation.
#
#   ./run.sh                 # Gazebo + RViz + autonomy
#   ./run.sh gui:=false      # headless Gazebo
set -e

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  echo "ERROR: ROS 2 ${ROS_DISTRO} not found at /opt/ros/${ROS_DISTRO}." >&2
  echo "Install ROS 2 Humble + Gazebo Classic, or use Docker (see README)." >&2
  exit 1
fi

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ ! -f "install/setup.bash" ]; then
  echo ">> Building workspace (first run)..."
  colcon build --symlink-install
fi

# shellcheck disable=SC1091
source install/setup.bash

echo ">> Launching simulation..."
exec ros2 launch robot_bringup bringup.launch.py "$@"
