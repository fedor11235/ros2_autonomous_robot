#!/usr/bin/env bash
# Source ROS 2 and the built workspace, then exec the container command.
set -e
source /opt/ros/humble/setup.bash
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"
