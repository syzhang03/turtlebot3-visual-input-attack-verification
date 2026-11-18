#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${TB3_WS:-$HOME/turtlebot3_ws}/install/setup.bash"
cd "${TB3_PROJECT_WS:-$PROJECT_DIR}"
colcon build --symlink-install --base-paths "$PROJECT_DIR/src" \
  --packages-up-to tb3_camera_lidar_monitor tb3_lidar_attack
