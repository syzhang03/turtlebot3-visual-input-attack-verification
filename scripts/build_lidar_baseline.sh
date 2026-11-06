#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${TB3_WS:-$HOME/turtlebot3_ws}/install/setup.bash"
cd "${TB3_PROJECT_WS:-$PROJECT_DIR}"
colcon build --symlink-install --base-paths "$PROJECT_DIR/src" --packages-select tb3_lidar_baseline
