#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${TB3_WS:-$HOME/turtlebot3_ws}/install/setup.bash"
source "$PROJECT_DIR/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-33}"
exec ros2 run tb3_visual_baseline baseline --ros-args \
  --params-file "$PROJECT_DIR/configs/baseline_white_line.yaml" \
  -p image_topic:=/attack/image_raw -p image_reliability:=reliable \
  -p enabled:=true
