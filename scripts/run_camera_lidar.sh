#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-observe}"
case "$MODE" in
  observe) ENABLED=false ;;
  drive) ENABLED=true ;;
  *) echo 'Usage: run_camera_lidar.sh [observe|drive] [ROS arguments...]' >&2; exit 2 ;;
esac
[[ $# -eq 0 ]] || shift
source /opt/ros/jazzy/setup.bash
source "${TB3_WS:-$HOME/turtlebot3_ws}/install/setup.bash"
source "${TB3_PROJECT_WS:-$PROJECT_DIR}/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-33}"
exec ros2 run tb3_camera_lidar_monitor camera_lidar_monitor --ros-args \
  --params-file "$PROJECT_DIR/configs/camera_lidar.yaml" -p "enabled:=$ENABLED" "$@"
