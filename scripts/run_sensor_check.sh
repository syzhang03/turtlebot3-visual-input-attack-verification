#!/usr/bin/env bash
# Passive liveness check: camera + LiDAR + IMU + wheel speed together.
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DURATION="${1:-10}"
[[ $# -eq 0 ]] || shift
source /opt/ros/jazzy/setup.bash
source "${TB3_WS:-$HOME/turtlebot3_ws}/install/setup.bash"
source "$PROJECT_DIR/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-33}"
exec ros2 run tb3_sensor_bringup check_sensors --seconds "$DURATION" "$@"
