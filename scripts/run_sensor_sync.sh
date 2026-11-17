#!/usr/bin/env bash
# Run the wheel-speed converter and the four-sensor synchronised CSV logger.
# Start the scene first and wait for SCENE_READY.
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${TB3_WS:-$HOME/turtlebot3_ws}/install/setup.bash"
source "$PROJECT_DIR/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-33}"

ros2 run tb3_sensor_bringup wheel_speed --ros-args \
  --params-file "$PROJECT_DIR/configs/sensors.yaml" &
WHEEL_PID=$!
trap 'kill "$WHEEL_PID" 2>/dev/null || true' EXIT INT TERM

ros2 run tb3_sensor_bringup sensor_sync --ros-args \
  --params-file "$PROJECT_DIR/configs/sensors.yaml"
