#!/usr/bin/env bash
# Record all four sensor streams plus the supporting odometry/TF/clock topics.
# Usage: bash scripts/record_four_sensors.sh [output_dir]
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$HOME/sensor-bags/$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p "$(dirname "$OUT")"
source /opt/ros/jazzy/setup.bash
source "${TB3_WS:-$HOME/turtlebot3_ws}/install/setup.bash"
source "$PROJECT_DIR/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-33}"
echo "Recording four sensors to $OUT"
exec ros2 bag record --use-sim-time -o "$OUT" \
  /camera/image_raw /scan /imu /wheel_speed /joint_states /odom /tf /clock
