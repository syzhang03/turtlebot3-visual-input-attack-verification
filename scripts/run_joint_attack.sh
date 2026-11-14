#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-clean}"
case "$MODE" in clean|camera|lidar|joint) ;; *)
  echo 'Usage: run_joint_attack.sh [clean|camera|lidar|joint] [camera_delay_s=1.0] [lidar_bias_m=1.0] [sector_half_width_deg=30.0]' >&2
  exit 2 ;;
esac
[[ $# -le 4 ]] || { echo 'Too many arguments' >&2; exit 2; }
source /opt/ros/jazzy/setup.bash
source "${TB3_WS:-$HOME/turtlebot3_ws}/install/setup.bash"
source "${TB3_PROJECT_WS:-$PROJECT_DIR}/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-33}"
exec 9>"/tmp/tb3-joint-relays-${ROS_DOMAIN_ID}.lock"
flock -n 9 || { echo 'Joint relays already running in this ROS domain' >&2; exit 1; }
exec ros2 launch "$PROJECT_DIR/scripts/joint_attack.launch.py" \
  "mode:=$MODE" "camera_delay_s:=${2:-1.0}" "lidar_bias_m:=${3:-1.0}" \
  "sector_half_width_deg:=${4:-30.0}" "results_dir:=${JOINT_RESULTS_DIR:-$HOME/joint-attack-results}"
