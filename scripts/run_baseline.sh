#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-observe}"
case "$MODE" in
  observe) ENABLED=false ;;
  drive) ENABLED=true ;;
  *) echo 'Usage: bash scripts/run_baseline.sh [observe|drive]' >&2; exit 2 ;;
esac
source /opt/ros/jazzy/setup.bash
source "${TB3_WS:-$HOME/turtlebot3_ws}/install/setup.bash"
source "$PROJECT_DIR/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-33}"
cd "$PROJECT_DIR"
exec ros2 run tb3_visual_baseline baseline --ros-args \
  --params-file "$PROJECT_DIR/configs/baseline.yaml" -p enabled:="$ENABLED"
