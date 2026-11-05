#!/usr/bin/env bash
# Start the fixed experiment scene; this does not start a controller or relay.
set -eo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP=/opt/ros/jazzy/setup.bash
TB3_SETUP="${TB3_WS:-$HOME/turtlebot3_ws}/install/setup.bash"

for required in "$ROS_SETUP" "$TB3_SETUP" \
  "$PROJECT_DIR/scripts/start_scene.launch.py" \
  "$PROJECT_DIR/worlds/intersection-white-line.sdf"; do
  if [[ ! -f "$required" ]]; then
    printf 'Missing required file: %s\n' "$required" >&2
    exit 1
  fi
done

source "$ROS_SETUP"
source "$TB3_SETUP"
export TURTLEBOT3_MODEL=waffle_pi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-33}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

# Reject a second copy of this helper. Do not kill unrelated simulations.
exec 9>"${TMPDIR:-/tmp}/tb3-experiment-scene-${UID}-${ROS_DOMAIN_ID}.lock"
if ! flock -n 9; then
  echo 'Scene helper already running. Stop its terminal with Ctrl+C first.' >&2
  exit 1
fi

echo 'Starting houses, road, white line and red box at X=4.0 m.'
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID. Stop any previously launched Gazebo first."
echo 'Wait for SCENE_READY. Keep this terminal open; Ctrl+C stops the scene.'
cd "$PROJECT_DIR"
exec ros2 launch "$PROJECT_DIR/scripts/start_scene.launch.py"
