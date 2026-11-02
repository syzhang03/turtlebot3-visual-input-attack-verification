#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-33}"
exec python3 "$SCRIPT_DIR/check_camera_streams.py" "$@"
