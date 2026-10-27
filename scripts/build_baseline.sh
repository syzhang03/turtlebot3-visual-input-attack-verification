#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${TB3_WS:-$HOME/turtlebot3_ws}/install/setup.bash"
cd "$PROJECT_DIR"
colcon build --symlink-install --packages-select tb3_visual_baseline
echo 'Build complete. Run: bash scripts/run_baseline.sh observe'
