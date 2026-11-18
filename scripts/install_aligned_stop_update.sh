#!/usr/bin/env bash
# Copy just this update from the VMware share; keep numbered backups and all logs.
set -eo pipefail
SHARED_PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT="${TB3_PROJECT_WS:-$HOME/camera-lidar-attack}"
[[ -d "$PROJECT/src/tb3_lidar_baseline" ]] || { echo "Existing project not found: $PROJECT" >&2; exit 1; }
FILES=(
  src/tb3_lidar_baseline/tb3_lidar_baseline/core.py
  src/tb3_lidar_baseline/tb3_lidar_baseline/node.py
  src/tb3_lidar_baseline/test/test_core.py
  src/tb3_lidar_baseline/test/test_ros.py
  scripts/run_joint_controller.sh
  scripts/run_stop_calibration.sh
  scripts/calibrate_stop_point.py
  scripts/summarize_joint_trial.py
  scripts/install_aligned_stop_update.sh
  tests/test_stop_calibration.py
  docs/aligned-stop-calibration.md
  docs/joint-camera-lidar-attack.md
)
for file in "${FILES[@]}"; do
  [[ -f "$SHARED_PROJECT/$file" ]] || { echo "Missing update source: $file" >&2; exit 1; }
done
if [[ "$(realpath "$PROJECT")" != "$SHARED_PROJECT" ]]; then
  for file in "${FILES[@]}"; do
    mkdir -p "$(dirname -- "$PROJECT/$file")"
    cp -a --backup=numbered -- "$SHARED_PROJECT/$file" "$PROJECT/$file"
  done
fi
cd "$PROJECT"
TB3_PROJECT_WS="$PROJECT" bash scripts/build_joint_attack.sh
source /opt/ros/jazzy/setup.bash
source "${TB3_WS:-$HOME/turtlebot3_ws}/install/setup.bash"
source install/setup.bash
ROS_DOMAIN_ID=34 python3 -m unittest discover -s tests -p test_stop_calibration.py -v
ROS_DOMAIN_ID=34 python3 -m unittest discover -s src/tb3_lidar_baseline/test -v
ROS_DOMAIN_ID=34 python3 -m unittest discover -s src/tb3_camera_lidar_monitor/test -v
echo 'Update built and tests passed. Reset the scene, then run scripts/run_stop_calibration.sh.'
