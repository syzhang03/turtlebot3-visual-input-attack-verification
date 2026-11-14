#!/usr/bin/env bash
# Install the camera input QoS update only; retain numbered backups and all runs.
set -eo pipefail
SHARED_PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT="${TB3_PROJECT_WS:-$HOME/camera-lidar-attack}"
[[ -d "$PROJECT/src/tb3_visual_baseline" ]] || { echo "Existing project not found: $PROJECT" >&2; exit 1; }
FILES=(
  src/tb3_visual_baseline/tb3_visual_baseline/replay_node.py
  src/tb3_visual_baseline/test/test_relay_ros.py
  scripts/joint_attack.launch.py
  scripts/check_camera_streams.py
  scripts/summarize_joint_trial.py
  scripts/install_camera_transport_update.sh
  docs/scene-display-diagnostic.md
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
python3 -c 'import rclpy; from tb3_visual_baseline.replay_node import DelayReplay; from rclpy.executors import SingleThreadedExecutor; from rclpy.qos import HistoryPolicy'
ROS_DOMAIN_ID=34 python3 -m unittest discover -s src/tb3_visual_baseline/test -p test_replay.py -v
ROS_DOMAIN_ID=34 python3 -m unittest discover -s src/tb3_visual_baseline/test -p test_relay_transport.py -v
ROS_DOMAIN_ID=34 python3 -m unittest discover -s src/tb3_visual_baseline/test -p test_relay_ros.py -v
echo 'Camera transport update built and tests passed.'
echo 'Restart joint relays; expect Input reliability=reliable; queue depth=1.'
echo 'Then run check_camera_streams.sh --seconds 60 --gazebo --raw-reliability reliable before driving.'
