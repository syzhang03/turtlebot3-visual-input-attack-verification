#!/usr/bin/env bash
# Record one manually reset scene. No controller or simulator is launched here.
set -euo pipefail
if [[ $# != 2 ]]; then echo 'Usage: bash scripts/run_camera_attack_trial.sh CONFIG_YAML NEW_OUTPUT_DIR' >&2; exit 2; fi
CONFIG="$(realpath "$1")"
OUT="$2"
mkdir "$OUT"  # Refuse to overwrite any existing run, including formal evidence.
OUT="$(realpath "$OUT")"
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
command -v ros2 >/dev/null || { echo 'Source ROS and project install first' >&2; exit 2; }
cp "$CONFIG" "$OUT/parameters.yaml"
git rev-parse HEAD > "$OUT/commit.txt"
git status --porcelain > "$OUT/worktree.txt"
git diff > "$OUT/worktree.patch"
env | sort | grep -E '^(ROS_|RMW_|GZ_)' > "$OUT/environment.txt" || true
ros2 topic info /attack/image_raw --verbose > "$OUT/endpoints-before.txt" 2>&1 || true
ros2 param dump /camera_lidar_monitor > "$OUT/monitor.yaml" 2>&1 || true
ros2 param dump /visual_baseline > "$OUT/baseline.yaml" 2>&1 || true
relay_pid=''; bag_pid=''
cleanup() {
  [[ -z "$relay_pid" ]] || kill -INT "$relay_pid" 2>/dev/null || true
  [[ -z "$bag_pid" ]] || kill -INT "$bag_pid" 2>/dev/null || true
  wait || true
}
trap cleanup EXIT INT TERM
ros2 bag record -o "$OUT/bag" /camera/image_raw /attack/image_raw /scan /odom \
  /cmd_vel /defense/consistency /clock > "$OUT/bag.log" 2>&1 &
bag_pid=$!
ros2 run tb3_camera_attack camera_occlusion --ros-args --params-file "$CONFIG" \
  -p output_topic:=/attack/image_raw -p "log_dir:=$OUT/relay" > "$OUT/relay.log" 2>&1 &
relay_pid=$!
echo 'Recording 30 seconds. Do not enable motion until single-publisher and delivery checks pass.'
sleep 30
kill -0 "$relay_pid" && kill -0 "$bag_pid" || { echo INVALID > "$OUT/status.txt"; exit 1; }
echo 'RECORDED_UNASSESSED: inspect logs, pairing, and bag; this is not a passing safety trial.' > "$OUT/status.txt"
