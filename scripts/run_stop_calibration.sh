#!/usr/bin/env bash
# A separate, clean, slow approach to measure the visual 0.4 crossing.
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
[[ $# -eq 0 ]] || { echo 'Usage: run_stop_calibration.sh' >&2; exit 2; }
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-33}"
exec 9>"/tmp/tb3-joint-relays-${ROS_DOMAIN_ID}.lock"
flock -n 9 || { echo 'Stop run_joint_attack.sh before raw-sensor calibration.' >&2; exit 1; }
ROOT="${STOP_CALIBRATION_RESULTS:-$HOME/stop-point-calibration}"
RUN_DIR="$ROOT/$(date -u +%Y%m%dT%H%M%S)-$$"
mkdir -p "$ROOT"
mkdir "$RUN_DIR"
printf '%s\n' "$RUN_DIR" > "$ROOT/latest-run.txt"
echo "Calibration results: $RUN_DIR"
echo 'Use a RESET scene. Only this controller may run. Raw camera + raw scan; speed=0.3m/s.'
echo 'After VISUAL_STOP, wait 3 seconds, then Ctrl+C. Next run: python3 scripts/calibrate_stop_point.py --latest'
exec bash "$PROJECT_DIR/scripts/run_camera_lidar.sh" drive \
  -p image_topic:=/camera/image_raw -p scan_topic:=/scan \
  -p image_reliability:=best_effort -p speed:=0.3 \
  -p stop_distance_mode:=dynamic -p visual_stop_threshold:=0.4 \
  -p visual_verify_threshold:=0.4 -p calibration_mode:=true \
  -p pair_tolerance:=0.06 -p record_image_acceptance:=true \
  -p enable_goal_stop:=false -p "log_dir:=$RUN_DIR/controller"
