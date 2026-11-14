#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-observe}"
PROFILE="${2:-aligned}"
case "$MODE" in observe|drive) ;; *) echo 'Usage: run_joint_controller.sh [observe|drive] [aligned|legacy]' >&2; exit 2 ;; esac
[[ $# -le 2 ]] || { echo 'Too many arguments' >&2; exit 2; }
PROFILE_ARGS=()
case "$PROFILE" in
  aligned)
    PROFILE_PATH="$PROJECT_DIR/configs/aligned_stop.yaml"
    [[ -f "$PROFILE_PATH" ]] || {
      echo 'Missing configs/aligned_stop.yaml: run run_stop_calibration.sh, then calibrate_stop_point.py --latest first.' >&2
      exit 1
    }
    PROFILE_ARGS=(--params-file "$PROFILE_PATH")
    ;;
  legacy) ;;
  *) echo 'Profile must be aligned or legacy' >&2; exit 2 ;;
esac
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-33}"
# Requiring the relay script's lock prevents accidentally using an old result pointer.
exec 9>"/tmp/tb3-joint-relays-${ROS_DOMAIN_ID}.lock"
if flock -n 9; then
  echo 'Start run_joint_attack.sh in another terminal first, then wait for both relay states.' >&2
  exit 1
fi
exec 9>&-
ROOT="${JOINT_RESULTS_DIR:-$HOME/joint-attack-results}"
[[ -f "$ROOT/latest-run.txt" ]] || { echo 'No joint run manifest found' >&2; exit 1; }
RUN_DIR="$(cat "$ROOT/latest-run.txt")"
[[ -f "$RUN_DIR/manifest.json" ]] || { echo 'Missing manifest for current run' >&2; exit 1; }
python3 - "$RUN_DIR/manifest.json" "$ROS_DOMAIN_ID" <<'PY'
import json
import sys
from pathlib import Path
manifest = json.loads(Path(sys.argv[1]).read_text())
if manifest['ros_domain_id'] != sys.argv[2]:
    raise SystemExit('Latest joint run belongs to another ROS domain; use a separate JOINT_RESULTS_DIR.')
PY
exec 8>"$RUN_DIR/controller.lock"
flock -n 8 || { echo 'A controller is already running for this trial' >&2; exit 1; }
echo "Controller results: $RUN_DIR/controller"
echo "Controller profile: $PROFILE"
if [[ "$PROFILE" == aligned ]]; then
  cp -- "$PROFILE_PATH" "$RUN_DIR/controller-profile.yaml"
  PROFILE_ARGS=(--params-file "$RUN_DIR/controller-profile.yaml")
fi
exec bash "$PROJECT_DIR/scripts/run_camera_lidar.sh" "$MODE" \
  "${PROFILE_ARGS[@]}" \
  -p image_topic:=/attack/image_raw -p scan_topic:=/attack/scan \
  -p speed:=1.0 -p visual_stop_threshold:=0.4 -p enable_goal_stop:=false \
  -p image_reliability:=reliable -p record_image_acceptance:=true \
  -p "log_dir:=$RUN_DIR/controller"
