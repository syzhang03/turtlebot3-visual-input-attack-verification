#!/usr/bin/env bash
# Run this copy from VM_share after stopping the relay and controller.
set -eo pipefail
SOURCE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET_DIR="${REPLAY_PROJECT_DIR:-$HOME/delay-replay-2026-10-30}"
if [[ ! -f "$TARGET_DIR/src/tb3_visual_baseline/setup.py" ]]; then
  echo "Project not found: $TARGET_DIR" >&2
  exit 1
fi
TARGET_DIR="$(cd -- "$TARGET_DIR" && pwd)"
if [[ "$SOURCE_DIR" == "$TARGET_DIR" ]]; then
  echo 'Run this update script from /mnt/hgfs/VM_share, not the Ubuntu project copy.' >&2
  exit 1
fi
FILES=(
  src/tb3_visual_baseline/tb3_visual_baseline/core.py
  src/tb3_visual_baseline/tb3_visual_baseline/node.py
  src/tb3_visual_baseline/tb3_visual_baseline/replay_node.py
  src/tb3_visual_baseline/test/test_relay_transport.py
  src/tb3_visual_baseline/test/test_core.py
  configs/delay_replay.yaml
  configs/baseline_white_line.yaml
  scripts/run_baseline.sh
  scripts/run_relay.sh
  scripts/run_relay_baseline.sh
  scripts/check_camera_streams.py
  scripts/check_camera_streams.sh
  docs/experiment-baseline.md
  docs/relay-transport.md
  docs/goal-stop.md
)
for relative in "${FILES[@]}"; do
  [[ -f "$SOURCE_DIR/$relative" ]] || { echo "Missing update file: $relative" >&2; exit 1; }
done
BACKUP_DIR="$TARGET_DIR/update-backups/relay-transport-$(date -u +%Y%m%dT%H%M%S)-$$"
mkdir -p "$BACKUP_DIR"
for relative in "${FILES[@]}"; do
  if [[ -f "$TARGET_DIR/$relative" ]]; then
    mkdir -p "$BACKUP_DIR/$(dirname -- "$relative")"
    cp -- "$TARGET_DIR/$relative" "$BACKUP_DIR/$relative"
  fi
  mkdir -p "$TARGET_DIR/$(dirname -- "$relative")"
  cp -- "$SOURCE_DIR/$relative" "$TARGET_DIR/$relative"
done
source /opt/ros/jazzy/setup.bash
source "${TB3_WS:-$HOME/turtlebot3_ws}/install/setup.bash"
cd "$TARGET_DIR"
colcon build --symlink-install --packages-select tb3_visual_baseline
PYTHONPATH="$TARGET_DIR/src/tb3_visual_baseline${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m unittest discover -s src/tb3_visual_baseline/test -v
echo "UPDATE_COMPLETE. Previous source saved in: $BACKUP_DIR"
echo 'Current experiment: speed=1.0, red threshold=0.4, enable_goal_stop=false.'
echo "Start zero-delay relay: bash $TARGET_DIR/scripts/run_relay.sh 0"
echo "Check without driving: bash $TARGET_DIR/scripts/check_camera_streams.sh"
