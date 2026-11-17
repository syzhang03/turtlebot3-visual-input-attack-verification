#!/usr/bin/env bash
# Sync the fixed experiment settings; no build or history edits are needed.
set -eo pipefail
SOURCE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET_DIR="${REPLAY_PROJECT_DIR:-$HOME/delay-replay-2026-10-30}"
[[ -f "$TARGET_DIR/src/tb3_visual_baseline/setup.py" ]] || {
  echo "Project not found: $TARGET_DIR" >&2; exit 1;
}
TARGET_DIR="$(cd -- "$TARGET_DIR" && pwd)"
if ! grep -q 'enable_goal_stop' "$TARGET_DIR/src/tb3_visual_baseline/tb3_visual_baseline/node.py" || \
   ! grep -q 'enable_goal_stop' "$TARGET_DIR/src/tb3_visual_baseline/tb3_visual_baseline/core.py"; then
  echo 'Controller code needs updating first. Run the shared scripts/update_goal_stop.sh.' >&2
  exit 1
fi
[[ "$SOURCE_DIR" != "$TARGET_DIR" ]] || {
  echo 'Run this script from /mnt/hgfs/VM_share.' >&2; exit 1;
}
FILES=(
  configs/baseline_white_line.yaml
  scripts/run_baseline.sh
  scripts/run_relay_baseline.sh
  docs/experiment-baseline.md
  docs/relay-transport.md
)
for relative in "${FILES[@]}"; do
  [[ -f "$SOURCE_DIR/$relative" ]] || { echo "Missing file: $relative" >&2; exit 1; }
done
for relative in "${FILES[@]}"; do
  mkdir -p "$TARGET_DIR/$(dirname -- "$relative")"
  cp -- "$SOURCE_DIR/$relative" "$TARGET_DIR/$relative"
done
echo 'SETTINGS_UPDATED: speed=1.0 m/s, red_area_threshold=0.4, enable_goal_stop=false.'
echo 'Restart the controller for the new settings to take effect.'
echo 'Existing trial logs and scene assets are unchanged; no rebuild is needed.'
