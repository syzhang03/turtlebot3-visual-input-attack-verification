#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DELAY_S="${1:-0.0}"
source /opt/ros/jazzy/setup.bash
source "${TB3_WS:-$HOME/turtlebot3_ws}/install/setup.bash"
source "$PROJECT_DIR/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-33}"
# ROS parameters are statically typed: CLI 0 is INTEGER, while 0.0 is DOUBLE.
DELAY_S="$(python3 - "$DELAY_S" <<'PY'
import math
import sys

try:
    value = float(sys.argv[1])
except ValueError:
    raise SystemExit('Delay must be a nonnegative number, for example 0 or 0.5.')
if not math.isfinite(value) or value < 0:
    raise SystemExit('Delay must be finite and nonnegative.')
text = repr(value)
if 'e' in text:
    mantissa, exponent = text.split('e')
    if '.' not in mantissa:
        mantissa += '.0'
    text = mantissa + 'e' + exponent
print(text)
PY
)"
exec ros2 run tb3_visual_baseline delay_replay --ros-args \
  --params-file "$PROJECT_DIR/configs/delay_replay.yaml" \
  -p delay_s:="$DELAY_S" -p start_s:=0.0 -p duration_s:=600.0 \
  -p output_reliability:=reliable
