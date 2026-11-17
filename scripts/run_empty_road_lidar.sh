#!/usr/bin/env bash
# Drive the LiDAR-only baseline on the empty road.  The endpoint stop is enabled.
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export LIDAR_CONFIG="$PROJECT_DIR/configs/lidar_empty_road.yaml"
exec bash "$PROJECT_DIR/scripts/run_lidar_baseline.sh" "$@"
