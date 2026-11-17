#!/usr/bin/env bash
# Drive the visual baseline on the empty road.  The endpoint stop is enabled.
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export BASELINE_CONFIG="$PROJECT_DIR/configs/baseline_empty_road.yaml"
exec bash "$PROJECT_DIR/scripts/run_baseline.sh" "$@"
