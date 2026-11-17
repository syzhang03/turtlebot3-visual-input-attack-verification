#!/usr/bin/env bash
# Use the shared updater so core, node, tests and parameters stay in sync.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$SCRIPT_DIR/update_relay_transport.sh"
