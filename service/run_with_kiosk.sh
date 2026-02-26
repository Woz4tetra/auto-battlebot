#!/bin/bash
# Enable kiosk mode (no screen blanking) and run the already-built auto_battlebot (no build).
# Usage: run_with_kiosk.sh [config_dir]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/enable_kiosk.sh"
exec "$SCRIPT_DIR/run.sh" "$@"
