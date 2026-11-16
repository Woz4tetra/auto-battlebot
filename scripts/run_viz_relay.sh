#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="${PROJECT_ROOT}/${AUTO_BATTLEBOT_BUILD_DIR:-build}"
RELAY_BIN="${BUILD_DIR}/viz_relay"
LOG_FILE="${TMPDIR:-/tmp}/auto_battlebot_viz_relay.log"

if [ ! -x "$RELAY_BIN" ]; then
    echo "viz_relay not built at $RELAY_BIN; run scripts/build.sh first" >&2
    exit 1
fi

if pgrep -x viz_relay > /dev/null 2>&1; then
    echo "viz_relay already running"
    exit 0
fi

nohup "$RELAY_BIN" "$@" > "$LOG_FILE" 2>&1 &
echo "viz_relay started (pid $!), log: $LOG_FILE"
