#!/bin/bash
# Start viz_relay in the background if one is not already serving the default socket.
#
# The relay owns the Foxglove WebSocket server (ws://0.0.0.0:8765) and stays up across app
# restarts, so Foxglove keeps its connection while auto_battlebot restarts. The app never
# waits on it: with no relay it still drives and records, and it reconnects on its own.
#
# Arguments are forwarded to viz_relay (e.g. --port 8766). Set AUTO_BATTLEBOT_BUILD_DIR to
# pick a build tree other than build/.

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
