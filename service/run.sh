#!/bin/bash
# Run auto_battlebot (no build). Prefers system-wide install when present.
# Usage: run.sh [config_dir]
#   config_dir defaults to config/main (dev) or installed config (when installed).

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
source "$SCRIPT_DIR/install_paths.sh"
CONFIG_ARG="${1:-}"

if [ -x "$AUTO_BATTLEBOT_EXE" ]; then
    CONFIG_PATH="${CONFIG_ARG:-$AUTO_BATTLEBOT_CONFIG}"
    echo "Running auto_battlebot (installed) with config: $CONFIG_PATH"
    exec "$AUTO_BATTLEBOT_EXE" -c "$CONFIG_PATH"
fi

EXE="$PROJECT_ROOT/build/auto_battlebot"
CONFIG_PATH="${CONFIG_ARG:-config/main}"
if [ ! -x "$EXE" ]; then
    echo "Error: executable not found at $EXE or $AUTO_BATTLEBOT_EXE (build with scripts/build.sh or install with scripts/build_and_install.sh)"
    exit 1
fi

cd "$PROJECT_ROOT/build"
echo "Running auto_battlebot (build dir) with config: $CONFIG_PATH"
exec ./auto_battlebot -c "$PROJECT_ROOT/$CONFIG_PATH"
