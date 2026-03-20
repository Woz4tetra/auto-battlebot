#!/bin/bash

# Build the C++ application and launch it alongside the Genesis simulation server.
# The sim server starts first (in the background), then the C++ binary connects to it.
# Ctrl-C kills both processes.
#
# Usage:
#   ./scripts/run_simulation.sh                                          # defaults
#   ./scripts/run_simulation.sh path/to/sim_config.toml                  # custom sim config
#   ./scripts/run_simulation.sh path/to/sim_config.toml config/myexp/    # custom sim + C++ config

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
SIM_DIR="$PROJECT_ROOT/simulation"
VENV_DIR="$SIM_DIR/venv"

if [ ! -d "$VENV_DIR" ]; then
    echo "Error: Simulation venv not found at $VENV_DIR"
    echo "Run ./scripts/setup_simulation.sh first."
    exit 1
fi

SIM_CONFIG="${1:-$SIM_DIR/sim_config.toml}"
CPP_CONFIG="${2:-./config/simulation/}"

# --- Build C++ (while the sim server starts up in parallel) -------------------

"$SCRIPT_DIR/build.sh" &
BUILD_PID=$!

# --- Start Genesis sim server -------------------------------------------------

"$VENV_DIR/bin/python" "$SIM_DIR/sim_server.py" "$SIM_CONFIG" &
SIM_PID=$!

# Kill both children on exit (Ctrl-C, errors, normal exit)
cleanup() {
    kill "$SIM_PID" 2>/dev/null
    kill "$BUILD_PID" 2>/dev/null
    wait "$SIM_PID" 2>/dev/null
    wait "$BUILD_PID" 2>/dev/null
}
trap cleanup EXIT

# Wait for build to finish before launching the C++ binary
wait "$BUILD_PID" || { echo "Build failed"; exit 1; }

# --- Run C++ application -----------------------------------------------------

"$SCRIPT_DIR/run.sh" -c "$CPP_CONFIG"
