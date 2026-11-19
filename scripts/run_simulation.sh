#!/bin/bash

set -e

usage() {
    cat <<'EOF'
Usage: ./scripts/run_simulation.sh [SIM_CONFIG [CPP_CONFIG]]

Build the C++ application and launch it alongside the simulation server.
The sim server starts first (in the background), then the C++ binary connects to it.
Ctrl-C kills both processes.

  ./scripts/run_simulation.sh                                           defaults
  ./scripts/run_simulation.sh path/to/sim_config.toml                   custom sim config
  ./scripts/run_simulation.sh path/to/sim_config.toml config/myexp.toml custom sim and C++ config
EOF
}

case "${1:-}" in
    -h | --help)
        usage
        exit 0
        ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
SIM_DIR="$PROJECT_ROOT/simulation"
VENV_DIR="$PROJECT_ROOT/venv"

if [ ! -d "$VENV_DIR" ]; then
    echo "Error: project venv not found at $VENV_DIR"
    echo "Run ./scripts/setup_python.sh first."
    exit 1
fi

SIM_CONFIG="${1:-$SIM_DIR/sim_mrs_buff_mk3.toml}"

ROBOT="$(basename "$SIM_CONFIG" .toml)"
ROBOT="${ROBOT#sim_}"
DEFAULT_CPP_CONFIG="./config/simulation/${ROBOT}_kinematic_sim.toml"
if [ ! -f "$PROJECT_ROOT/${DEFAULT_CPP_CONFIG#./}" ]; then
    DEFAULT_CPP_CONFIG="./config/simulation/kinematic_sim.toml"
fi
CPP_CONFIG="${2:-$DEFAULT_CPP_CONFIG}"
echo "sim config: $SIM_CONFIG"
echo "cpp config: $CPP_CONFIG"

"$SCRIPT_DIR/build.sh" &
BUILD_PID=$!

"$VENV_DIR/bin/python" "$SIM_DIR/kinematic_sim_server.py" "$SIM_CONFIG" &
SIM_PID=$!

cleanup() {
    kill "$SIM_PID" 2>/dev/null
    kill "$BUILD_PID" 2>/dev/null
    wait "$SIM_PID" 2>/dev/null
    wait "$BUILD_PID" 2>/dev/null
}
trap cleanup EXIT

wait "$BUILD_PID" || { echo "Build failed"; exit 1; }

"$SCRIPT_DIR/run.sh" -c "$CPP_CONFIG"
