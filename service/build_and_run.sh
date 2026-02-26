#!/bin/bash
# Build the project and run auto_battlebot.
# Usage: build_and_run.sh [config_dir] [build args...]
#   config_dir defaults to config/main. Extra args (e.g. --test) go to the build.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
CONFIG_PATH="${1:-config/main}"
[ $# -gt 0 ] && shift

source "$PROJECT_ROOT/install/build_cpp_project.sh"
build_cpp_project "$@"
exec "$SCRIPT_DIR/run.sh" "$CONFIG_PATH"
