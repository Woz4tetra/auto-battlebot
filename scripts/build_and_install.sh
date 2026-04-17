#!/bin/bash
# Build the project and install executable + config to a user-accessible location.
# Default prefix: <project>/install (executable -> install/bin/auto_battlebot,
# config -> install/share/auto_battlebot/config/). No sudo required.
# Override prefix: PREFIX=/usr/local scripts/build_and_install.sh  (requires sudo for system install)
# The systemd service (service/auto_battlebot.service) uses /usr/local by default;
# point ExecStart at install/bin/auto_battlebot if using the default project install.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
PREFIX="${PREFIX:-$HOME/.local}"

source "$PROJECT_ROOT/install/build_cpp_project.sh"
build_cpp_project "$@"

echo "Installing to $PREFIX..."
cd "$PROJECT_ROOT/build"
cmake --install . --prefix "$PREFIX"

echo "Run: $PREFIX/bin/auto_battlebot -c $PREFIX/share/auto_battlebot/config/main.toml"
