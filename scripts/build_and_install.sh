#!/bin/bash
# Build the project and install executable + config to a system-wide location.
# Default prefix: /usr/local (executable -> /usr/local/bin/auto_battlebot,
# config -> /usr/local/share/auto_battlebot/config/).
# Run with sudo to install to /usr/local, or set PREFIX to a writable path.
# Override prefix: PREFIX=/opt/auto_battlebot scripts/build_and_install.sh
# This location is used by install/run.sh, install/run_with_kiosk.sh, and service/auto_battlebot.service.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
PREFIX="${PREFIX:-/usr/local}"

source "$PROJECT_ROOT/install/build_cpp_project.sh"
build_cpp_project "$@"

echo "Installing to $PREFIX..."
cd "$PROJECT_ROOT/build"
cmake --install . --prefix "$PREFIX"

echo "Installed: $PREFIX/bin/auto_battlebot"
echo "Config:    $PREFIX/share/auto_battlebot/config/"
echo "Run with:  service/run.sh  (uses installed binary when present)"
echo "Kiosk:     service/run_with_kiosk.sh"
echo "Service:   sudo cp service/auto_battlebot.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable auto_battlebot"
