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

echo "Installed: $PREFIX/bin/auto_battlebot"
echo "Library:   $PREFIX/lib/libauto_battlebot.so.1  (required at run time)"
echo "Config:    $PREFIX/share/auto_battlebot/config/"
echo "Run:       $PREFIX/bin/auto_battlebot -c $PREFIX/share/auto_battlebot/config/main"
if [ ! -f "$PREFIX/lib/libauto_battlebot.so.1" ]; then
    echo "WARNING: $PREFIX/lib/libauto_battlebot.so.1 not found; the executable may fail to start."
fi
echo "Service:   Edit service/auto_battlebot.service ExecStart to use $PREFIX/bin/auto_battlebot and $PREFIX/share/auto_battlebot/config/main, then sudo cp service/auto_battlebot.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable auto_battlebot"
