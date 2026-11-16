#!/bin/bash
set -e

usage() {
    cat <<'EOF'
Usage: scripts/build_and_install.sh [build args]

Build the project and install the executable and config to a user-accessible location.
Default prefix: $HOME/.local (executable at bin/auto_battlebot, config at
share/auto_battlebot/config/). No sudo is required.
Override with PREFIX=/usr/local scripts/build_and_install.sh (needs sudo for a system install).
The systemd service (service/auto-battlebot.service) uses /usr/local by default.
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
PREFIX="${PREFIX:-$HOME/.local}"

source "$PROJECT_ROOT/install/build_cpp_project.sh"
build_cpp_project "$@"

if command -v node >/dev/null 2>&1; then
    "$SCRIPT_DIR/build_web.sh"
else
    echo "node not found; the dashboard will not be installed (install/install_node.sh)"
fi

echo "Installing to $PREFIX..."
cd "$PROJECT_ROOT/build"
cmake --install . --prefix "$PREFIX"

echo "Run: $PREFIX/bin/auto_battlebot -c $PREFIX/share/auto_battlebot/config/main.toml"
