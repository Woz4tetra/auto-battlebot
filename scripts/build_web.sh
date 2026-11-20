#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
WEB_DIR="$PROJECT_ROOT/web"

if ! command -v node >/dev/null 2>&1; then
    echo "node not found; run install/install_node.sh" >&2
    exit 1
fi

cd "$WEB_DIR"
if [ ! -d node_modules ] || [ package-lock.json -nt node_modules ]; then
    npm ci --no-audit --no-fund
    touch node_modules
fi
npm run build
echo "Dashboard built: $WEB_DIR/dist"
