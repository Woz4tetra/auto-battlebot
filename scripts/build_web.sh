#!/bin/bash
# Build the web dashboard into web/dist/, which viz_relay serves over HTTP.
#
# Runs `npm ci` when package-lock.json is newer than node_modules (or node_modules is missing),
# then `npm run build`. Needs Node 20.19 or newer: install/install_node.sh sets it up.

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
    # npm ci recreates the directory, but touch it so the timestamp check holds even when npm
    # keeps an older mtime.
    touch node_modules
fi
npm run build
echo "Dashboard built: $WEB_DIR/dist"
