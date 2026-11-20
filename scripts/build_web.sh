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
lock_hash=$(sha256sum package-lock.json | cut -d' ' -f1)
if [ ! -d node_modules ] || [ "$(cat node_modules/.lock-hash 2>/dev/null)" != "$lock_hash" ]; then
    npm ci --no-audit --no-fund
    echo "$lock_hash" >node_modules/.lock-hash
fi
npm run build
echo "Dashboard built: $WEB_DIR/dist"
