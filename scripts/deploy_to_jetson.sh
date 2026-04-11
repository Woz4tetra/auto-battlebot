#!/bin/bash
# Deploy source files (respecting .gitignore) and the models/ directory to a Jetson over a local network.
# Only changed files are transferred; files absent on the host are deleted on the remote.
#
# Usage:
#   scripts/deploy_to_jetson.sh [HOST]
#
# Configuration (env vars, all optional):
#   JETSON_HOST  - hostname or IP of the Jetson (default: ubuntu)
#   JETSON_USER  - SSH username             (default: ben)
#   JETSON_PATH  - destination path on the Jetson (default: ~/auto-battlebot)
#
# Examples:
#   scripts/deploy_to_jetson.sh
#   scripts/deploy_to_jetson.sh 192.168.1.50
#   JETSON_HOST=ubuntu JETSON_USER=ubuntu scripts/deploy_to_jetson.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

JETSON_HOST="${1:-${JETSON_HOST:-jetson}}"
JETSON_USER="${JETSON_USER:-ben}"
JETSON_PATH="${JETSON_PATH:-~/auto-battlebot}"

REMOTE_DEST="${JETSON_USER}@${JETSON_HOST}:${JETSON_PATH}"

RSYNC_OPTS=(
    --archive
    --verbose
    --human-readable
    --progress
    --delete
)

# ── Code sync ────────────────────────────────────────────────────────────────
# --filter reads each .gitignore encountered during traversal, excluding all
# gitignored paths. /models is excluded by the root .gitignore, so --delete
# will not touch it on the remote. .git/ is excluded explicitly.
echo "Syncing code to ${REMOTE_DEST}..."
rsync "${RSYNC_OPTS[@]}" \
    --filter=':- .gitignore' \
    --exclude='.git/' \
    "$PROJECT_ROOT/" \
    "$REMOTE_DEST/"

# ── Models sync ──────────────────────────────────────────────────────────────
# models/ is gitignored and not included in the code pass above.
if [ -d "$PROJECT_ROOT/models" ]; then
    echo "Syncing models/ to ${REMOTE_DEST}/models/..."
    rsync --archive --verbose --human-readable --progress \
        "$PROJECT_ROOT/models/" \
        "${JETSON_USER}@${JETSON_HOST}:${JETSON_PATH}/models/"
else
    echo "No local models/ directory found, skipping models sync."
fi

# ── Remote build/install ──────────────────────────────────────────────────────
# Run the Jetson-side build/install script only after sync steps complete.
echo "Running remote build/install on ${JETSON_HOST}..."
ssh "${JETSON_USER}@${JETSON_HOST}" "bash -lc 'cd ${JETSON_PATH} && service/build_and_install_service.sh'"

echo "Deploy and remote build complete."
