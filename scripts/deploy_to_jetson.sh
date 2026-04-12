#!/bin/bash
# Deploy source files (respecting .gitignore) and the data/models/ directory to a Jetson over a local network.
# Only changed files are transferred; files absent on the host are deleted on the remote.
#
# Usage:
#   scripts/deploy_to_jetson.sh [HOST] [--skip-service-restart -s]
#
# Configuration (env vars, all optional):
#   JETSON_HOST  - hostname or IP of the Jetson (default: ubuntu)
#   JETSON_USER  - SSH username             (default: ben)
#   JETSON_PATH  - destination path on the Jetson (default: ~/auto-battlebot)
#
# Options:
#   --skip-service-restart  Build/install but do not restart systemd service
#
# Examples:
#   scripts/deploy_to_jetson.sh
#   scripts/deploy_to_jetson.sh 192.168.1.50
#   scripts/deploy_to_jetson.sh --skip-service-restart
#   JETSON_HOST=ubuntu JETSON_USER=ubuntu scripts/deploy_to_jetson.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

JETSON_HOST="${JETSON_HOST:-jetson}"
JETSON_USER="${JETSON_USER:-ben}"
JETSON_PATH="${JETSON_PATH:-~/auto-battlebot}"
SKIP_SERVICE_RESTART=0

for arg in "$@"; do
    case "$arg" in
        --skip-service-restart|-s)
            SKIP_SERVICE_RESTART=1
            ;;
        --help|-h)
            echo "Usage: scripts/deploy_to_jetson.sh [HOST] [--skip-service-restart -s]"
            exit 0
            ;;
        --*)
            echo "Unknown option: $arg"
            echo "Usage: scripts/deploy_to_jetson.sh [HOST] [--skip-service-restart -s]"
            exit 1
            ;;
        *)
            JETSON_HOST="$arg"
            ;;
    esac
done

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
# gitignored paths. /data is excluded by the root .gitignore, so --delete
# will not touch it on the remote. .git/ is excluded explicitly.
echo "Syncing code to ${REMOTE_DEST}..."
rsync "${RSYNC_OPTS[@]}" \
    --filter=':- .gitignore' \
    --exclude='.git/' \
    "$PROJECT_ROOT/" \
    "$REMOTE_DEST/"

# ── Models sync ──────────────────────────────────────────────────────────────
# data/models/ is gitignored and not included in the code pass above.
if [ -d "$PROJECT_ROOT/data/models" ]; then
    echo "Syncing data/models/ to ${REMOTE_DEST}/data/models/..."
    rsync --archive --verbose --human-readable --progress \
        "$PROJECT_ROOT/data/models/" \
        "${JETSON_USER}@${JETSON_HOST}:${JETSON_PATH}/data/models/"
else
    echo "No local data/models/ directory found, skipping models sync."
fi

# ── Remote build/install ──────────────────────────────────────────────────────
# Run the Jetson-side build/install script only after sync steps complete.
echo "Running remote build/install on ${JETSON_HOST}..."
if [ "$SKIP_SERVICE_RESTART" -eq 1 ]; then
    echo "Skipping service restart as requested."
    ssh "${JETSON_USER}@${JETSON_HOST}" "bash -lc 'cd ${JETSON_PATH} && scripts/build_and_install.sh'"
else
    ssh "${JETSON_USER}@${JETSON_HOST}" "bash -lc 'cd ${JETSON_PATH} && service/build_and_install_service.sh'"
fi

echo "Deploy and remote build complete."
