#!/bin/bash
# Deploy source files (respecting .gitignore) and the data/models/ directory to a Jetson over a local network.
# Only changed files are transferred; files absent on the host are deleted on the remote.
#
# Usage:
#   scripts/deploy_to_jetson.sh [[USER@]HOST] [--skip-service-restart -s] [--sync-only -n]
#
# Configuration (env vars, all optional):
#   JETSON_HOST  - hostname or IP of the Jetson (default: jetson)
#   JETSON_USER  - SSH username (default: detected, see below)
#   JETSON_KEY   - SSH private key (default: ~/.ssh/<host> if it exists, domain stripped,
#                  e.g. ~/.ssh/auto-battlebot-compute-2 for auto-battlebot-compute-2.local)
#   JETSON_PATH  - destination path on the Jetson (default: ~/auto-battlebot)
#
# With a key, ssh offers only that key. Without it, the agent offers every key it holds and
# sshd disconnects after MaxAuthTries (6) before reaching a password prompt.
#
# Without a user, the script tries the one ~/.ssh/config resolves (else the local username),
# then `ben`, then `user` (the ZED Box Mini factory login), keeping the first that logs in
# with the key. Probing needs key auth; with no key it uses the first candidate.
#
# Options:
#   --skip-service-restart  Build/install but do not restart systemd service
#   --sync-only             Sync code and models only: no build, no service restart
#
# Examples:
#   scripts/deploy_to_jetson.sh
#   scripts/deploy_to_jetson.sh 192.168.1.50
#   scripts/deploy_to_jetson.sh user@auto-battlebot-compute-2.local --sync-only
#   scripts/deploy_to_jetson.sh --skip-service-restart
#   scripts/deploy_to_jetson.sh --sync-only
#   JETSON_HOST=ubuntu JETSON_USER=ubuntu scripts/deploy_to_jetson.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

JETSON_HOST="${JETSON_HOST:-jetson}"
JETSON_PATH="${JETSON_PATH:-~/auto-battlebot}"
SKIP_SERVICE_RESTART=0
SYNC_ONLY=0

for arg in "$@"; do
    case "$arg" in
        --skip-service-restart|-s)
            SKIP_SERVICE_RESTART=1
            ;;
        --sync-only|-n)
            SYNC_ONLY=1
            ;;
        --help|-h)
            echo "Usage: scripts/deploy_to_jetson.sh [[USER@]HOST] [--skip-service-restart -s] [--sync-only -n]"
            exit 0
            ;;
        --*)
            echo "Unknown option: $arg"
            echo "Usage: scripts/deploy_to_jetson.sh [[USER@]HOST] [--skip-service-restart -s] [--sync-only -n]"
            exit 1
            ;;
        *@*)
            JETSON_USER="${arg%%@*}"
            JETSON_HOST="${arg#*@}"
            ;;
        *)
            JETSON_HOST="$arg"
            ;;
    esac
done

if [ -z "${JETSON_KEY:-}" ] && [ -f "$HOME/.ssh/${JETSON_HOST%%.*}" ]; then
    JETSON_KEY="$HOME/.ssh/${JETSON_HOST%%.*}"
fi
SSH_OPTS=()
if [ -n "${JETSON_KEY:-}" ]; then
    SSH_OPTS=(-i "$JETSON_KEY" -o IdentitiesOnly=yes)
    echo "Using key ${JETSON_KEY}"
fi

if [ -z "${JETSON_USER:-}" ]; then
    CANDIDATE_USERS=("$(ssh -G "$JETSON_HOST" 2>/dev/null | awk '$1 == "user" { print $2; exit }')" ben user)
    JETSON_USER="${CANDIDATE_USERS[0]}"
    if [ -n "${JETSON_KEY:-}" ]; then
        for candidate in "${CANDIDATE_USERS[@]}"; do
            if ssh "${SSH_OPTS[@]}" -o BatchMode=yes -o ConnectTimeout=5 \
                "${candidate}@${JETSON_HOST}" true 2>/dev/null; then
                JETSON_USER="$candidate"
                break
            fi
        done
    fi
fi
SSH_TARGET="${JETSON_USER}@${JETSON_HOST}"
REMOTE_DEST="${SSH_TARGET}:${JETSON_PATH}"
# rsync takes the remote shell as one string.
RSYNC_SSH="ssh${JETSON_KEY:+ -i '${JETSON_KEY}' -o IdentitiesOnly=yes}"
echo "Deploying as ${SSH_TARGET}"

# ── Version stamp ─────────────────────────────────────────────────────────────
# The Jetson has no git repo (.git is excluded below), so stamp the git version
# into a file that the build reads as a fallback. Same semantics as
# cmake/git_version.cmake: short hash, "-dirty" for tracked modifications.
GIT_VERSION="$(git -C "$PROJECT_ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
if [ -n "$(git -C "$PROJECT_ROOT" status --porcelain --untracked-files=no 2>/dev/null)" ]; then
    GIT_VERSION="${GIT_VERSION}-dirty"
fi
echo "Stamping build version: ${GIT_VERSION}"
echo "$GIT_VERSION" > "$PROJECT_ROOT/.build_version"

# ── Deploy excludes ────────────────────────────────────────────────────────────────
# .deployignore lists paths that stay off the Jetson even though they are
# tracked in git. Syntax matches .gitignore, including "!pattern" to re-include.
# The rules become an rsync filter file applied before the .gitignore merge, so
# a "!pattern" here can also pull back a gitignored path.
DEPLOY_IGNORE="$PROJECT_ROOT/.deployignore"
DEPLOY_FILTER_OPTS=()
if [ -f "$DEPLOY_IGNORE" ]; then
    DEPLOY_FILTER="$(mktemp)"
    trap 'rm -f "$DEPLOY_FILTER"' EXIT
    awk '
        { sub(/\r$/, ""); sub(/[[:space:]]+$/, "") }
        /^[[:space:]]*($|#)/ { next }
        /^!/ { print "+ " substr($0, 2); next }
        { print "- " $0 }
    ' "$DEPLOY_IGNORE" > "$DEPLOY_FILTER"
    echo "Applying $(grep -c . "$DEPLOY_FILTER") rule(s) from .deployignore"
    DEPLOY_FILTER_OPTS=( "--filter=. $DEPLOY_FILTER" )
fi

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
# .deployignore adds excludes on top of that (see above).
echo "Syncing code to ${REMOTE_DEST}..."
# --include for .build_version is ordered before the gitignore merge so the
# stamped version file transfers even though it is gitignored (first match wins).
rsync "${RSYNC_OPTS[@]}" -e "$RSYNC_SSH" \
    --include='/.build_version' \
    "${DEPLOY_FILTER_OPTS[@]}" \
    --filter=':- .gitignore' \
    --exclude='.git/' \
    "$PROJECT_ROOT/" \
    "$REMOTE_DEST/"

# ── Models sync ──────────────────────────────────────────────────────────────
# data/models/ is gitignored and not included in the code pass above.
if [ -d "$PROJECT_ROOT/data/models" ]; then
    echo "Syncing data/models/ to ${REMOTE_DEST}/data/models/..."
    rsync -e "$RSYNC_SSH" --mkpath --archive --verbose --human-readable --progress \
        "$PROJECT_ROOT/data/models/" \
        "${REMOTE_DEST}/data/models/"
else
    echo "No local data/models/ directory found, skipping models sync."
fi

# ── Remote build/install ──────────────────────────────────────────────────────
# Run the Jetson-side build/install script only after sync steps complete.
if [ "$SYNC_ONLY" -eq 1 ]; then
    echo "Sync complete. Skipping remote build and service restart as requested."
    exit 0
fi

echo "Running remote build/install on ${JETSON_HOST}..."
if [ "$SKIP_SERVICE_RESTART" -eq 1 ]; then
    echo "Skipping service restart as requested."
    ssh "${SSH_OPTS[@]}" "$SSH_TARGET" "bash -lc 'cd ${JETSON_PATH} && scripts/build_and_install.sh'"
else
    ssh "${SSH_OPTS[@]}" "$SSH_TARGET" "bash -lc 'cd ${JETSON_PATH} && service/build_and_install_service.sh'"
fi

echo "Deploy and remote build complete."
