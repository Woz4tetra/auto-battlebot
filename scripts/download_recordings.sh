#!/usr/bin/env bash
# Download SVO and MCAP recordings from the Jetson that we do not already have.
#
# A recording counts as "already have it" if a file with the same basename exists
# anywhere under data/svo, data/temp_svo, data/recordings, or data/saved_recordings.
# Everything else is pulled down: .svo2 into data/svo, .mcap into data/recordings.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

JETSON_HOST="${JETSON_HOST:-jetson}"
JETSON_USER="${JETSON_USER:-ben}"
JETSON_PATH="${JETSON_PATH:-auto-battlebot}"

# Directories searched on both ends, relative to the project root.
SEARCH_DIRS=(data/svo data/temp_svo data/recordings data/saved_recordings)

SVO_DEST="$PROJECT_ROOT/data/svo"
MCAP_DEST="$PROJECT_ROOT/data/recordings"

# Skip remote files touched in the last N minutes so an in-progress recording is
# not pulled half-written and then treated as already downloaded next run.
MIN_AGE_MIN="${MIN_AGE_MIN:-2}"

# Only consider remote recordings modified within this many hours. 0 means no limit.
HOURS="${HOURS:-24}"

DRY_RUN=0
LIST_ONLY=0

usage() {
    cat <<'EOF'
Usage: scripts/download_recordings.sh [HOST] [options]

Options:
  -H, --hours N   Only consider recordings modified in the last N hours
                  (default: 24; use 0 for no time limit)
  -n, --dry-run   Show what rsync would transfer, transfer nothing
  -l, --list      List the new remote recordings and exit
  -h, --help      Show this help

Environment:
  JETSON_HOST     Remote host (default: jetson)
  JETSON_USER     Remote user (default: ben)
  JETSON_PATH     Remote project root (default: auto-battlebot)
  HOURS           Same as --hours (default: 24)
  MIN_AGE_MIN     Ignore remote files newer than this many minutes (default: 2)
EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        -H|--hours)
            [ "$#" -ge 2 ] || { echo "$1 requires a value" >&2; exit 1; }
            HOURS="$2"; shift ;;
        --hours=*) HOURS="${1#*=}" ;;
        -n|--dry-run) DRY_RUN=1 ;;
        -l|--list) LIST_ONLY=1 ;;
        -h|--help) usage; exit 0 ;;
        --*|-*) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
        *) JETSON_HOST="$1" ;;
    esac
    shift
done

case "$HOURS" in
    ''|*[!0-9]*) echo "--hours must be a non-negative integer, got: $HOURS" >&2; exit 1 ;;
esac
MAX_AGE_MIN=$((HOURS * 60))

REMOTE="${JETSON_USER}@${JETSON_HOST}"

echo "Indexing local recordings under ${PROJECT_ROOT}/data ..."
LOCAL_INDEX="$(mktemp)"
REMOTE_LIST="$(mktemp)"
SVO_LIST="$(mktemp)"
MCAP_LIST="$(mktemp)"
trap 'rm -f "$LOCAL_INDEX" "$REMOTE_LIST" "$SVO_LIST" "$MCAP_LIST"' EXIT

local_dirs=()
for dir in "${SEARCH_DIRS[@]}"; do
    [ -d "$PROJECT_ROOT/$dir" ] && local_dirs+=("$PROJECT_ROOT/$dir")
done

if [ "${#local_dirs[@]}" -gt 0 ]; then
    find "${local_dirs[@]}" -type f \( -name '*.svo2' -o -name '*.mcap' \) \
        -not -path '*/.rsync-partial/*' -printf '%f\n' | sort -u > "$LOCAL_INDEX"
fi
echo "  $(wc -l < "$LOCAL_INDEX") local recording(s) already present"

if [ "$MAX_AGE_MIN" -gt 0 ]; then
    echo "Listing recordings on ${REMOTE}:${JETSON_PATH} from the last ${HOURS}h ..."
else
    echo "Listing recordings on ${REMOTE}:${JETSON_PATH} (no time limit) ..."
fi
# Runs remotely: skip search directories that do not exist there, so a missing
# one does not make find exit non-zero and abort this script under pipefail.
ssh "$REMOTE" "sh -s '${JETSON_PATH}' '${MIN_AGE_MIN}' '${MAX_AGE_MIN}' ${SEARCH_DIRS[*]}" <<'REMOTE_EOF' | sort > "$REMOTE_LIST"
root=$1; shift
min_age=$1; shift
max_age=$1; shift
cd "$root" || { echo "ERROR: remote path '$root' not found" >&2; exit 3; }
dirs=
for d in "$@"; do
    [ -d "$d" ] && dirs="$dirs $d"
done
[ -n "$dirs" ] || exit 0
window=
[ "$max_age" -gt 0 ] && window="-mmin -$max_age"
find $dirs -type f \( -name '*.svo2' -o -name '*.mcap' \) \
    -mmin +"$min_age" $window 2>/dev/null || true
REMOTE_EOF
echo "  $(wc -l < "$REMOTE_LIST") remote recording(s) found"

# Select remote paths whose basename is not present locally, one per basename.
awk -v svo_out="$SVO_LIST" -v mcap_out="$MCAP_LIST" '
    NR == FNR { have[$0] = 1; next }
    { name = $0; sub(/.*\//, "", name) }
    name in have || name in seen { next }
    {
        seen[name] = 1
        if (name ~ /\.svo2$/) print > svo_out
        else if (name ~ /\.mcap$/) print > mcap_out
    }
' "$LOCAL_INDEX" "$REMOTE_LIST"

svo_count="$(wc -l < "$SVO_LIST")"
mcap_count="$(wc -l < "$MCAP_LIST")"
echo "New: ${svo_count} SVO(s) -> data/svo, ${mcap_count} MCAP(s) -> data/recordings"

if [ "$((svo_count + mcap_count))" -eq 0 ]; then
    echo "Nothing to download."
    exit 0
fi

if [ "$LIST_ONLY" -eq 1 ]; then
    cat "$SVO_LIST" "$MCAP_LIST"
    exit 0
fi

RSYNC_OPTS=(
    --archive
    --human-readable
    --partial-dir=.rsync-partial
    --info=progress2
    --ignore-existing
    --no-relative
    --files-from
)
[ "$DRY_RUN" -eq 1 ] && RSYNC_OPTS=(--dry-run --itemize-changes "${RSYNC_OPTS[@]}")

pull() {
    local list="$1" dest="$2"
    [ -s "$list" ] || return 0
    mkdir -p "$dest"
    echo "Downloading $(wc -l < "$list") file(s) to ${dest} ..."
    rsync "${RSYNC_OPTS[@]}" "$list" "${REMOTE}:${JETSON_PATH}/" "$dest/"
}

pull "$SVO_LIST" "$SVO_DEST"
pull "$MCAP_LIST" "$MCAP_DEST"

echo "Done."
