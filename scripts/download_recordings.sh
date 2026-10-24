#!/usr/bin/env bash
# Download SVO and MCAP recordings from the Jetson that we do not already have.
#
# A recording counts as "already have it" if a file with the same basename exists
# anywhere under data/svo, data/temp_svo, data/recordings, or data/saved_recordings.
# Everything else is pulled down: .svo2 into data/svo, .mcap into data/recordings.
#
# SSH key and user are resolved the same way as scripts/deploy_to_jetson.sh: the key
# defaults to ~/.ssh/<host> (domain stripped) if it exists, and without a user the
# script probes the ~/.ssh/config user (else the local username), then `ben`, then
# `user` (the ZED Box Mini factory login), keeping the first that logs in with the key.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

JETSON_HOST="${JETSON_HOST:-jetson}"
JETSON_PATH="${JETSON_PATH:-auto-battlebot}"

# Directories searched on both ends, relative to the project root.
SEARCH_DIRS=(data/svo data/temp_svo data/recordings data/saved_recordings)

SVO_DEST="$PROJECT_ROOT/data/svo"
MCAP_DEST="$PROJECT_ROOT/data/recordings"

# Only consider remote recordings modified within this many hours. 0 means no limit.
HOURS="${HOURS:-24}"

DRY_RUN=0
LIST_ONLY=0
AUTONOMY_ONLY=0

usage() {
    cat <<'EOF'
Usage: scripts/download_recordings.sh [[USER@]HOST] [options]

Options:
  -H, --hours N   Only consider recordings modified in the last N hours
                  (default: 24; use 0 for no time limit)
  -n, --dry-run   Show what rsync would transfer, transfer nothing
  -l, --list      List the new remote recordings and exit
      --autonomy-only
                  Skip MCAPs whose /status/system never reports the autonomy
                  switch on (checked on the remote; needs its venv). SVOs and
                  MCAPs from before autonomy_switch_on existed have no such
                  record: SVOs pass through, those MCAPs are skipped
  -h, --help      Show this help

Environment:
  JETSON_HOST     Remote host (default: jetson)
  JETSON_USER     Remote user (default: detected: ssh config user, ben, user)
  JETSON_KEY      SSH private key (default: ~/.ssh/<host> if it exists, domain stripped)
  JETSON_PATH     Remote project root (default: auto-battlebot)
  HOURS           Same as --hours (default: 24)
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
        --autonomy-only) AUTONOMY_ONLY=1 ;;
        -h|--help) usage; exit 0 ;;
        --*|-*) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
        *@*) JETSON_USER="${1%%@*}"; JETSON_HOST="${1#*@}" ;;
        *) JETSON_HOST="$1" ;;
    esac
    shift
done

case "$HOURS" in
    ''|*[!0-9]*) echo "--hours must be a non-negative integer, got: $HOURS" >&2; exit 1 ;;
esac
MAX_AGE_MIN=$((HOURS * 60))

if [ -z "${JETSON_KEY:-}" ] && [ -f "$HOME/.ssh/${JETSON_HOST%%.*}" ]; then
    JETSON_KEY="$HOME/.ssh/${JETSON_HOST%%.*}"
fi
# With a key, ssh offers only that key. Without it, the agent offers every key it holds
# and sshd disconnects after MaxAuthTries (6) before reaching a password prompt.
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
REMOTE="${JETSON_USER}@${JETSON_HOST}"
# rsync takes the remote shell as one string.
RSYNC_SSH="ssh${JETSON_KEY:+ -i '${JETSON_KEY}' -o IdentitiesOnly=yes}"
echo "Downloading as ${REMOTE}"

echo "Indexing local recordings under ${PROJECT_ROOT}/data ..."
LOCAL_INDEX="$(mktemp)"
RAW_LIST="$(mktemp)"
REMOTE_LIST="$(mktemp)"
SKIP_LIST="$(mktemp)"
SVO_LIST="$(mktemp)"
MCAP_LIST="$(mktemp)"
trap 'rm -f "$LOCAL_INDEX" "$RAW_LIST" "$REMOTE_LIST" "$SKIP_LIST" "$SVO_LIST" "$MCAP_LIST"' EXIT

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
ssh "${SSH_OPTS[@]}" "$REMOTE" "sh -s '${JETSON_PATH}' '${MAX_AGE_MIN}' ${SEARCH_DIRS[*]}" <<'REMOTE_EOF' | sort > "$RAW_LIST"
root=$1; shift
max_age=$1; shift
cd "$root" || { echo "ERROR: remote path '$root' not found" >&2; exit 3; }
command -v lsof >/dev/null || { echo "ERROR: lsof not installed on remote" >&2; exit 4; }
dirs=
for d in "$@"; do
    [ -d "$d" ] && dirs="$dirs $d"
done
[ -n "$dirs" ] || exit 0
window=
[ "$max_age" -gt 0 ] && window="-mmin -$max_age"

# Absolute paths of every file any process currently holds open.
open_paths=$(mktemp)
trap 'rm -f "$open_paths"' EXIT
lsof -F n 2>/dev/null | sed -n 's/^n//p' | sort -u > "$open_paths"

find $dirs -type f \( -name '*.svo2' -o -name '*.mcap' \) $window 2>/dev/null |
while IFS= read -r f; do
    if grep -Fxq "$PWD/$f" "$open_paths"; then
        echo "O $f"
    else
        echo "R $f"
    fi
done
REMOTE_EOF
sed -n 's/^R //p' "$RAW_LIST" > "$REMOTE_LIST"
sed -n 's/^O //p' "$RAW_LIST" > "$SKIP_LIST"
echo "  $(wc -l < "$REMOTE_LIST") remote recording(s) found"
if [ -s "$SKIP_LIST" ]; then
    echo "  ignoring $(wc -l < "$SKIP_LIST") file(s) still open by a running process:"
    sed 's|^|    |' "$SKIP_LIST"
fi

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

if [ "$AUTONOMY_ONLY" -eq 1 ] && [ -s "$MCAP_LIST" ]; then
    echo "Checking $(wc -l < "$MCAP_LIST") MCAP(s) for autonomous activity ..."
    AUTONOMY_LIST="$(mktemp)"
    trap 'rm -f "$LOCAL_INDEX" "$RAW_LIST" "$REMOTE_LIST" "$SKIP_LIST" "$SVO_LIST" "$MCAP_LIST" "$AUTONOMY_LIST"' EXIT
    # Prints "A <seconds> <path>" for MCAPs with autonomy on, "N 0.0 <path>" without.
    # /status/system is 10 Hz, so seconds = messages / 10.
    ssh "${SSH_OPTS[@]}" "$REMOTE" \
        "cd '${JETSON_PATH}' && venv/bin/python -I -c '$(cat <<'PY_EOF'
import json
import sys

from mcap.reader import make_reader
from mcap.records import Channel, Message
from mcap.stream_reader import StreamReader


def switch_on(data):
    return bool(json.loads(data).get("autonomy_switch_on"))


def count_indexed(path):
    with open(path, "rb") as file:
        reader = make_reader(file)
        return sum(switch_on(m.data) for _, _, m in reader.iter_messages(topics=["/status/system"]))


def count_streamed(path):
    """Fallback for a file with no footer (writer crashed): count up to the break."""
    on = 0
    status_ids = set()
    try:
        with open(path, "rb") as file:
            for record in StreamReader(file).records:
                if isinstance(record, Channel) and record.topic == "/status/system":
                    status_ids.add(record.id)
                elif isinstance(record, Message) and record.channel_id in status_ids:
                    on += switch_on(record.data)
    except Exception:
        pass
    return on


for line in sys.stdin:
    path = line.rstrip("\n")
    try:
        on = count_indexed(path)
    except Exception:
        on = count_streamed(path)
    tag = "A" if on else "N"
    print(f"{tag} {on / 10:.1f} {path}")
PY_EOF
)'" < "$MCAP_LIST" > "$AUTONOMY_LIST"
    awk '$1 == "A" { printf "  keep  %6.1fs autonomous  %s\n", $2, $3 }
         $1 == "N" { printf "  skip  no autonomy         %s\n", $3 }' "$AUTONOMY_LIST"
    awk '$1 == "A" { print $3 }' "$AUTONOMY_LIST" > "$MCAP_LIST"
fi

svo_count="$(wc -l < "$SVO_LIST")"
mcap_count="$(wc -l < "$MCAP_LIST")"
echo "New: ${svo_count} SVO(s) -> data/svo, ${mcap_count} MCAP(s) -> data/recordings"

if [ "$((svo_count + mcap_count))" -eq 0 ]; then
    echo "Nothing to download."
    exit 0
fi

show_new() {
    local list="$1" dest="$2" path
    [ -s "$list" ] || return 0
    while IFS= read -r path; do
        printf '  %s\n    -> %s/%s\n' "$path" "$dest" "${path##*/}"
    done < "$list"
}

show_new "$SVO_LIST" "$SVO_DEST"
show_new "$MCAP_LIST" "$MCAP_DEST"

if [ "$LIST_ONLY" -eq 1 ]; then
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
    rsync -e "$RSYNC_SSH" "${RSYNC_OPTS[@]}" "$list" "${REMOTE}:${JETSON_PATH}/" "$dest/"
}

pull "$SVO_LIST" "$SVO_DEST"
pull "$MCAP_LIST" "$MCAP_DEST"

echo "Done."
