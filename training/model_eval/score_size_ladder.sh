#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 2 ]; then
    echo "usage: $0 <date> <size> [size ...]" >&2
    exit 2
fi
DATE=$1
shift
SIZES=("$@")
ARM=${ARM:-d50000}
BASELINE=${BASELINE:-s}
SHAPE=${SHAPE:-rect384x640}
SM=${SM:-sm86}
CONF=${CONF:-0.5}
RUNS=${RUNS:-all}

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$REPO"
SET=training/data/cage_high_x50_conf044
EVAL=${EVAL:-$SET/d40000}
VENUES=${VENUES:-${SET}_by_venue}
suffix=""
[ "$CONF" = 0.5 ] || suffix="_conf${CONF}"
OUT=${OUT:-$SET/scores_size_ladder${suffix}}
LABELS="mr_stabs_mk2,mrs_buff_mk3,opponent,house_bot"

shape_part=""
[ "$SHAPE" = square ] || shape_part="_${SHAPE}"
candidates=()
for size in "${SIZES[@]}"; do
    engine="data/models/yolo26${size}-pose_${ARM}_${DATE}_last${shape_part}_x86_64_${SM}.engine"
    [ -f "$engine" ] || { echo "missing engine $engine" >&2; exit 1; }
    candidates+=(--candidate "$size=$engine")
done
for pair in ${EXTRA:-}; do
    [ -f "${pair#*=}" ] || { echo "missing engine ${pair#*=}" >&2; exit 1; }
    candidates+=(--candidate "$pair")
done

score() {
    local gt=$1 taxonomy=$2 out=$3
    echo "=== $out"
    venv/bin/python training/model_eval/score.py "$gt" "${candidates[@]}" --labels "$LABELS" \
        --taxonomy "$taxonomy" --conf "$CONF" --baseline "$BASELINE" --bootstrap 1000 \
        --output "$out"
}

want() {
    [ "$RUNS" = all ] || [[ ",$RUNS," == *",$1,"* ]]
}

OPPONENT=training/model_eval/taxonomy_opponent.yaml
if want opponent; then score "$EVAL" "$OPPONENT" "$OUT/opponent"; fi
if want heading; then score "$EVAL" training/model_eval/taxonomy_keypoint_ours.yaml "$OUT/heading"; fi
if want venues; then
    score "$VENUES/nhrl" "$OPPONENT" "$OUT/venue_nhrl"
    score "$VENUES/massd" "$OPPONENT" "$OUT/venue_massd"
fi
if want recordings; then
    for recording in "$EVAL"/*/; do
        [ -f "$recording/data.yaml" ] || continue
        score "$recording" "$OPPONENT" "$OUT/rec_$(basename "$recording")"
    done
fi
