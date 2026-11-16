#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 3 ]; then
    echo "usage: $0 <model_key> <date> <baseline_arm> [arm ...]" >&2
    exit 2
fi
MODEL=$1
DATE=$2
shift 2
ARMS=("$@")
CKPT=${CKPT:-last}
SM=${SM:-sm86}
CONF=${CONF:-0.5}
RUNS=${RUNS:-all}

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$REPO"
EVAL=training/data/nhrl_keypoints_eval_test
# Symlink roots over the eval set's recordings, each with the eval set's validation_state.json,
# because the loader only reads review state in the directory it is pointed at.
VENUES=training/data/nhrl_keypoints_eval_test_by_venue
OUT=${OUT:-$EVAL/scores_domain_mix_${MODEL}}
LABELS="mr_stabs_mk2,mrs_buff_mk3,opponent,house_bot"

candidates=()
BASELINE=""
for arm in "${ARMS[@]}"; do
    ckpt=${arm#*:}
    [ "$ckpt" = "$arm" ] && ckpt=$CKPT
    arm=${arm%%:*}
    name=$arm
    [ "$ckpt" = "$CKPT" ] || name="${arm}_${ckpt}"
    engine="data/models/${MODEL}_${arm}_${DATE}_${ckpt}_x86_64_${SM}.engine"
    [ -f "$engine" ] || { echo "missing engine $engine" >&2; exit 1; }
    candidates+=(--candidate "$name=$engine")
    [ -n "$BASELINE" ] || BASELINE=$name
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
if want pooled; then score "$EVAL" training/model_eval/taxonomy.yaml "$OUT/pooled"; fi
if want opponent; then score "$EVAL" "$OPPONENT" "$OUT/opponent"; fi
if want heading; then score "$EVAL" training/model_eval/taxonomy_keypoint_ours.yaml "$OUT/heading"; fi
if want venues; then
    score "$VENUES/nhrl_may" "$OPPONENT" "$OUT/venue_nhrl"
    score "$VENUES/massd_aug" "$OPPONENT" "$OUT/venue_massd"
fi
if want recordings; then
    for recording in "$EVAL"/*/; do
        [ -f "$recording/data.yaml" ] || continue
        score "$recording" "$OPPONENT" "$OUT/rec_$(basename "$recording")"
    done
fi
