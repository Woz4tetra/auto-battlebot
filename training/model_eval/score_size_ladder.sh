#!/usr/bin/env bash
# Score the yolo26 pose size ladder on nhrl_keypoints_eval_test in one paired bootstrap per run.
#
# docs/experiments/perception_performance/pose_model_size_ladder_plan_2026-09-24.md. Every size
# goes into one score.py call, so the bootstrap pairs them all against the baseline. That is what
# score_domain_mix.sh cannot do: it takes one model key and many arms, and this takes one arm and
# many model keys.
#
#   bash training/model_eval/score_size_ladder.sh 2026-09-24 n s m l x
#
# Engines are data/models/yolo26<size>-pose_<arm>_<date>_last_<shape>_x86_64_<sm>.engine, the name
# run_domain_mix_arm.sh gives them, with <shape> the rectangular export (SHAPE=rect384x640, the
# default) or nothing for the square one (SHAPE=square). EXTRA adds more candidates as
# `name=engine` pairs, which is how the grid's square anchors join the square run:
#
#   SHAPE=square RUNS=opponent,heading OUT=.../anchors \
#   EXTRA="swap_half=data/models/yolo26s-pose_swap_half_2026-09-13_last_x86_64_sm89.engine" \
#       bash training/model_eval/score_size_ladder.sh 2026-09-24 s x
#
# Output, under OUT (default $EVAL/scores_size_ladder, with a conf suffix off 0.5):
#   opponent/      all frames, taxonomy_opponent.yaml: recall for Q1 to Q3
#   heading/       all frames, taxonomy_keypoint_ours.yaml: heading error for Q2 to Q4
#   venue_nhrl/    the 590 NHRL May frames, opponents only
#   venue_massd/   the 98 MassD August frames, opponents only
#   rec_<name>/    one recording, which is one opponent, opponents only
#
# Environment: ARM (default d50000), BASELINE (default s), SHAPE, SM (default sm89, pathfinder),
# CONF (default 0.5), EVAL (default the full eval set; a split root for the threshold read), OUT,
# RUNS (default all: opponent, heading, venues, recordings), EXTRA.
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
SM=${SM:-sm89}
CONF=${CONF:-0.5}
RUNS=${RUNS:-all}

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$REPO"
EVAL=${EVAL:-training/data/nhrl_keypoints_eval_test}
# Symlink roots over the eval set's recordings, each with its own slice of validation_state.json,
# because the loader only reads review state in the directory it is pointed at.
VENUES=training/data/nhrl_keypoints_eval_test_by_venue
suffix=""
[ "$CONF" = 0.5 ] || suffix="_conf${CONF}"
OUT=${OUT:-training/data/nhrl_keypoints_eval_test/scores_size_ladder${suffix}}
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
    score "$VENUES/nhrl_may" "$OPPONENT" "$OUT/venue_nhrl"
    score "$VENUES/massd_aug" "$OPPONENT" "$OUT/venue_massd"
fi
if want recordings; then
    # Every label file in a recording is a `pass` frame, so a bare recording scores the same
    # frames the gated root does.
    for recording in "$EVAL"/*/; do
        [ -f "$recording/data.yaml" ] || continue
        score "$recording" "$OPPONENT" "$OUT/rec_$(basename "$recording")"
    done
fi
