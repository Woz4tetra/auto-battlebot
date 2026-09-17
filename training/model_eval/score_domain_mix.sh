#!/usr/bin/env bash
# Score domain-mix arms on nhrl_keypoints_eval_test: pooled, heading, per venue, per recording.
#
# Step 5 of docs/experiments/perception_performance/synthetic_domain_mix_plan_2026-09-12.md. Reads
# each arm's square-640 engine at data/models/<model>_<arm>_<date>_<ckpt>_x86_64_<sm>.engine, the
# name run_domain_mix_arm.sh gives it. The first arm is the baseline every other arm is bootstrapped
# against.
#
#   bash training/model_eval/score_domain_mix.sh yolo26s-pose 2026-09-13 base d2500 d5000 d10000
#
# An arm may carry its own checkpoint as `arm:ckpt`, which overrides CKPT for that arm alone and
# names it `arm_ckpt` in the output. The matched-presentation table needs this: each arm is read at
# the checkpoint bracketing the baseline's frame-presentations, which is a different epoch per arm.
#
#   RUNS=opponent bash training/model_eval/score_domain_mix.sh yolo26s-pose 2026-09-13 \
#       base d20000:epoch50 d40000:epoch25
#
# Output: training/data/nhrl_keypoints_eval_test/scores_domain_mix_<model>/
#   pooled/        all 688 frames, taxonomy.yaml (every class scored together)
#   opponent/      all 688 frames, taxonomy_opponent.yaml: the adoption metric
#   heading/       all 688 frames, taxonomy_keypoint_ours.yaml (our robot's keypoints only)
#   venue_nhrl/    the 590 NHRL May frames, opponents only
#   venue_massd/   the 98 MassD August frames, opponents only
#   rec_<name>/    one recording, which is one opponent, opponents only
# Every run keeps the bootstrap. 98 MassD frames is where a point estimate misleads most.
#
# Environment: CKPT (default last), SM (default sm86, this box), OUT, CONF (default 0.5), and RUNS
# (default all), a comma-separated subset of pooled, opponent, heading, venues and recordings. The
# matched-presentation read wants `opponent` alone: twelve runs over ten candidates is an hour of
# GPU for one column of one table.
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
    # `arm:ckpt` pins one arm to its own checkpoint; a bare arm takes CKPT.
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
