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
# Output: training/data/nhrl_keypoints_eval_test/scores_domain_mix_<model>/
#   pooled/        all 688 frames, taxonomy.yaml (opponent and house_bot scored with our robots)
#   heading/       all 688 frames, taxonomy_keypoint_ours.yaml (our robot's keypoints only)
#   venue_nhrl/    the 590 NHRL May frames
#   venue_massd/   the 98 MassD August frames
#   rec_<name>/    one recording, which is one opponent
# Every run keeps the bootstrap. 98 MassD frames is where a point estimate misleads most.
#
# Environment: CKPT (default last), SM (default sm86, this box), OUT, CONF (default 0.5).
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

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$REPO"
EVAL=training/data/nhrl_keypoints_eval_test
# Symlink roots over the eval set's recordings, each with the eval set's validation_state.json,
# because the loader only reads review state in the directory it is pointed at.
VENUES=training/data/nhrl_keypoints_eval_test_by_venue
OUT=${OUT:-$EVAL/scores_domain_mix_${MODEL}}
LABELS="mr_stabs_mk2,mrs_buff_mk3,opponent,house_bot"

candidates=()
for arm in "${ARMS[@]}"; do
    engine="data/models/${MODEL}_${arm}_${DATE}_${CKPT}_x86_64_${SM}.engine"
    [ -f "$engine" ] || { echo "missing engine $engine" >&2; exit 1; }
    candidates+=(--candidate "$arm=$engine")
done

score() {
    local gt=$1 taxonomy=$2 out=$3
    echo "=== $out"
    venv/bin/python training/model_eval/score.py "$gt" "${candidates[@]}" --labels "$LABELS" \
        --taxonomy "$taxonomy" --conf "$CONF" --baseline "${ARMS[0]}" --bootstrap 1000 \
        --output "$out"
}

score "$EVAL" training/model_eval/taxonomy.yaml "$OUT/pooled"
score "$EVAL" training/model_eval/taxonomy_keypoint_ours.yaml "$OUT/heading"
score "$VENUES/nhrl_may" training/model_eval/taxonomy.yaml "$OUT/venue_nhrl"
score "$VENUES/massd_aug" training/model_eval/taxonomy.yaml "$OUT/venue_massd"
for recording in "$EVAL"/*/; do
    [ -f "$recording/data.yaml" ] || continue
    score "$recording" training/model_eval/taxonomy.yaml "$OUT/rec_$(basename "$recording")"
done
