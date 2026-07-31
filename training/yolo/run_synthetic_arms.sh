#!/usr/bin/env bash
# Three-arm synthetic-vs-real experiment (docs/experiments/perception_performance/synthetic_arms_plan.md).
#
# Sequential, not parallel: each arm gets the same 3-GPU DDP recipe, so arm-to-arm differences
# come from the training set and nothing else. Run dirs are renamed per arm afterwards because
# train.py names every run by date + model key alone.
set -u

REPO=/home/ben/auto-battlebot
LOGDIR=$REPO/training/projects/synth_arms_logs
ARMS=(
    "real_only:nhrl_robots_bbox_2class"
    "synth_only:synth_only_2class"
    "mixed:mixed_2class"
)

mkdir -p "$LOGDIR"
cd "$REPO" || exit 1
# shellcheck disable=SC1091
source scripts/activate_python.sh
cd "$REPO/training/yolo" || exit 1

for entry in "${ARMS[@]}"; do
    name=${entry%%:*}
    dataset=${entry##*:}
    echo "=== START $name ($dataset) $(date -Is)"
    python train.py "../data/$dataset/data.yml" yolo26n -e 100 --save-period 25 --seed 0 \
        2>&1 | tee "$LOGDIR/$name.log"
    status=${PIPESTATUS[0]}
    echo "=== END $name status=$status $(date -Is)"

    newest=$(ls -dt ../projects/auto_battlebots_* 2>/dev/null | head -1)
    if [ -n "$newest" ]; then
        target="../projects/arm_${name}_$(basename "$newest")"
        mv "$newest" "$target" && echo "=== run dir -> $target"
    fi
done

echo "=== ALL ARMS DONE $(date -Is)"
