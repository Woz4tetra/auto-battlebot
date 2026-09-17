#!/usr/bin/env bash
# Train one domain-mix arm, keep its checkpoints under the arm's name, and build its engine.
#
# Step 4 of docs/experiments/perception_performance/synthetic_domain_mix_plan_2026-09-12.md. One
# queue job per arm, so the queue's ordering and time estimates see each arm separately:
#
#   venv/bin/python training/gpu_queue.py submit --name dm_s_base --by <agent> -d 0 1 2 \
#     --work 1844700 --profile yolo26s-pose@640 -- \
#     bash training/yolo/run_domain_mix_arm.sh training/data/domain_mix_arms_2026-09-13 base yolo26s-pose
#
# --work is the arm's frames times its epochs, read off manifest.json in the arms directory
# (`base` is 18,447 x 100). The queue cannot see either number here: epochs arrive positionally
# rather than as -e, and the frame count lives in the arm list. Without --work every arm of every
# size predicts the same wall time.
#
# train.py names every run by date and model key alone, so the run directory is found as the one
# this job created, and the weights are copied to data/models/<model>_<arm>_<date>_<ckpt>.pt with
# the --save-period 25 checkpoints alongside last.pt for the matched-step reads. The last.pt engine
# is built square at 640, the geometry every earlier pose arm was scored at; the epoch checkpoints
# stay .pt until a matched-step table needs them.
set -euo pipefail

if [ $# -lt 3 ]; then
    echo "usage: $0 <arms_dir> <arm> <model_key> [epochs]" >&2
    exit 2
fi
ARMS_DIR=$1
ARM=$2
MODEL=$3
EPOCHS=${4:-100}
SAVE_PERIOD=25
DATE=${ARM_DATE:-$(date +%F)}

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$REPO"
PY=venv/bin/python
YML="$ARMS_DIR/$ARM.yml"
[ -f "$YML" ] || { echo "no arm yaml at $YML" >&2; exit 2; }

marker=$(mktemp)
trap 'rm -f "$marker"' EXIT
echo "=== train $ARM ($MODEL, $EPOCHS epochs) $(date -Is)"
$PY training/yolo/train.py "$YML" "$MODEL" -d 0 1 2 -b 96 -e "$EPOCHS" --imgsz 640 \
    --save-period "$SAVE_PERIOD" --seed 0 --cache ram

run_dir=$(find runs/projects training/projects -maxdepth 1 -type d -newer "$marker" \
    -name "auto_battlebots_*_${MODEL}" 2>/dev/null | sort | tail -1)
[ -n "$run_dir" ] || { echo "no run directory newer than this job for $MODEL" >&2; exit 1; }
echo "=== run dir $run_dir"
echo "$ARM $MODEL $YML" > "$run_dir/domain_mix_arm.txt"

stem="data/models/${MODEL}_${ARM}_${DATE}"
for ckpt in last $(seq -f "epoch%g" "$SAVE_PERIOD" "$SAVE_PERIOD" "$EPOCHS"); do
    if [ -f "$run_dir/weights/$ckpt.pt" ]; then
        cp "$run_dir/weights/$ckpt.pt" "${stem}_${ckpt}.pt"
        echo "=== kept ${stem}_${ckpt}.pt"
    fi
done

$PY training/yolo/convert_to_onnx.py "${stem}_last.pt"
$PY training/yolo/convert_to_tensorrt.py "${stem}_last.onnx" --workspace 4
echo "=== done $ARM $(date -Is)"
