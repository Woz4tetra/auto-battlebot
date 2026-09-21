#!/usr/bin/env bash
# Train one domain-mix arm, keep its checkpoints under the arm's name, and build its engines.
#
# Step 4 of docs/experiments/perception_performance/synthetic_domain_mix_plan_2026-09-12.md and
# step 2 of pose_only_perception_plan_2026-09-19.md. One queue job per arm, so the queue's
# ordering and time estimates see each arm separately:
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
# the --save-period checkpoints alongside last.pt for the matched-step reads. Two engines are
# built from last.pt: the square one at the training size, the geometry every earlier pose arm was
# scored at, and a rectangular one at --export-shape, which is what the deployed model runs
# (pose_only_report_2026-09-19.md: 384x640 costs 19.94 ms on the Orin NX against 32.92 ms square).
# The epoch checkpoints stay .pt until a matched-step table needs them.
set -euo pipefail

usage() {
    cat >&2 <<'USAGE'
usage: run_domain_mix_arm.sh <arms_dir> <arm> <model_key> [epochs] [options]

options:
  --save-period N      checkpoint every N epochs (default 25, 0 disables)
  --label NAME         name the kept weights after NAME instead of <arm>, so two arms
                       that share an arm list and a model key do not overwrite each other
  --imgsz N            training input size (default 640)
  --batch N            total batch across GPUs (default 96)
  --cache ram|disk|false   image cache (default ram)
  --export-shape HxW   rectangular engine geometry (default 384x640)
USAGE
    exit 2
}

[ $# -ge 3 ] || usage
ARMS_DIR=$1
ARM=$2
MODEL=$3
shift 3

EPOCHS=100
if [ $# -gt 0 ] && [[ $1 != --* ]]; then
    EPOCHS=$1
    shift
fi

SAVE_PERIOD=25
LABEL=$ARM
IMGSZ=640
BATCH=96
CACHE=ram
EXPORT_SHAPE=384x640
while [ $# -gt 0 ]; do
    case $1 in
        --save-period) SAVE_PERIOD=$2; shift 2 ;;
        --label) LABEL=$2; shift 2 ;;
        --imgsz) IMGSZ=$2; shift 2 ;;
        --batch) BATCH=$2; shift 2 ;;
        --cache) CACHE=$2; shift 2 ;;
        --export-shape) EXPORT_SHAPE=$2; shift 2 ;;
        *) echo "unknown option $1" >&2; usage ;;
    esac
done

[[ $EXPORT_SHAPE =~ ^[0-9]+x[0-9]+$ ]] || { echo "--export-shape wants HxW, got $EXPORT_SHAPE" >&2; exit 2; }
EXPORT_H=${EXPORT_SHAPE%x*}
EXPORT_W=${EXPORT_SHAPE#*x}

# Stamped before training, not after, so an arm that runs past midnight keeps one date.
DATE=${ARM_DATE:-$(date +%F)}

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$REPO"
PY=venv/bin/python
YML="$ARMS_DIR/$ARM.yml"
[ -f "$YML" ] || { echo "no arm yaml at $YML" >&2; exit 2; }

marker=$(mktemp)
trap 'rm -f "$marker"' EXIT
echo "=== train $LABEL ($MODEL, $EPOCHS epochs, imgsz $IMGSZ, batch $BATCH, cache $CACHE) $(date -Is)"
$PY training/yolo/train.py "$YML" "$MODEL" -d 0 1 2 -b "$BATCH" -e "$EPOCHS" --imgsz "$IMGSZ" \
    --save-period "$SAVE_PERIOD" --seed 0 --cache "$CACHE"

run_dir=$(find runs/projects training/projects -maxdepth 1 -type d -newer "$marker" \
    -name "auto_battlebots_*_${MODEL}" 2>/dev/null | sort | tail -1)
[ -n "$run_dir" ] || { echo "no run directory newer than this job for $MODEL" >&2; exit 1; }
echo "=== run dir $run_dir"
echo "$LABEL $ARM $MODEL $YML imgsz=$IMGSZ batch=$BATCH" > "$run_dir/domain_mix_arm.txt"

ckpts=(last)
if [ "$SAVE_PERIOD" -gt 0 ]; then
    mapfile -t -O "${#ckpts[@]}" ckpts < <(seq -f "epoch%g" "$SAVE_PERIOD" "$SAVE_PERIOD" "$EPOCHS")
fi
stem="data/models/${MODEL}_${LABEL}_${DATE}"
for ckpt in "${ckpts[@]}"; do
    if [ -f "$run_dir/weights/$ckpt.pt" ]; then
        cp "$run_dir/weights/$ckpt.pt" "${stem}_${ckpt}.pt"
        echo "=== kept ${stem}_${ckpt}.pt"
    fi
done

# Rectangular first, then square. convert_to_onnx.py always writes the default <stem>_last.onnx
# and only then moves it to -o, so running square first leaves no square .onnx behind -- which is
# the file a Jetson needs to rebuild the engine for its own TensorRT version.
echo "=== export rectangular ${EXPORT_H}x${EXPORT_W}"
rect="${stem}_last_rect${EXPORT_H}x${EXPORT_W}"
$PY training/yolo/convert_to_onnx.py "${stem}_last.pt" --imgsz "$EXPORT_H" "$EXPORT_W" -o "${rect}.onnx"
$PY training/yolo/convert_to_tensorrt.py "${rect}.onnx" --workspace 4

echo "=== export square ${IMGSZ}x${IMGSZ}"
$PY training/yolo/convert_to_onnx.py "${stem}_last.pt" --imgsz "$IMGSZ"
$PY training/yolo/convert_to_tensorrt.py "${stem}_last.onnx" --workspace 4
echo "=== done $LABEL $(date -Is)"
