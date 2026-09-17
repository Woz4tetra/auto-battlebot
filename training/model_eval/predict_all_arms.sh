#!/usr/bin/env bash
# Label one dataset with every domain-mix arm, each arm into its own directory, for a side-by-side
# read of which arm labels the footage best.
#
# `prelabel_dataset.py` fills only empty label files, on purpose: a second pre-label round must not
# overwrite frames someone has corrected. Comparing arms wants the opposite, every frame labelled by
# every arm, so each arm gets its own copy of the dataset whose images are hardlinked and whose
# `labels/` starts out missing. The source dataset is never written to, and the hardlinks mean
# fifteen arms cost fifteen sets of label files rather than fifteen copies of the images.
#
#   bash training/model_eval/predict_all_arms.sh training/data/nhrl_cage_high_eval \
#       training/data/cage_high_arm_labels_2026-09-16 yolo26s-pose 2026-09-13 base d2500 swap_half
#
# Output: <out_root>/<arm>/<recording>/{images,labels,data.yaml}, each openable in edit_labels.py
# and validate_yolo_dataset.py. Each recording also carries `prelabel_holdout.json`, which records
# the model, conf and class map that wrote its labels.
#
# Environment: CONF (default 0.15, the pre-label floor: deleting a spurious box is one keypress,
# drawing a missed one is a dozen), CKPT (default last), DEVICE (default 0), IMGSZ (default 640),
# MAP (default "nhrl_robot=opponent", the arms' class for an opponent against this dataset's name),
# FORCE (set to rebuild an arm whose directory already exists).
set -euo pipefail

if [ $# -lt 5 ]; then
    echo "usage: $0 <source_dataset> <out_root> <model_key> <date> <arm> [arm ...]" >&2
    exit 2
fi
SOURCE=$1
OUT_ROOT=$2
MODEL=$3
DATE=$4
shift 4
ARMS=("$@")
CONF=${CONF:-0.15}
CKPT=${CKPT:-last}
DEVICE=${DEVICE:-0}
IMGSZ=${IMGSZ:-640}
MAP=${MAP:-nhrl_robot=opponent}
FORCE=${FORCE:-}

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$REPO"
[ -d "$SOURCE" ] || { echo "no dataset at $SOURCE" >&2; exit 1; }

# Every arm's weights must exist before the first one runs, so a typo fails in seconds rather than
# after an hour of inference.
for arm in "${ARMS[@]}"; do
    weights="data/models/${MODEL}_${arm}_${DATE}_${CKPT}.pt"
    [ -f "$weights" ] || { echo "missing weights $weights" >&2; exit 1; }
done

# The recordings to copy: the dataset itself if it holds images/, else every child that does.
recordings=()
if [ -d "$SOURCE/images" ]; then
    recordings=("$SOURCE")
else
    for child in "$SOURCE"/*/; do
        [ -d "$child/images" ] && recordings+=("${child%/}")
    done
fi
[ ${#recordings[@]} -gt 0 ] || { echo "$SOURCE holds no images/ directory" >&2; exit 1; }
echo "=== ${#recordings[@]} recordings, ${#ARMS[@]} arms, conf $CONF, device $DEVICE"

for arm in "${ARMS[@]}"; do
    dest="$OUT_ROOT/$arm"
    if [ -d "$dest" ] && [ -z "$FORCE" ]; then
        echo "=== $arm: have $dest, skipping (set FORCE=1 to rebuild)"
        continue
    fi
    [ -z "$FORCE" ] || rm -rf "$dest"
    echo "=== $arm"
    for recording in "${recordings[@]}"; do
        name=$(basename "$recording")
        mkdir -p "$dest/$name"
        # -l hardlinks the frames, so the images cost nothing per arm. labels/ is left absent:
        # prelabel_dataset.py treats a missing label file as empty and writes every frame.
        cp -al "$recording/images" "$dest/$name/images"
        for meta in data.yaml data.yml; do
            [ -f "$recording/$meta" ] && cp "$recording/$meta" "$dest/$name/$meta"
        done
    done
    venv/bin/python training/model_eval/prelabel_dataset.py "$dest" \
        --model "data/models/${MODEL}_${arm}_${DATE}_${CKPT}.pt" \
        --map "$MAP" --conf "$CONF" --imgsz "$IMGSZ" --device "$DEVICE" --holdout 0
done

echo "=== done; compare with: venv/bin/python training/yolo/validate_yolo_dataset.py $OUT_ROOT/<arm>/<recording>"
