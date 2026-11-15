#!/usr/bin/env bash
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

for arm in "${ARMS[@]}"; do
    weights="data/models/${MODEL}_${arm}_${DATE}_${CKPT}.pt"
    [ -f "$weights" ] || { echo "missing weights $weights" >&2; exit 1; }
done

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
