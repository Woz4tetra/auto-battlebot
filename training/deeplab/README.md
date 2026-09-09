# deeplab

Field-mask segmentation: the model that tells the perception stack which pixels are
arena floor. `DeepLabMaskModel` is what every real config selects
(`config/_common.toml`, `_desktop.toml`, `_jetson.toml`), so this is a live deploy
path, not an experiment. The deployed artifact is
`data/models/field_deeplabv3p_r50_2026-07-29_*.engine`.

Every file here is one CLI. Run them from the repo root. The model builder, checkpoint
metadata, input geometry, and filename parsing live in `auto_battlebot/segmentation/`,
because `playground/bgsub_cage/` uses them too.

Floor labels come from `training/yolo/make_seg_dataset.py` in `floor_only` mode.

## Corpus to engine

```bash
# 1. inventory and clean the floor-mask corpus
python training/deeplab/build_field_manifest.py --out <manifest>
python training/deeplab/make_field_splits.py --manifest <manifest> --out <splits>
python training/deeplab/build_field_dataset.py --src <segmask_root> --out <dataset>

# 2. train, then score checkpoints against the held-out set
python training/deeplab/semantic_train.py --backbone r50 -o <run>
python training/deeplab/score_masks.py <dataset>/val \
  --candidate new=<run>/best.pth --by-field

# 3. export
python training/deeplab/convert_to_tensorrt.py <run>/best.pth
```

## Build the corpus

| Script | Purpose |
| --- | --- |
| `build_field_manifest.py` | Inventory the floor-mask corpus: field type, scene, and provenance per frame. |
| `make_field_splits.py` | Deduplicated, scene-disjoint, field-stratified splits. |
| `build_field_dataset.py` | Assemble a segmask tree into one train/val dataset, split at video level. Supersedes the older frame-level splitters, whose shuffle leaked frames between train and val. |
| `convex_hull_masks.py` | Regenerate a segmask dataset with every mask blob replaced by its convex hull. |
| `build_field_crop_dataset.py` | Field-cropped corpus for the input-geometry arm. `score.py` reads its output layout. |

## Train, score, export

| Script | Purpose |
| --- | --- |
| `semantic_train.py` | Trainer. |
| `score_masks.py` | Score checkpoints against held-out ground truth, per field type: macro-average IoU and boundary F1. |
| `field_fraction_gate.py` | Go/no-go gate for the field-crop arm. |
| `convert_to_onnx.py` | `.pth` to ONNX. |
| `convert_to_tensorrt.py` | `.pth` or ONNX to a TensorRT engine. Shares its builder scaffolding with the YOLO converter via `auto_battlebot/tensorrt_build.py`. |

## Inspect

| Script | Purpose |
| --- | --- |
| `validate_segmask_dataset.py` | Dataset validation UI. |
| `view_mask.py` | Interactive viewer for an image and mask pair. |

`field_overrides.toml` corrects field-type assignments `build_field_manifest.py` gets
wrong. `remap_robots.toml` and `remap_segmask_floor.toml` are label-remap inputs.
