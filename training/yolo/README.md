# yolo

YOLO detector and pose training: build a dataset, train, export a TensorRT engine.
Every file here is one CLI. Run them from the repo root.

The scripts stay flat rather than nested in subdirectories because experiment
write-ups under `docs/experiments/` cite these paths in the commands they record.
This README is the grouping instead.

Scoring the engines you produce is `training/model_eval/`.

## Pipeline

```bash
# 1. build a dataset
python training/yolo/pool_datasets.py --real <real> --synth <synthetic> \
  --synth-fraction 0.5 --synth-order random --out <out>
python training/yolo/validate_yolo_integrity.py <dataset>

# 2. train (submit through the queue, never directly; see CLAUDE.md)
venv/bin/python training/gpu_queue.py submit --name <arm> --by <agent> -- \
  venv/bin/python training/yolo/train.py <dataset> yolo26s -d 0 1 2 -b 96 -e 100

# 3. export
python training/yolo/convert_to_onnx.py <run>/weights/best.pt
python training/yolo/convert_to_tensorrt.py <run>/weights/best.onnx
```

## Train and export

| Script | Purpose |
| --- | --- |
| `train.py` | Main trainer. Runs land in `runs/projects/`. |
| `fine_tune_train.py` | Fine-tune an existing checkpoint. |
| `convert_to_onnx.py` | `.pt` to ONNX. |
| `convert_to_tensorrt.py` | ONNX to a TensorRT engine, including INT8 calibration. Shares its builder scaffolding with the DeepLab converter via `auto_battlebot/tensorrt_build.py`. |
| `test_tensorrt_video.py` | Run an engine over a video to eyeball it. |
| `test_tensorrt_image.py` | Run an engine over still images, directories, or globs. |
| `plot_results.py` | Plot a run's `results.csv`. |
| `clear_image_cache.py` | Delete ultralytics `cache="disk"` caches, keeping recent ones. Epoch speed depends on the `.npy` cache fitting in the page cache. |

## Build a dataset

| Script | Purpose |
| --- | --- |
| `make_seg_dataset.py` | Flat YOLO-seg dataset from one or more segmentation datasets. `floor_only` mode produces the floor labels the DeepLab field mask trains on. |
| `merge_yolo_datasets.py` | Merge several YOLO datasets into one. |
| `pool_datasets.py` | Pool a real dataset with a fraction of a synthetic one. |
| `split_yolo_dataset.py` | Split flat `images/` + `labels/` into train/val/test. |
| `split_by_scene.py` | Split into scene-disjoint groups, so a scene never straddles train and val. |
| `make_scaling_splits.py` | Data-scaling arms: one shared val set plus nested subsets. |
| `make_stretched_dataset.py` | Anisotropically-resized copy, for the stretch arm. |
| `make_uniform_aspect_subset.py` | Uniform-aspect-ratio view, for rectangular training. |

## Edit labels

| Script | Purpose |
| --- | --- |
| `remap_labels.py` | Remap class IDs. |
| `remap_labels_by_name.py` | Merge per-scene datasets, remapping classes by name. |
| `remap_exports_by_name.py` | Remap each per-scene export to a shared vocabulary, keeping the per-export layout. |
| `edit_yolo_classes.py` | Drop, prune, rename, and renumber classes. |
| `delete_labels.py` | Remove class IDs from label files. |
| `remove_failed_annotations.py` | Drop annotations that failed review. |
| `merge_validation_state.py` | Merge per-export `validation_state.json` files up to the dataset root. |
| `seg_to_bbox.py` | Segmentation polygons to bbox-only. |
| `pose_to_bbox.py` | Pose (keypoint) labels to bbox-only. |
| `seg_to_boxonly_pose.py` | Opponent segmentation labels to box-only, masked-keypoint pose labels. |

The `remap_config_*.toml` files are inputs to the remap scripts.

## Validate

| Script | Purpose |
| --- | --- |
| `validate_yolo_integrity.py` | Check a dataset is internally consistent and safe to train on. Run this before every training job. |
| `validate_yolo_dataset.py` | Interactive validation UI. Writes `validation_state.json`, which is what `score.py` gates ground truth on. |
