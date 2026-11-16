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
venv/bin/python training/gpu_queue.py submit --name <arm> --by <agent> \
  --work <frames x epochs> --profile yolo26s@640 -- \
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

## Notes moved from code comments (2026-11-15)

- `training/deeplab/convert_to_tensorrt.py` (`above def export_onnx(`):

  ```text
  TensorRT's tactic timings are keyed by layer shape and type, so they are reusable across builds
  of the same architecture with different weights. Persisting them turns each build after the first
  from a kernel-autotuning search into little more than weight serialisation.

  Repo-local so the cache travels with the checkout and is obviously disposable; `.cache/` is
  already gitignored. Not in $HOME, where it would silently outlive the project and be easy to
  forget when a TensorRT or driver upgrade invalidates it. Shared with the YOLO converter: entries
  are content-keyed, so unrelated architectures coexist in one file without colliding.
  ```

- `training/yolo/clear_image_cache.py`, `sweep`:
  Delete stale caches under ``root``. Returns (bytes freed, files removed).
  The importable entry point, used by ``train.py`` to reclaim disk before a run. ``protect`` names datasets to spare regardless of age: ``busy_datasets`` skips this process and its ancestors, so a caller about to train on a dataset must name it explicitly or risk deleting its own cache (harmless, it regenerates, but a slow way to start a run).
- `training/yolo/convert_to_tensorrt.py`:

  ```text
  Convert a YOLO pose model in ONNX form to TensorRT engine format.

  The input is always .onnx; export a checkpoint first with convert_to_onnx.py. Building
  through TensorRT Builder + OnnxParser rather than Ultralytics model.export() keeps the
  plan format the one the C++ YoloKeypointModel can load, and needs no Ultralytics.

    python training/yolo/convert_to_onnx.py model.pt
    python training/yolo/convert_to_tensorrt.py model.onnx -o data/models/model.engine

  Engines are GPU- and TensorRT-version specific: an engine built on x86 cannot
  run on Jetson (or vice versa). For Jetson deployment, copy the .onnx to the
  Jetson and run this script there to produce the .engine used by the C++ app.

  The C++ YoloKeypointModel expects:
  - Input: single tensor, shape [1, 3, H, W] (NCHW, float32), e.g. [1, 3, 640, 640].
  - Output: single tensor, shape [1, num_features, num_predictions], e.g. [1, 56, 8400]
    (features = 4 bbox + num_classes + num_keypoints*3).

  Output filenames include a platform tag (e.g. _x86_64_sm89, _aarch64_sm72) that
  encodes both CPU architecture and GPU compute capability so incompatible engines
  are not loaded by accident. --int8 adds a precision tag ahead of it
  (model_int8_x86_64_sm86.engine), leaving FP16 filenames byte-identical.

  INT8 needs calibration: activation ranges depend on the data, so real frames are run
  through the network to measure them.

    python training/yolo/convert_to_tensorrt.py model.onnx --int8       --calib-dir training/data/nhrl_robots_bbox_2class/val/images --calib-count 1000
  ```

- `training/yolo/crop_yolo_dataset.py`:

  ```text
  Crop a YOLO dataset's images and annotations to a pixel rectangle, in place.

  Broadcast footage carries banners the camera never sees: the MassDestruction streams put
  a scoreboard above the cage view and a ticker below it. Training or scoring on those rows
  teaches the detector about graphics. This crops the images and moves the labels with them,
  so a hand-labeled set stays usable without relabeling.

  Boxes are shifted into the crop and clipped to it; a box left thinner than --min-box-px on
  either axis is dropped. Keypoints that land outside the crop are written as
  ``0.000000 0.000000 0`` (visibility 0 = out-of-frame, the convention in
  training/synthetic/synthgen/annotations.py); the rest keep their visibility flag. Rows that
  are neither detect (5 values) nor pose (5 + 3k) abort the run before anything is written,
  since a polygon cannot be clipped this way.

  Each recording's data.yaml gains a ``crop``/``source_size`` record, which also makes the
  pass idempotent: an image already at the crop size is left alone rather than cropped twice.

  Usage:
    # one recording, or a root of subdatasets filtered by directory name
    python training/yolo/crop_yolo_dataset.py training/data/cage_high_x50_conf044       --rect 0 94 1920 895 --match 'r[123]_*' --dry-run
  ```

- `training/yolo/make_seg_dataset.py`, `_build_hole_cut_buffers` (`above cutters = []`):
  Build tiny cuts from each hole to the exterior boundary. Removing those slits turns holey polygons into regular polygons without fan triangulation.
- `training/yolo/merge_validation_state.py`:

  ```text
  Merge per-export `validation_state.json` files into one at the dataset root.

  `validate_yolo_dataset.py` keys its verdicts on the image path relative to whatever directory it
  was opened on. Open a single export and the keys read `images/frame_000000.jpg`; open a tree of
  exports and the same frame is `<export>/images/frame_000000.jpg`. The verdicts are the same
  human review either way, but the keys do not match, so a tree opened at the root starts from
  zero even though every frame under it has already been judged.

  This rewrites the per-export keys to be root-relative and writes the union. Frames whose image is
  no longer on disk are dropped, since a verdict pointing at nothing only makes the counts lie.

  Usage:
      python training/yolo/merge_validation_state.py <dataset_root>
  ```

- `training/yolo/pose_to_bbox.py`:

  ```text
  Convert a YOLO pose (keypoint) dataset into a detection (bbox-only) dataset.

  Preserves the train/val/test split structure exactly. Each pose label row
  ``class cx cy w h  kx ky kv  kx ky kv ...`` becomes a box row ``class cx cy w h``:
  the keypoint columns are dropped, the box columns pass through verbatim. Rows already
  in box form (4 coords) pass through unchanged. Optionally remaps class ids so the
  output lands in a different class vocabulary (e.g. fold a synthetic pose dataset into
  a real detection dataset's classes). Images are hardlinked (fast, no copy) when the
  output is on the same filesystem, otherwise copied; ``.npy`` training caches are skipped.

  Unlike seg_to_bbox.py, this must NOT treat the extra columns as polygon coordinates:
  a pose row's keypoints are not box geometry. Only the first four coords are the box.

  Built for the synthetic+bbox experiment: turn the synthetic keypoint dataset
  ``all_robot_keypoints`` into box-only labels remapped into the ``nhrl_robots_bbox``
  vocabulary, so it can be pooled with the real bbox set. See
  ``docs/experiments/perception_performance/synthetic_plus_bbox_plan.md``.

  Usage:
    python training/yolo/pose_to_bbox.py training/data/all_robot_keypoints       -o training/data/synth_bbox_from_keypoints       --class-map 2:1,0:3,1:4       --names-from training/data/nhrl_robots_bbox/data.yml
  ```

- `training/yolo/remap_exports_by_name.py`, `write_data_yml`:
  Write the export's data.yml.
  The split keys are inherited from what SegmentFlow emits and are decorative here, an export is a flat images/ + labels/ pair with no train/ or val/ under it. Downstream splitting tools write their own data.yml.
- `training/yolo/remap_exports_by_name.py`:

  ```text
  Remap each per-scene YOLO export to a shared vocabulary, keeping the per-export layout.

  `remap_labels_by_name.py` does the same name-based remap but flattens every scene into one
  merged dataset. That is right when the next step is training and wrong when the next step is
  review: the merge prefixes filenames and leaves the hand-made `validation_state.json` behind,
  so the frame verdicts in `validate_yolo_dataset.py` no longer line up.

  This tool writes a mirror of the source tree instead. One output directory per export, same
  image and label filenames, `validation_state.json` carried across, and a `data.yml` holding the
  new vocabulary. Source exports are never touched, so the per-robot class names survive.

  Config format is `remap_labels_by_name.py`'s: `target_names`, `default`, and a `[map]` of source
  class name -> target name (or `__drop__`). Names with no rule fall through to `default` and are
  all printed, so a new non-robot class appearing upstream is visible rather than silently folded
  in.

  Usage:
      python training/yolo/remap_exports_by_name.py         --src ~/Desktop/new_data         --config training/yolo/remap_config_2class.toml         --out ~/Desktop/new_data_2class
  ```

- `training/yolo/remap_labels_by_name.py`, `main` (`above out_images = args.out / "train" / "images"`):
  Written under train/ because split_by_scene.py and validate_yolo_integrity.py both expect split subdirectories; the scene split downstream re-partitions this anyway.
- `training/yolo/run_domain_mix_arm.sh` (`above set -euo pipefail`):

  ```text
  Train one domain-mix arm, keep its checkpoints under the arm's name, and build its engines.

  Step 4 of docs/experiments/perception_performance/synthetic_domain_mix_plan_2026-09-12.md and
  step 2 of pose_only_perception_plan_2026-09-19.md. One queue job per arm, so the queue's
  ordering and time estimates see each arm separately:

    venv/bin/python training/gpu_queue.py submit --name dm_s_base --by <agent> -d 0 1 2 \
      --work 1844700 --profile yolo26s-pose@640 -- \
      bash training/yolo/run_domain_mix_arm.sh training/data/domain_mix_arms_2026-09-13 base yolo26s-pose

  --work is the arm's frames times its epochs, read off manifest.json in the arms directory
  (`base` is 18,447 x 100). The queue cannot see either number here: epochs arrive positionally
  rather than as -e, and the frame count lives in the arm list. Without --work every arm of every
  size predicts the same wall time.

  train.py names every run by date and model key alone, so the run directory is found as the one
  this job created, and the weights are copied to data/models/<model>_<arm>_<date>_<ckpt>.pt with
  the --save-period checkpoints alongside last.pt for the matched-step reads. Two engines are
  built from last.pt: the square one at the training size, the geometry every earlier pose arm was
  scored at, and a rectangular one at --export-shape, which is what the deployed model runs
  (pose_only_report_2026-09-19.md: 384x640 costs 19.94 ms on the Orin NX against 32.92 ms square).
  The epoch checkpoints stay .pt until a matched-step table needs them.
  ```

- `training/yolo/split_by_scene.py`, `assign_stratified`:
  Greedily assign whole scenes to groups to match each group's target share.
  Scenes are placed scarcest-first (largest share of any single metric), so the rare classes get to pick their group before the bulk frames crowd the targets. Each scene goes wherever it minimizes the summed squared deviation from every group's target, across frame count and every stratified class at once.
- `training/yolo/split_by_scene.py`:

  ```text
  Split a YOLO dataset into scene-disjoint groups.

  A *scene* is one source recording. Frames within a scene are consecutive video frames
  of the same fight and are near-duplicates of each other, so any frame-level split leaks:
  the val half is then neighbouring frames of the train half, and val stops measuring
  generalization. This tool only ever moves whole scenes.

  Produces four groups::

      old        what a "deployed" model trained on
      new        footage that model has never seen
      hold_old   scene-disjoint probe of the old distribution (retention)
      hold_new   scene-disjoint probe of the new distribution (acquisition)

  Two ways to divide old from new:

  ``--mode temporal`` (default)
      Split on recording date at ``--cutoff``. This matches the operational question --
      new footage arrives from a *later* event, so it genuinely differs in cage, lighting
      and robot roster. Class balance across the halves is then whatever the calendar
      gives; it is reported, not corrected.

  ``--mode stratified``
      Balance the halves on frame count and per-class box counts, ignoring date. Cleaner
      numbers, but old and new end up the same distribution, so there is nothing unique
      for a new-data-only fine-tune to forget, and the retention probe measures little.

  The holdout carve inside each half is always stratified: a holdout is meant to be a
  faithful sample of its own half, not another distribution shift.

  Images and their stem-matched labels are hardlinked (no copy); ``.npy`` / ``labels.cache``
  files are ultralytics artifacts and are skipped, training regenerates them.

  Usage:
    python training/yolo/split_by_scene.py       --src training/data/nhrl_robots_bbox_real       --out training/data/scenesplit_2026-07-25       --mode temporal --cutoff 2025-11 --holdout-frac 0.2       --stratify-class robot,house_bot
  ```

- `training/yolo/split_yolo_dataset.py`:
  Split a flat images/ + labels/ pair into train/val/test directories.
  Note this is a **frame-level random** split. For anything graded on generalization, prefer ``split_by_scene.py``: consecutive frames of one recording are near-duplicates, so a random split puts near-copies of the training frames into val and val stops measuring anything useful.
- `training/yolo/validate_yolo_dataset.py`, `photo_for`:
  Return the cached (key, PhotoImage) pair for `index`.
  Reusing the PhotoImage is what keeps arrow-key navigation quiet: moving the selection only recolors borders, it does not re-upload every image to X.
- `training/yolo/validate_yolo_dataset.py`, `compose` (`above image.draft("RGB", box)`):
  draft() lets libjpeg decode at a reduced DCT scale: the single biggest win for grid paging, since full 1080p decodes dominate otherwise.
- `training/yolo/validate_yolo_integrity.py` (`above COORD_EPS = 1e-3`):
  Normalized coordinates should live in [0, 1]; allow a little slack for rounding.
- `training/yolo/validate_yolo_integrity.py`:

  ```text
  Check that a YOLO dataset is internally consistent and safe to train on.

  Runs a batch of read-only integrity checks and prints a report:

  - data.yml fields valid for YOLO: names present and non-empty, nc (if given) matches
    len(names), kpt_shape/flip_idx well formed for pose sets. Extra fields (colors,
    source_mcap, path, ...) are ignored.
  - Every annotation's class id is in range 0..len(names)-1 (the core check).
  - Every row is a well formed detect box, pose row, or seg polygon, with normalized
    coordinates in [0, 1] and positive box sizes.
  - Image/label pairing: labels that reference a missing image are flagged; images with
    no label file are counted as background negatives.
  - Health signals: per-class instance counts, classes with zero instances, duplicate
    class names, empty label files.

  Layouts supported: split dirs (train/ val/ test/, each with images/ + labels/) or a
  single flat images/ + labels/ pair at the dataset root.

  Exit code is 0 when no errors are found, 1 otherwise. With --strict, warnings also fail.

  Usage:
      python validate_yolo_integrity.py training/data/nhrl_seg/nhrl_robots
      python validate_yolo_integrity.py training/data/synthetic --check-images --strict
  ```

