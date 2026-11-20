# YOLO training data tools

Design facts for the dataset and cache tools under `training/yolo/`.

## Disk image caches

Training with `cache="disk"` writes one `.npy` per image next to it. Ultralytics regenerates them and never prunes them, but they dwarf the images. On megamind, 297 GB of `.npy` sat next to 24 GB of unique data and filled the NVMe to 97 %. One corpus grew to 196 GB of cache against 12 GB of images.

Deleting a cache is safe. The only cost of a wrong delete is a few minutes of IO to re-cache on the next run. So both tools skip caches that were used recently.

### `clear_image_cache.py`

- Age is measured from access time by default, not modification time. A cache is written once and read by every later run, so mtime says when the dataset was first trained on. A cache generated months ago and used daily looks ancient by mtime. atime says when it was last read.
- The root filesystem is mounted `relatime`, which updates atime on read once a day, so day-granularity ages are accurate. On a `noatime` mount atime is frozen, and the tool says so instead of deleting a hot cache.
- Datasets referenced by a running process are never touched, whatever their age.

```bash
python training/yolo/clear_image_cache.py --dry-run          # default 7-day floor
python training/yolo/clear_image_cache.py --older-than 14    # delete caches unused 14+ days
python training/yolo/clear_image_cache.py --older-than 0     # everything not in use
```

### `reclaim_stale_caches()` in `training/yolo/train.py`

Runs before training starts and deletes caches unused for `max_age_days`.

- The dataset about to be trained on is protected explicitly. `busy_datasets` skips this process and its ancestors, so a shell that merely mentions a path does not pin it. That also means it cannot see that this process is about to use the data.
- It is never fatal. A failure here must not take down a multi-hour training run.

## Merging scenes by class name

`training/yolo/remap_labels_by_name.py` merges per-scene YOLO datasets into one flat dataset, remapping classes by name.

`remap_labels.py` applies one source-index to target-index map. That fails when scenes do not share a vocabulary, as in the archived `nhrl_seg_indiv_labeled`: 50 scenes declare 105 classes and 6 declare 111, with shifted indices (`Floor` 36 vs 39, `House Bot` 41 vs 44, `Mrs Buff MK2` 67 vs 70). One index map there produces a silently mislabelled corpus, the failure that left `Mrs Buff MK2` labelled as our own robot for months.

- It reads each scene's own `data.yml`, maps source class names to the target vocabulary, drops the names told to drop, and writes one flat dataset.
- Images are copied, not linked, so the output survives the source being archived or moved to another filesystem.
- A name with no explicit rule falls through to the config's `default` target and is printed, so a new non-robot class upstream is visible.
- `--to-bbox` converts polygons to axis-aligned boxes in the same pass with `seg_to_bbox.polygon_to_bbox`, which avoids a full intermediate copy of the corpus.

```bash
python training/yolo/remap_labels_by_name.py \
    --src /media/storage/auto-battlebots-archive/nhrl_seg_indiv_labeled \
    --config training/yolo/remap_config_3class.toml \
    --out training/data/nhrl_robots_bbox_3class --to-bbox
```

### Review verdicts

`passing_frames()` returns the image stems marked `pass` in a scene's `validation_state.json`, or `None` for an untracked scene. These datasets carry a per-frame human verdict from `validate_yolo_dataset.py`. Ignoring it pulls in frames a person already rejected: 1,680 of them across the 56 archived scenes, 4.7 % of the corpus.

## Segmentation to bbox conversion

`training/yolo/seg_to_bbox.py` converts a YOLO segmentation dataset into a detection dataset.

- Class ids and the train/val/test split structure are preserved exactly.
- A polygon row `class x1 y1 x2 y2 ...` becomes a box row `class cx cy w h` from the polygon's axis-aligned bounds. Rows already in box form (4 coordinates) pass through.
- Images are hardlinked when the output is on the same filesystem, otherwise copied.
- It was built for the seg-vs-bbox comparison: identical images, classes, and splits, box labels only, so any box-AP difference comes from the head and nothing else.

```bash
python training/yolo/seg_to_bbox.py training/data/nhrl_seg/nhrl_robots -o training/data/nhrl_robots_bbox
```

### Multi-contour rows

`split_subcontours()` splits one label row's point list where it jumps to a disjoint contour. A single YOLO-seg row can hold several contours concatenated end to end. In this corpus 23.6 % of sampled polygons contain a step larger than 3 % of the frame, against a median step of 0.21 % and a p99 of 1.8 %. Handing the whole path to `fillPoly` fills the bridge between blobs, which turned one Bee-Roll row into a box covering 26 % of the frame.

- `SUBCONTOUR_JUMP = 0.03` is about the p99.7 of consecutive-point steps.
- Over-splitting is safe. The pieces rasterise into the same mask, so parts of a contiguous shape touch and merge back into one contour. Only truly disjoint blobs stay separate.
- `TOP_CONTOURS = 3` in `top_contours_bbox()` keeps the largest parts, because a robot's mask commonly splits into a few real parts.

## Scaling split construction

`training/yolo/make_scaling_splits.py` builds the data-scaling arms.

- `choose_val_scenes()` picks `count` whole scenes whose frame share is near `frac` and whose house_bot share matches the target. It searches random whole-scene combinations instead of taking scenes in order. Scene sizes are so uneven that any fixed rule lands far off the target, and the random search gets within a fraction of a percent on both axes.
- `summarize()` reports frames, scenes and the scene list. Class counts are per scene, so they are exact only for whole-scene arms. Frame counts are reported either way.

## Pooling real and synthetic data

`training/yolo/pool_datasets.py` pools a full real dataset with a fraction of a synthetic dataset into one flat YOLO tree. It was written for the synthetic scene-variation probe of the data_epoch_min experiment (Recipe C in `docs/experiments/perception/experiment_runbook.md`).

- Every real frame is included. Synthetic frames are subsampled by `--synth-fraction`, taken either in filename order or at random (`--synth-order sequential` or `random`).
- Synthetic filenames carry a timestamp and frame index (`...T17-01-18-000050...`), so filename order is render-sequence order. `sequential` is scene-correlated and `random` spreads scene coverage. The gap between the two accuracy-vs-fraction curves at a fixed real count says how much synthetic scene variation the model needs.
- Only the image and its stem-matched `.txt` label are linked. `.npy` and `labels.cache` files next to the images are ultralytics `cache="disk"` artifacts and are skipped. Training regenerates them.
- The output is a flat `images/` and `labels/` tree for `split_yolo_dataset.py`, then `validate_yolo_integrity.py --strict`.
- Pool into the train split only. Build a real-only val split separately and grade on the external eval, never on same-corpus val.

```bash
python pool_datasets.py --real ../data/real_our_robots --synth ../data/all_robot_keypoints/train \
  --out ../data/phaseB_r100_s25_seq --synth-fraction 0.25 --synth-order sequential
```

## Detect-head training defaults

The `yolo26n` entry in `training/yolo/train.py` sets `imgsz` 640, the same as `yolo26n-seg`, so a seg-vs-bbox box-quality comparison varies the head type and not the input size. `batch` differs (128 vs 32), so override it back for a strict comparison.

- Epochs are 100, not 500. Every arm in `data_scaling_2026-07-27.md` peaked on val between epoch 63 and 90 on `nhrl_robots_bbox_2class`, and none was still rising at 100. `category_addition_2026-07-25.md` found fully annealed 150-epoch endpoints failed the eval parity gate. A 500-epoch schedule anneals far past the point where this data generalizes.
- Cache is `ram`, not `disk`. The disk cache writes one full-resolution `.npy` per image, about 200 GB against 12 GB of JPEGs on this corpus (the blowup described under `reclaim_stale_caches()`). The resized RAM cache is about 17 GB per DDP process.

## Input geometry dataset builders

Two scripts build the training corpora for the input-resolution arms (`docs/experiments/perception/input_geometry_2026-09-05.md`), plus a gate that decides whether the crop arm is worth building.

### Stretched copy (`training/yolo/make_stretched_dataset.py`)

Builds an anisotropically resized copy of a YOLO dataset, for the stretch arm (arm D), which fills the input tensor by ignoring aspect ratio instead of padding.

- Squeezing 16:9 into a square costs 0.5x horizontally and 0.89x vertically. A robot lands at sqrt(0.5 * 0.89) = 0.67 of its source size against 0.50 for a letterbox, which is 1.33x more robot at the same 409,600 tensor pixels and no padding.
- Normalized YOLO labels need no change, because `cx cy w h` are fractions of width and height. The label files are symlinks. Only images are rewritten.
- Train at `imgsz=640` with `rect=False`. The ultralytics letterbox is a no-op on already square images.

```bash
python training/yolo/make_stretched_dataset.py \
  training/data/nhrl_robots_bbox_2class training/data/nhrl_robots_bbox_2class_stretch
```

### Field-cropped copy (`training/deeplab/build_field_crop_dataset.py`)

Builds the field-cropped corpus for the crop arm (arm E), which asks whether spending the input tensor on the field alone beats spending it on the whole frame. At deployment the field is already known (`field_filter.track_field` runs before perception and is computed once at init), so cropping to it costs arithmetic. The training corpus has no depth, so the field comes from the DeepLab RGB model.

- The `masks` stage runs DeepLab over every image and records the field bounding box in normalized frame coordinates, cached to JSON. It is GPU-bound, a few minutes over 32k images.
- The `crop` stage reads that JSON, crops each image to the box plus a margin, and rewrites the labels. It is CPU-bound, so the margin can be re-tuned without the GPU.
- A crop moves normalized YOLO coordinates, unlike the stretch arm. Every box is re-expressed against the cropped frame and clipped to it.
- `crop_one()` returns labels in, labels out, and frames dropped. With `drop_clipped` set, a frame whose crop discarded a label is skipped entirely. Keeping it would leave a robot visible with no box on it, which trains a false negative and is the one case where an unlabelled image is not a useful background negative. It costs 0.3% of frames at margin 0.20.

```bash
python training/deeplab/build_field_crop_dataset.py masks \
  --model ../../data/models/field_deeplabv3p_r50_2026-07-29.pth \
  --src ../data/nhrl_robots_bbox_2class --out field_boxes.json
python training/deeplab/build_field_crop_dataset.py crop \
  --src ../data/nhrl_robots_bbox_2class --dst ../data/nhrl_robots_bbox_2class_fieldcrop \
  --boxes field_boxes.json --margin 0.10
```

### Field fraction gate (`training/deeplab/field_fraction_gate.py`)

A go/no-go gate for the crop arm. Cropping closes the train/deploy domain gap only if the field fills a similar fraction of the frame in both corpora. NHRL overhead cage cameras are framed on the cage, so the field may already fill the frame there, while the ZED shoots from inside the cage and sees much less. If the fractions differ sharply, cropping teaches a scale the deployment camera never delivers.

- It runs the DeepLab field model over a sample of both corpora and reports, per corpus, the fraction of the frame covered by the field mask and by its bounding box (the box is what a crop uses).
- `--labels` reports how much margin the crop needs before it stops clipping annotated robots, and how much zoom survives that margin. A crop tight enough to zoom cuts robots in half. A crop loose enough to keep them covers the frame.
- `--tensors` reports what the crop is worth once letterboxed into a real engine input, against not cropping. `tensor_tradeoff()` computes zoom (the letterbox scale relative to arm A's 640x640) and padding per engine shape, cropped and uncropped. Source resolution cancels out of the zoom ratio for a 16:9 frame, so 1280x720 stands in for both corpora.
- A letterbox scale is set by whichever axis binds first, and the field box spans nearly the full frame width on both corpora. Cropping removes rows the width-bound scale already applied, so it buys no zoom at any tensor shape 16:9 or squarer. It only spends the freed rows on padding.
- Eval recordings have different framing, so the gate reports each recording separately. One outlier recording cannot decide it.

```bash
python training/deeplab/field_fraction_gate.py --model data/models/field_deeplabv3p_r50_2026-07-29.pth \
  --train-images training/data/nhrl_robots_bbox_2class/train/images \
  --eval-root training/data/nhrl_keypoints_eval_test --sample 300 --device cpu
```
