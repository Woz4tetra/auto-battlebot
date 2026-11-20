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
