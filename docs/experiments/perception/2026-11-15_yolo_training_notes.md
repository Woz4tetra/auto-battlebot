# 2026-11-15 yolo training notes

Notes moved out of code comments during the comment purge. Each entry names the file and symbol it came from.

## Notes moved from code comments (2026-11-15)

- `training/yolo/clear_image_cache.py`:

  ```text
  Delete ultralytics ``cache="disk"`` image caches, keeping recently-used ones.

  Training with ``cache="disk"`` writes one ``.npy`` per image next to it. These are pure
  derived data, ultralytics regenerates them on the next run, but they dwarf the images
  themselves: on megamind 297 GB of ``.npy`` sat next to 24 GB of actual unique data, and
  filled the NVMe to 97 %.

  Deleting them is safe but not free: the next training run on that dataset re-caches, which
  costs a few minutes of IO. So this tool skips caches that were *used* recently.

  **Age is measured from access time by default, not modification time.** A cache is written
  once and then read by every subsequent run, so mtime says "when was this dataset first
  trained on", a cache generated months ago and used daily looks ancient by mtime. atime
  answers the question that matters: when was this last actually read. The root filesystem is
  mounted ``relatime``, which updates atime on read once a day, so day-granularity ages are
  accurate. On a ``noatime`` mount atime is frozen and this tool says so rather than silently
  deleting a hot cache.

  Datasets referenced by a running process are never touched, whatever their age.

  Usage:
    # what would go, using the default 7-day floor
    python training/yolo/clear_image_cache.py --dry-run

    # actually delete caches unused for 14+ days
    python training/yolo/clear_image_cache.py --older-than 14

    # aggressive: everything not currently in use
    python training/yolo/clear_image_cache.py --older-than 0
  ```

- `training/yolo/remap_labels_by_name.py`, `passing_frames`:
  Image stems marked `pass` in the scene's validation_state.json, or None if untracked.
  These datasets carry a human review verdict per frame from validate_yolo_dataset.py. Ignoring it silently pulls in frames a person already rejected, 1,680 of them across the 56 archived scenes, 4.7 % of the corpus.
- `training/yolo/remap_labels_by_name.py`:

  ```text
  Merge per-scene YOLO datasets into one, remapping classes **by name**.

  `remap_labels.py` applies a single source-index -> target-index map. That is unusable when the
  scenes do not share a vocabulary, which is exactly the case for the archived
  `nhrl_seg_indiv_labeled`: 50 scenes declare 105 classes and 6 declare 111, and the indices are
  shifted between them (`Floor` 36 vs 39, `House Bot` 41 vs 44, `Mrs Buff MK2` 67 vs 70). Applying
  one index map there produces a silently mislabelled corpus, the same failure mode that left
  `Mrs Buff MK2` labelled as our own robot for months.

  This tool reads each scene's own `data.yml`, maps source class *names* to the target vocabulary,
  drops the names told to drop, and writes one flat dataset. Images are **copied**, not linked, so the
  output is standalone and survives the source being archived or moved to another filesystem.

  Names with no explicit rule fall through to the config's `default` target and are **all printed**,
  so a new non-robot class appearing upstream is visible rather than silently folded in.

  With ``--to-bbox`` the polygon geometry is converted to axis-aligned boxes in the same pass (reusing
  ``seg_to_bbox.polygon_to_bbox``), avoiding a full intermediate copy of the corpus.

  Usage:
    python training/yolo/remap_labels_by_name.py       --src /media/storage/auto-battlebots-archive/nhrl_seg_indiv_labeled       --config training/yolo/remap_config_3class.toml       --out training/data/nhrl_robots_bbox_3class --to-bbox
  ```

- `training/yolo/seg_to_bbox.py` (`TOP_CONTOURS = 3`):
  a robot's mask commonly splits into a few real parts; see top_contours_bbox
- `training/yolo/seg_to_bbox.py`, `split_subcontours`:

  ```text
  Split one label row's point list where it leaps to a disjoint contour.

  A single YOLO-seg row can hold several contours concatenated end to end, 23.6 % of sampled
  polygons in this corpus contain a step larger than 3 % of the frame, against a median step of
  0.21 % and p99 of 1.8 %. Handing the whole path to ``fillPoly`` fills the bridge between the
  blobs, which is how one Bee-Roll row became a 26 %-of-frame box.

  Over-splitting is safe: the pieces are rasterised into the same mask, so parts of a genuinely
  contiguous shape touch and merge back into one contour. Only truly disjoint blobs stay separate.
  ```

- `training/yolo/seg_to_bbox.py` (`SUBCONTOUR_JUMP = 0.03`):
  ~p99.7 of consecutive-point steps; see split_subcontours
- `training/yolo/seg_to_bbox.py`:

  ```text
  Convert a YOLO segmentation-polygon dataset into a detection (bbox-only) dataset.

  Preserves class ids and the train/val/test split structure exactly. Each polygon
  label row ``class x1 y1 x2 y2 ...`` becomes a box row ``class cx cy w h`` from the
  polygon's axis-aligned bounds. Rows already in box form (4 coords) pass through.
  Images are hardlinked (fast, no copy) when the output is on the same filesystem,
  otherwise copied.

  Built for the seg-vs-bbox controlled comparison: identical images, classes, and
  splits as the seg dataset, box labels only, so any box-AP difference is attributable
  to the seg-vs-detect head and nothing else.

  Usage:
    python training/yolo/seg_to_bbox.py training/data/nhrl_seg/nhrl_robots       -o training/data/nhrl_robots_bbox
  ```

- `training/yolo/train.py`, `reclaim_stale_caches`:

  ```text
  Delete ultralytics disk caches unused for ``max_age_days``, before training starts.

  ``cache="disk"`` writes one ``.npy`` per image and never prunes them: on megamind a single
  corpus grew to 196 GB of cache against 12 GB of images, and a stale set from an abandoned
  dataset can fill the NVMe and stall an unrelated run. Deleting is safe, ultralytics rebuilds
  what it needs, so the only cost of being wrong is re-caching time.

  The dataset about to be trained on is protected explicitly. ``busy_datasets`` skips this process
  and its ancestors (so that a shell mentioning a path does not pin it), which means it cannot see
  that *we* are the ones about to use this data.

  Never fatal: a failure here must not take down a multi-hour training run.
  ```

