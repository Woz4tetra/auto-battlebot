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

## Notes moved from code comments (2026-11-15)

- `training/deeplab/build_field_crop_dataset.py`, `stage_masks` (`above groups = [("", args.src)] if args.recursive else [(s, args.src / s / "images") fo...`):
  Recursive mode walks an eval root of per-recording directories and keys each frame by its stem, which is the capture timestamp and unique across recordings. The split mode keys by "<split>/<filename>", which is what the crop stage reads.
- `training/deeplab/build_field_crop_dataset.py` (`above MIN_VISIBLE_AREA = 0.25`):
  A clipped box keeping less than this share of its original area is dropped rather than taught as a sliver: a robot mostly outside the crop is not what the box was annotating.
- `training/deeplab/build_field_dataset.py` (`above YOUTUBE_ID_LEN = 11`):
  SegmentFlow names an export after the video it was cut from, and a YouTube id is 11 characters. Slices of one video differ only after that prefix, so the prefix is the video identity.
- `training/deeplab/build_field_dataset.py`:

  ```text
  Assemble a segmask tree into one train/val field-segmentation dataset.

  `merge_segmask_datasets.py` + `split_segmask_dataset.py` would also produce a merged dataset, but
  `split_segmask_dataset.py` shuffles at frame level. Every corpus here is video-derived at a fixed
  stride, so a frame-level shuffle puts near-identical neighbours on both sides of the split and the
  val score stops measuring generalisation. This tool splits at the level of the source video
  instead, and never splits one video across train and val.

  It also refuses to reshuffle a corpus that is already split. A source subtree holding its own
  `train/` and `val/` is treated as settled: its frames keep the side they are already on. That
  keeps a previously published split, and any anchor scores computed against it, still comparable
  after new material is folded in. Only the unsplit material gets an assignment, sized to match the
  held corpus's val fraction.

  Masks are assumed to already share one label space; the tool checks this rather than remapping,
  since a remap needs per-source class names that flat segmask exports do not carry.

  Usage:
      python training/deeplab/build_field_dataset.py         --src ~/Desktop/deeplab_field_2026-07-29         --out ~/Desktop/deeplab_field_merged_2026-07-29
  ```

- `training/deeplab/build_field_manifest.py`, `verify_duplicates`:

  ```text
  Hash both sides of every duplicate pair and record what actually matches.

  A pair is un-paired only when the *images* differ, because the leak this exists to
  remove is the same pixels appearing on both sides of the split, a pair that shares
  an image but disagrees on the mask is still one frame, still leaks, and is kept
  paired with `mask_differs` set so the disagreement stays auditable. Their mask IoU is
  measured too: two labelings of one image bound the label noise in this corpus, which
  is what boundary F1 has to be read against.

  Mutates the manifest's records in place and returns the verification report.
  ```

- `training/deeplab/build_field_manifest.py`, `summarize` (`above augment_splits: dict[str, set[str]] = defaultdict(set)`):
  The split was drawn per-file, so both kinds of copy leak across it. Counted separately because they are separate defects: the augment figure stands even if the duplicate claim were to fall over.
- `training/deeplab/build_field_manifest.py`, `pair_duplicates` (`above original = min(originals, key=lambda r: r["path"])`):
  More than one original would make "the twin" ambiguous; take the first in sorted order so the manifest is reproducible, and let the summary report it.
- `training/deeplab/build_field_manifest.py`:

  ```text
  Inventory the floor-mask corpus: field type, scene, and provenance per frame.

  The corpus is a flat pile of `<name>.jpg` + `<name>_mask.png` pairs under train/ val/
  test/, and the filename is the only provenance. This script reads that filename for
  every frame and writes a manifest recording:

      field / scene   the arena and the recording (see field_labels.py)
      source          the labelled frame identity; offline `_augment-N` copies share it
      duplicate_of    the unprefixed twin of a `floor_dataset__`-prefixed file

  Nothing is deleted or moved, the only write is the manifest. `make_field_splits.py`
  consumes it to build the clean, scene-disjoint corpus.

  Duplicate pairing is by name. `--verify-duplicates` re-reads the manifest and hashes
  both sides of every pair, so the claim is checked at full scale before
  `make_field_splits.py` acts on it; a pair whose bytes differ is un-paired and flagged
  rather than dropped.

  Usage:
      python3 training/deeplab/build_field_manifest.py training/data/floor_mask_dataset         --out training/data/deeplab_field_2026-07-28/manifest.json

      python3 training/deeplab/build_field_manifest.py training/data/floor_mask_dataset         --manifest training/data/deeplab_field_2026-07-28/manifest.json         --verify-duplicates --hash md5
  ```

- `training/deeplab/convert_to_tensorrt.py`:

  ```text
  Convert a DeepLab .pth checkpoint (or .onnx model) to TensorRT engine format.

  Accepts either a .pth checkpoint (which is first exported to ONNX internally)
  or a pre-exported .onnx model (which skips straight to the TensorRT build).

  Engines are GPU- and TensorRT-version specific: an engine built on x86 cannot
  run on Jetson (or vice versa). For Jetson deployment, copy the .pth/.onnx model
  to the Jetson and run this script there to produce the .engine used by the C++
  app. Same TensorRT version (or VERSION_COMPATIBLE build) helps across patch
  versions on the same platform.

  Output filenames include a platform tag (e.g. _x86_64_sm89, _aarch64_sm72) that
  encodes both CPU architecture and GPU compute capability so incompatible engines
  are not loaded by accident.
  ```

- `training/deeplab/make_field_splits.py`, `mask_for`:
  The mask to use for a frame: the newer annotation when the image was labelled twice.
  `newer_mask` is set by build_field_manifest.py --verify-duplicates. Without it the record's own mask is the only one there is.
- `training/deeplab/make_field_splits.py`, `score` (`above return abs(held - target), len(subset)`):
  Fewer scenes breaks ties: a holdout concentrated in one recording is a cleaner statement about that recording than one smeared across many.
- `training/deeplab/make_field_splits.py` (`above MAX_SCENES_FOR_EXHAUSTIVE = 18`):
  Exhaustive subset search stays cheap below this; the largest field has 15 scenes.
- `training/deeplab/make_field_splits.py`:

  ```text
  Build the clean DeepLab field corpus: deduplicated, scene-disjoint, field-stratified.

  Reads the manifest from `build_field_manifest.py` and materializes a corpus whose
  train/eval line no scene crosses, so val IoU measures generalization rather than
  partial memorization. Also writes the Phase-B leave-one-field-out arms.

  Where one image was annotated twice, the **newer** mask wins: the `floor_dataset__` twin
  is a re-label, and carrying the older annotation would train and score against a
  superseded one. Masks are renamed after their image, since the surviving mask may come
  from the twin.

  `--materialize copy` (the default) writes real files so the corpus stands alone and the
  tree it was built from can be archived. Arms stay plain image lists either way, 18 arms
  of physical copies would be ~350 GB against 282 GB free.

  Layout written under --out:

      train/, val/        image + mask pairs (semantic_train.py's default layout)
      eval -> val         the name score_masks.py is pointed at
      arms/full.txt       every train image; the Phase-A arm
      arms/lofo_<probe>_n<N>.txt
                          probe field held out entirely (n0), then added back at
                          N frames drawn scene-spread. Rungs are nested, asserted here.
      splits.json         which scenes went where, and why
      field_index.json    image name -> field / scene / split, read by semantic_train.py
                          for macro-average model selection and by score_masks.py

  Single-scene fields cannot be held out scene-disjointly. They stay wholly in train and
  are reported as having no eval representation rather than silently breaking the rule.

  Usage:
      python3 training/deeplab/make_field_splits.py         --manifest training/data/deeplab_field_2026-07-28/manifest.json         --out training/data/deeplab_field_2026-07-28         --val-frac 0.15 --drop-augment-copies --drop-duplicates --seed 4176
  ```

- `training/deeplab/score_masks.py`, `frame_stats`:
  Per-frame sufficient statistics: (intersection, union, {tol: boundary counts}).
  Kept as counts rather than ratios so the bootstrap can resample frames and re-sum instead of re-running inference on every resample.
- `training/deeplab/semantic_train.py`, `load_field_index`:
  Image name -> field type, written by make_field_splits.py.
  Absent, training still works but model selection falls back to the global mean IoU, which on this corpus is roughly a Cage-6 score.
- `training/deeplab/validate_segmask_dataset.py`, `paint_cell`:
  Push `index` into `cell`, touching only the widget options that changed.
  Every config() call repaints, so a blind rewrite of all nine cells on every keystroke is what the flicker was.
- `training/deeplab/validate_segmask_dataset.py`, `rebuild_grid` (`above caption = tk.Label(`):
  width=1 char keeps a long filename from widening the cell
- `training/deeplab/validate_segmask_dataset.py`, `rebuild_grid` (`above image_label = tk.Label(frame, bd=0, bg="#000000", width=width, height=height)`):
  Fixed pixel size: the label must not resize itself around whatever image it is handed, or every cell reflows as a page loads.
- `training/deeplab/validate_segmask_dataset.py`, `photo_for`:
  Return (cache key, PhotoImage) for `index`, reusing the last upload.
  Reusing the PhotoImage is what keeps arrow-key navigation quiet: moving the selection only recolors borders, it does not re-upload nine images to X.
- `training/deeplab/validate_segmask_dataset.py`, `render_thumb`:
  Return a cached thumbnail for `index`, rendering it if needed.
  Every thumbnail is letterboxed to the identical `thumb_box()` size. Cells then never change shape when a new image lands in them, which is what made the grid jump around while a page was loading.
- `training/deeplab/validate_segmask_dataset.py`, `compose_off_main_thread` (`above image.draft("RGB", box)`):
  draft() lets libjpeg decode at a reduced DCT scale: the single biggest win for grid paging, since full 1080p decodes dominate otherwise.
- `training/deeplab/validate_segmask_dataset.py`, `__init__` (`above self.thumb_cache: "OrderedDict[Tuple, Image.Image]" = OrderedDict()`):
  Thumbnails are decoded off the main thread and cached, so paging stays snappy. The PhotoImage cache is separate because those must be built on the main thread; keeping them lets a redraw skip cells whose picture has not changed.
- `training/deeplab/validate_segmask_dataset.py`:

  ```text
  Segmentation Mask Dataset Validation UI

  This script provides a GUI for manually validating segmentation mask dataset
  annotations. Images are shown as a page of thumbnails with their mask
  overlays; arrow keys move the selection, y/n pass or fail it, and t/b judge
  every image on screen at once. Press Z to zoom the selected frame.
  The validation state is saved and can be resumed later.

  Mask convention: each image foo.jpg has a corresponding foo_mask.png where
  pixel values are integer class IDs (same format used by semantic_train.py).
  ```

