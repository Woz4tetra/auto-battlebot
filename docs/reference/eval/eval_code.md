# Eval code

How `auto_battlebot/eval/` and the eval dataset scripts work. The procedure for adding a recording is `docs/guides/adding_eval_recordings.md`.

## Evaluation code notes

Notes on `auto_battlebot/eval/` that matter when you add a recording or a candidate.

### `eval/dataset.py`

#### `FrameKey`

Which frame this is: the subdataset it came from, and its stamp.

Keying by stamp alone loses frames whenever two subdatasets number a frame the same.
That is the normal case for a set built from video, where every recording starts at
stamp 0: cage_high_x50_conf044 loaded 571 of its 636 frames, the nine recordings'
frame 0 collapsing into one. Sets sampled from SVO recordings carry stamps unique
across recordings and were never affected.

`dataset` is the subdataset's path relative to the GT root, "." for a single dataset.
`stamp_ns` stays available for the consumers that have to align GT against something
else timestamped, such as compare_cpp_python.py against a C++ MCAP.

#### `Frame`

GT boxes and candidate predictions as (N, 4) xyxy arrays. `gt_keypoints` and `pred_keypoints`
hold one (K, 3) array per box: x, y in pixels plus visibility (GT) or confidence (prediction).
Box-only rows use empty arrays.

#### `reviewed_frames`

`validation_state.json` is the authority: it maps each image path, relative to the root, to a
validation verdict, and only `pass` frames are scored. `.edit_state.json` is the older
`edit_labels.py` state and is consulted only when no validation state exists. Preferring the
older file silently drops whole recordings: on `nhrl_keypoints_eval_test` the edit state lists
429 frames and names none of the 98 MassD ones, all of which validate as `pass`.

The subdataset travels with the stem for the same reason `FrameKey` carries it: two recordings
can name a frame the same, and a verdict on one must not select or reject the other. Both files
live at the dir that was pointed at, so a symlink root with neither scores every label file
present.

#### `load_gt`

Returns a `LoadedGt` of `frames` ({FrameKey: (boxes, labels, keypoints)}), `names`, and
`images` ({FrameKey: image path}). Subdatasets may trail off early (a recording without some
class), but class indices must agree.

### `eval/detectors.py`

#### `FieldCropDetector`

Crops each frame to its field box before inference, then maps detections back.

Arm E of `input_geometry_2026-09-05` trains on field-cropped images, so it has to be
evaluated on field-cropped frames. Cropping here rather than pre-cropping the eval set
leaves the GT in full-frame coordinates, so every candidate is scored against the same
boxes and the paired bootstrap stays paired.

Boxes come from a JSON keyed by frame stem, as written by
`training/deeplab/build_field_crop_dataset.py masks --recursive`. A frame with no field
detected passes through uncropped, matching what the dataset builder does.

#### `PrecomputedDetector`

Detections read from a JSON file instead of produced by an engine.

Some detectors cannot be handed one image at a time. Background subtraction needs the
whole recording to build its background, so it is run separately by
`background_subtraction_predict.py` and its output is replayed here, which keeps every
metric, threshold and plot identical to an engine's.

Format: {"labels": [...], "frames": {"<stamp_ns>": [{"xyxy": [x1, y1, x2, y2],
"score": float, "class_id": int, "kps": [[x, y, v], ...]}]}}.

The replay file is keyed by stamp alone, so a set whose recordings share stamps cannot be
replayed against. Only single-recording runs write these, and `detect` raises for a key from a
subdataset.

#### `build_detector`

The detector for one candidate, with any preprocessing that candidate was trained on.

Geometry arms differ in how the frame reaches the tensor, and an engine fed the wrong
preprocessing fails quietly rather than loudly: a stretch-trained engine handed a
letterboxed frame simply sees the wrong aspect ratio. Both modes are opt-in per
candidate so a mixed run scores each arm the way it was trained.

### `eval/metrics.py`

Detection and keypoint metrics over matched GT/prediction frames.

Detection metrics are frame-decomposable (recall, precision, F1, localization recall) so
the paired bootstrap in `significance.py` can resample them. mAP is dataset-level and
stays a point estimate. Keypoint metrics are level-independent.

#### `match_indices`

Greedy class-blind IoU matching by descending confidence. Returns `MatchPair(gt, pred)` rows:
unmatched GT as `(g, None)`, unmatched predictions as `(None, p)`.

#### `keypoint_metrics_from_stats`

Sample sizes behind the ratios above. On an eval set where only a few classes carry
keypoints these are small enough to decide whether a delta is worth believing, so
they are reported next to the metric rather than left in the stats dict. Excluded
from KEYPOINT_METRICS: they are counts, not scores, and are not bootstrapped.

### `eval/significance.py`

`_resample_ratio` takes per-frame numerator and denominator arrays and a (B, n_frames) index matrix
of B bootstrap resamples. It is fully vectorized.

## Eval dataset scripts

### `playground/depth_cache/cache_gt_depth.py`

`cache_gt_depth.py`:

Extract ZED depth for every ground-truth frame in an eval dataset, into a cache.

Depth is not recorded in the MCAP: the ZED SDK computes it on device and the pipeline never publishes it. The stereo pair is in the SVO though, so replaying the SVO through pyzed with the same DEPTH_MODE the runtime uses reproduces what the camera would have measured.

The join is by SVO frame index, never by timestamp. Each GT frame's `camera_transforms/<stamp_ns>.json` carries `image_stream_index`, which is the absolute index into the SVO. SVO image stamps sit about half a frame before the pipeline stamps that name the GT images, so a nearest-timestamp match silently picks the wrong frame.

Frames whose transform method is `ambiguous` or `unavailable` are skipped: the first has two processed frames claiming one index and the second has no pose at all, so neither can be placed in the field frame later.

Output is one `<subdataset>.npz` per sub dataset, keyed by stamp_ns, holding float16 depth in metres at the SVO's native resolution. Non-finite depth (no stereo match) is preserved as NaN, because which pixels the camera failed to measure is the result this experiment turns on.

```
Usage:
    python playground/cache_gt_depth.py training/data/nhrl_keypoints_eval_test         --svo-roots data/svo -o <cache dir>
```

### `perception/camera_geometry.py`

Projects eval-dataset pixels onto the arena floor from the geometry that `export_camera_transforms.py` writes (`camera_info.json` and `camera_transforms/<stamp>.json` per sub dataset). A pixel becomes a camera ray, the ray rotates into the field frame, and it intersects the horizontal plane the target sits on. The runtime does the same ray-plane projection in `project_keypoint_onto_plane` (`src/transform_utils.cpp`), so distances computed here are in the units navigation consumes.

Plane heights come from `[robot_filter] keypoint_height_meters*` in `config/_common.toml`: our robots' keypoints are labeled on the floor, house bots ride 0.12 m up, everything else takes the 0.03 m default. Projecting a robot onto the wrong plane biases its range. `tests/python/test_camera_geometry.py` pins the table against the TOML.

Only `exact` and `interpolated` transforms are usable. `unavailable` has no pose at all and `ambiguous` has two frames claiming the index, so both are dropped by default.
