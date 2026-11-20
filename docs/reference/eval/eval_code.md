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

## Confidence threshold sweep

`training/model_eval/conf_sweep.py` finds the confidence threshold whose detection count matches what a dataset should hold. A pre-label run at a fixed confidence over-detects on purpose, so the box count says as much about the threshold as about the model. The script runs each candidate once at a low floor, keeps every detection's confidence, and sweeps thresholds offline. One inference pass answers "what threshold would have produced the expected count" for every threshold at once.

- The expectation is a per-frame box rate measured from a labelled set. For NHRL cage footage it is 2.81 (see `docs/experiments/perception/2026-11-15_model_eval_notes.md`).
- Matching the count is necessary, not sufficient. An arm can hit the right count with the boxes in the wrong places, and a class the footage never contains (Mr Stabs in Mrs Buff's matches) is a false positive at every threshold. The per-class table is printed for that reason.
- Detections are cached per candidate under `--cache`, so re-sweeping against a different expectation costs nothing.

## Field size in camera transforms

`export_sub_dataset` in `training/model_eval/export_camera_transforms.py` writes the extents the RANSAC plane fit measured at that field init, centred on the field origin. They locate the arena edges, which the transform alone does not. Estimates vary per init and some are plainly bad fits, so the value is reported as measured and not snapped to a nominal arena size.

## Field projection check

`training/model_eval/make_field_projection_check.py` draws the arena floor onto eval frames so the exported `tf_field_from_camera` can be checked by eye. Floor distances and range bands downstream are only as good as that transform.

- The nominal 8 ft arena square and a 0.5 m grid are drawn on the floor plane. The square should sit on the cage walls. `field_size_m` from the RANSAC fit is not used, since it reads undersized and sometimes fails outright.
- Range rings at 1, 2 and 3 m from the camera's nadir should look like concentric ellipses hugging the floor.
- Each labelled robot's keypoint midpoint is round-tripped pixel to floor to pixel. The marker must land back on the original, and the printed residual says by how much. This catches a broken transform even when the grid looks plausible.

## Robot capture mosaic

`training/model_eval/make_robot_capture_mosaic.py` builds a mosaic of the sharpest real captures of each robot in an eval dataset. It supports the Meshy fidelity grades (`meshy_grade_*.md`), which ask whether a Meshy mesh built from a stale thumbnail still looks like the robot the camera sees at fight time. One row per robot, the N best crops from its ground-truth boxes.

- Our robots are read from their own GT label. The eval GT labels every opponent as a generic `opponent`, so each opponent is named by the recording it fought in (`--opponent name=recording_prefix`), the same map the grade uses.
- "Best" is the variance of the Laplacian over the crop after it is resized to a fixed edge. It prefers sharp, large boxes and penalizes motion blur and small robots alike, since a small crop is upsampled and reads as soft.
- Boxes touching the frame edge are skipped because they are partly out of view.
- Picks from one recording stay at least `--min-gap` seconds apart. Otherwise a robot that sits still for ten seconds fills its row with the same view.

## Mask centroid vs box center

`training/model_eval/mask_centroid_vs_box.py` measures how far a YOLO-seg mask centroid sits from its own box center. The pipeline consumes only a position, so the question is whether the centroid carries position information the box center does not.

1. Per detection, the offset between mask centroid and box center, in pixels and as a fraction of the box's longer side (the normalization `score.py` uses for keypoint PCK).
2. Over IoU-matched detections, how far each position estimate lands from the ground-truth box center. A detect-only model can be passed as a third estimator, so the mask centroid, the seg box center and a bbox-only model are graded on one scale.

- Inference runs through ultralytics on the `.pt` weights, not the TensorRT engine, because `TrtYoloModel` discards the mask coefficients. Detection metrics from this path differ slightly from `score.py`'s engine numbers. The offsets are self-consistent because centroid and box come from the same forward pass.
- `floor_errors` repeats the errors in millimetres on the arena floor. Each estimate's pixel is projected onto the plane the robot sits on and the distance is taken between floor points. Perspective makes a pixel worth far more at the far wall than at the camera, so a single mm-per-pixel factor would misstate both ends.
- `range_band_table()` prints the aim-point error in millimetres per estimator, split into equal-count range bands. The bands are quantiles of each robot's ground range from the camera, not fixed distances, so every row carries the same weight instead of the far band holding a handful of robots.
- The GT box center is itself an estimator of the aim point. It is the label, with no detection error in it, so it is the best any box-based method can do.
- `keypoint_midpoints()` returns the (N, 2) front/back keypoint midpoint per GT box, NaN where either end is unlabeled. It is the middle of the chassis along its heading axis, which is not the center of the bounding box.

## Context vs appearance probes

`training/model_eval/interpret_context_vs_appearance.py` tests the claim that the synthetic+bbox opponent detector keys on context (arena, "not us, not house bot, on the floor") rather than the opponent's own appearance. It runs four probes on two models (`real_bbox` baseline and `mix_all`) over the independent eval, prints metrics and saves figures.

1. RISE occlusion saliency: whether the evidence for an opponent box lives inside the box or in the surrounding context. The metric is saliency concentration inside the box vs a context ring vs background, normalized by area.
2. Cut-paste context swap: the opponent crop on a neutral gray canvas (appearance only), on a different frame's real arena (appearance plus swapped context), and original. If the score collapses without arena context, the model needs context, not the robot's pixels.
3. Feature-space check: whether real-opponent and synthetic-opponent crops are separable in backbone-embedding space (domain gap), by linear-probe accuracy and centroid cosine distance per model, and whether `mix_all` pulls the two closer.
4. Grad-CAM: gradient saliency at the P3 neck layer for the opponent score at each GT box, which corroborates RISE with the inside-box mass fraction.

All probes define the target as the max opponent class score (channels object and robot) among head anchors whose center falls inside the target box.

## Recall by object size

`training/model_eval/recall_by_size.py` splits agnostic recall by ground-truth object size for two or more engines. `score.py` reports one recall over the whole eval set, which says whether an arm is better but not where it differs. INT8 quantization is expected to fail unevenly: a distant robot produces low-amplitude activations, and if a tensor's INT8 scale is set wide those activations round to zero before the head, while a close robot survives. That is a claim about the size distribution of the misses, which one recall number cannot confirm or refute.

- Boxes are binned by sqrt(area) in source pixels, the measure COCO's small/medium/large split uses (32 px and 96 px).
- Matching is the class-blind greedy IoU matcher from `score.py` at the same threshold, so the totals row reproduces that report's agnostic recall.

```bash
venv/bin/python training/model_eval/recall_by_size.py training/data/nhrl_keypoints_eval_test \
  --candidate B16=data/models/<fp16>.engine --candidate B8=data/models/<int8>.engine \
  --labels "opponent,house_bot" --taxonomy training/model_eval/taxonomy_merged.yaml \
  --conf 0.5 --baseline B16 --output <dir>
```

## Paired scoring scripts

`training/model_eval/score_size_ladder.sh` scores the yolo26 pose size ladder (`pose_model_size_ladder_plan_2026-09-24.md`) on the cage-high eval set in one paired bootstrap per run. Every size goes into one `score.py` call, so the bootstrap pairs them all against the baseline. `score_domain_mix.sh` cannot do that: it takes one model key and many arms, and the ladder script takes one arm and many model keys.

```bash
bash training/model_eval/score_size_ladder.sh 2026-09-24 n s m l x
```

- Engines are `data/models/yolo26<size>-pose_<arm>_<date>_last_<shape>_x86_64_<sm>.engine`, the name `run_domain_mix_arm.sh` gives them. `<shape>` is the rectangular export (`SHAPE=rect384x640`, the default) or absent for the square one (`SHAPE=square`).
- `EXTRA` adds candidates as `name=engine` pairs, which is how the grid's square anchors join the square run:

```bash
SHAPE=square RUNS=opponent,heading OUT=.../anchors \
EXTRA="swap_half=data/models/yolo26s-pose_swap_half_2026-09-13_last_x86_64_sm86.engine" \
    bash training/model_eval/score_size_ladder.sh 2026-09-24 s x
```

- The eval set is `training/data/cage_high_x50_conf044`: 636 hand-corrected broadcast frames over six NHRL Brettzone and three MassD recordings (`cage_high_eval_scoring_2026-09-18.md`). The domain renders were built to look like this footage, so it is the ladder's target domain. The `_cagehigh` arms trained on these exact frames and cannot be scored here. `d50000` shares none.
- The recordings sit two levels down, so `EVAL` points at the `d40000/` subdirectory.

Output goes under `OUT` (default `cage_high_x50_conf044/scores_size_ladder`, with a conf suffix when conf is not 0.5):

| Directory | Contents |
| --- | --- |
| `opponent/` | all 636 frames, `taxonomy_opponent.yaml`: recall for Q1 to Q3 |
| `heading/` | all 636 frames, `taxonomy_keypoint_ours.yaml`: heading error for Q2 to Q4 |
| `venue_nhrl/` | the 516 NHRL Brettzone frames, opponents only |
| `venue_massd/` | the 120 MassD frames, opponents only |
| `rec_<name>/` | one recording, which is one opponent, opponents only |

Environment: `ARM` (default `d50000`), `BASELINE` (default `s`), `SHAPE`, `SM` (default `sm86`, megamind, where the set lives), `CONF` (default 0.5), `EVAL` (a split root for the threshold read), `NHRL_RECORDINGS` and `MASSD_RECORDINGS` (globs under `EVAL` that select each venue's recordings, passed to `score.py --recordings`), `OUT`, `RUNS` (default all: opponent, heading, venues, recordings), `EXTRA`.

## Workarounds in `training/model_eval`

- `score_size_ladder.sh` scores bare recordings directly because every label file in an eval recording is a `pass` frame, so a bare recording scores the same frames the gated root does.
