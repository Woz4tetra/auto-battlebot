# model_eval

Offline detector evaluation: hand-corrected YOLO ground truth scores any number of
candidate TensorRT engines. `score.py` runs each engine directly on the GT images with
the same preprocessing and NMS as the C++ pipeline, so no playback run or hardware is
needed to grade a model.

Two ways to build ground truth:

- `make_eval_dataset.py`: sample raw frames from fight MCAP recordings and label from
  scratch. This is how `training/data/nhrl_keypoints_eval_test` was built.
- `export_labels.py`: pre-label frames with the detections a `label_playback` run
  recorded (`/blob_detections`, `/keypoint_detections`), then correct the mistakes.
  Cheaper labeling when the current models are mostly right.

Either way, `edit_labels.py` is the editor, and only frames you mark reviewed count as
ground truth when scoring.

To add a new fight recording to `nhrl_keypoints_eval_test`, follow
[docs/guides/adding_eval_recordings.md](../../docs/guides/adding_eval_recordings.md). It covers the full
run, from merging the SVO through camera transforms and the push to megamind. The workflow
below is the shorter version for labeling and scoring an existing dataset.

## Workflow

1. Build a dataset to label. From scratch:

```bash
python training/model_eval/make_eval_dataset.py data/recordings/<fight>.mcap
```

   Or pre-labeled from a labeling run (keeps `/camera/image` and detection topics):

```bash
./scripts/build_and_run.sh -c config/experiments/label_playback.toml
python training/model_eval/export_labels.py data/recordings/<labeling_run>.mcap
```

Each recording produces one directory with shared images and (for pre-labels) a
subdataset per model:

```
training/data/model_eval/<recording>/
    images/                     # shared PNGs (real files)
    blob/     images -> ../images, labels/ (class cx cy w h), data.yaml
    keypoint/ images -> ../images, labels/ (class cx cy w h kx ky v ...), data.yaml
```

2. Correct or create the labels (move/resize/add/delete/reclass boxes, drag keypoints),
   and mark frames reviewed as you go:

```bash
python training/model_eval/edit_labels.py training/data/nhrl_keypoints_eval_test
```

Label at the finest granularity you care about (per robot instance). Edit `data.yaml` to
add class names the pre-labeler did not know.

3. Score. The GT argument is a dataset dir or a root of subdatasets; `--labels` maps
   engine class indices to GT label names (the C++ `label_indices` config, lowercased):

```bash
# blob model (YOLO-seg)
python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
    --candidate deployed=data/models/yolo26n-seg_nhrl_robots_2026-04-27_x86_64_sm89.engine \
    --labels opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3 \
    --conf 0.6 --taxonomy training/model_eval/taxonomy.yaml

# keypoint model (YOLO-pose), scored on our-robot boxes only
python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
    --candidate deployed=data/models/yolo26n-pose_our_robots_2026-05-01_x86_64_sm89.engine \
    --labels mr_stabs_mk2,mrs_buff_mk3 \
    --conf 0.5 --taxonomy training/model_eval/taxonomy_keypoint.yaml
```

Repeat `--candidate name=engine` to compare models; with two or more, a paired bootstrap
reports which metric deltas against the baseline (first candidate, or `--baseline NAME`)
are significant.

Match the C++ config when grading a deployed model: `--conf` is `confidence_threshold`
(blob deployed at 0.6, keypoint default 0.5), `--nms-iou` is `iou_threshold` (0.45
default in both).

## Output

- `summary.csv` + stdout table: mAP@.5, mAP@[.5:.95], precision/recall/F1,
  localization recall, wrong-class rate. One row per candidate per level.
  When the engine carries keypoints: `kp_err_px` (mean keypoint pixel error),
  `kp_pck@0.1` (fraction of keypoints within 10% of the GT box's longer side), and
  heading error/accuracy from the front->back keypoint vector, over IoU-matched boxes.
- `headline.png`: agnostic recall ("found the robot") vs archetype/instance mAP
  ("named it right") per candidate. The gap is the cost of splitting `OPPONENT`.
- `confusion_<candidate>_<level>.png`: right-box-wrong-class matrix at the IoU threshold.
- `significance.csv` (two or more candidates): paired-bootstrap deltas and verdicts.

## Camera geometry

`export_camera_transforms.py` adds camera pose and intrinsics to an eval dataset without
touching its images or labels:

```bash
python training/model_eval/export_camera_transforms.py --dry-run   # report coverage only
python training/model_eval/export_camera_transforms.py
```

Per sub dataset it writes `camera_info.json` (intrinsics, constant over the recording) and
`camera_transforms/<stamp_ns>.json` (one per image, carrying `tf_field_from_camera` plus the
two transforms it composes). Multiply a point in the camera frame by `tf_field_from_camera`
to get it in the field frame.

Check the `method` field before trusting a transform:

| method | meaning | count in `nhrl_keypoints_eval_test` |
|---|---|---|
| `exact` | one processed frame owns this image's frame index | 445 |
| `interpolated` | pipeline skipped the frame; slerp/lerp between neighbours | 146 |
| `ambiguous` | two processed frames claim the index (05-01 only) | 14 |
| `unavailable` | no field frame yet, or nothing to interpolate from | 17 |

Each record also carries `field_size_m`, the extents the RANSAC plane fit measured at that
frame's field init. Treat it as what the pipeline believed, not as arena ground truth: the
real arena is an 8 ft (2.44 m) square, and the fit reads undersized on most frames
(2.25 x 1.98 m on 10-06-02). Across inits it ranges 1.96-2.35 m in x and 1.43-2.24 m in y on
that recording alone, and other recordings contain outright failed fits (0.27 x 0.15 m,
0.56 x 3.13 m). No failed init covers a sampled image, but the estimate is noisy. The pose is
the trustworthy part: projecting a nominal 8 ft square through `tf_field_from_camera` lands on
the arena walls.

The pipeline ran at ~25 fps against 30 fps capture, which is why a fifth of the images were
never processed live. Frames are matched on SVO frame index, not timestamp: the SVO recorder
and the pipeline stamp the same frame about half a frame apart, so timestamps alone only give
a nearest-neighbour guess. Recordings made after this was found also carry
`/camera/frame_meta` (`svo_path` + `svo_frame_index`), which makes the join direct.

The 05-01 recording is the weak one. Its SVO dropped a third of its frames during recording,
so only 38 of its 100 images resolve exactly.

## Levels

- `agnostic`: all labels collapse to `robot`. Pure localization.
- `archetype`: labels map through `taxonomy.yaml` (`archetypes:`).
- `instance`: labels as-is.

## Notes

- Reviewed-frame filtering, in priority order at the GT dir you pass: `validation_state.json`
  (only frames whose verdict is `pass` are scored), else `.edit_state.json` (only frames in its
  `reviewed` array), else every label file counts. `validation_state.json` wins because
  `.edit_state.json` on `nhrl_keypoints_eval_test` is stale: it lists 429 frames and names none
  of the 98 MassD ones, so preferring it silently scored that recording as zero frames.
- `taxonomy_keypoint.yaml` excludes everything but our robots, since the pose model only
  detects them. The default `taxonomy.yaml` excludes `object` (no deployed model has an
  object class).
- The editor edits detect and pose rows; anything else (seg polygons) is preserved verbatim.
- New boxes drawn in a pose dataset inherit the keypoint count from existing boxes,
  centered in the box, ready to drag into place.
- In `export_labels.py` output, each subdataset's `images/` is a symlink to the
  recording's shared `images/`. Downstream tools follow the symlink.
- Scores land in `<gt>/scores` by default; override with `--output`.
- `compare_cpp_python.py` scores recorded C++ playback detections (MCAPs) side by side
  with direct Python inference on the same frames, plus a linear fit between the two
  paths' matched detections. Caution: desktop playback frames are affine-warped ~2-3%
  vs live Jetson frames (ZED rectification difference), so playback detections carry a
  systematic offset against live-labeled GT. See
  `docs/experiments/perception/baseline_2026-07-07.md`.
- Related: perception reliability over a full fight uses `perception_reliability.py`.
- `make_robot_capture_mosaic.py` renders the sharpest GT crops of each robot in an eval
  dataset, one row per robot, with opponents named by recording (`--opponent name=prefix`).
  Supporting figure for the Meshy fidelity grades.

## Notes moved from code comments (2026-11-15)

- `training/model_eval/benchmark_engines.py`, `main` (`above post_mean = means["total"] - means["pre"] - means["gpu"]`):
  Residual, not a timed loop: NMS/decode plus the coordinate rescale. Means are additive so this subtraction is valid; the same trick on medians would not be.
- `training/model_eval/benchmark_engines.py`:

  ```text
  Per-engine inference latency, reported the same way on the dev box and the Jetson.

  `trtexec` is not available everywhere, on megamind TensorRT comes from the pip wheel,
  which ships no binary, so this reuses `auto_battlebot.perception.trt_yolo.TrtYoloModel`, the same
  class `score.py` scores with. That also makes the numbers directly comparable to the
  accuracy run instead of measuring a differently-configured engine.

  Four levels are reported, because they answer different questions:

    pre    `preprocess_frame` only: letterbox resize and the NCHW float blob. Pure CPU.
    gpu    `_run` only: H2D copy, `execute_async_v3`, D2H copy, stream sync. Comparable to
           `trtexec` with data transfers.
    post   the residual, `total - pre - gpu`: NMS/decode and the coordinate rescale. Pure
           CPU, and in Python rather than the C++ the app runs, so this is the level that
           transfers worst. Derived from means, which are additive; medians are not.
    total  `infer`: pre + gpu + post end to end. This is what the C++ `update()` call
           actually costs, and it is the one the frame-period decision rests on --
           `docs/experiments/perception/parallel_yolo_batch/shared_preprocess_2026-07-24.md` found GPU
           inference is 1-3 ms while a full per-model update is ~12 ms on the Jetson, so
           quoting GPU time alone understates a bigger model by ~4x.

  The `pre`/`post` split matters on the Jetson specifically: its CPU is far slower relative
  to its GPU than the dev box's, so the CPU stages are a much larger share of the total and
  the Python-vs-C++ gap on those stages is what decides how much of a Python `total` the
  C++ app actually pays.

  Feed it a real eval frame (the default) rather than noise: NMS cost scales with the
  number of surviving boxes, and a noise image decodes to a degenerate detection count.
  ```

- `training/model_eval/compare_cpp_python.py`:

  ```text
  Compare the C++ and Python inference paths for one detector on the same GT frames.

  The C++ path is detections the perception stack recorded during playback
  (/blob_detections or /keypoint_detections in one or more MCAPs). The Python path is
  direct TensorRT inference on the GT images (auto_battlebot.perception.trt_yolo, as score.py uses).
  Both are scored with score.py's machinery on exactly the GT frames the recordings cover,
  so the two columns are frame-for-frame comparable.

  Beyond the metric table, matched C++/Python detection pairs are compared directly:
  box center offset, and a linear fit of C++ keypoint coordinates against Python's
  (slope != 1 means a scale error somewhere in the C++ path, intercept != 0 a translation).

  Usage:
      python training/model_eval/compare_cpp_python.py training/data/nhrl_keypoints_eval_test         --engine data/models/yolo26n-pose_our_robots_2026-05-01_x86_64_sm89.engine         --labels mr_stabs_mk2,mrs_buff_mk3 --topic keypoint --conf 0.5         --taxonomy training/model_eval/taxonomy_keypoint.yaml         --cpp-mcap 'data/recordings/auto_battlebot_experiments_eval_smoke_*.mcap'
  ```

- `training/model_eval/edit_labels.py`:

  ```text
  YOLO label editor for correcting model pre-labels into ground truth.

  Like training/yolo/validate_yolo_dataset.py, but edits annotations instead of
  pass/fail marking: move, resize, add, delete, and reclass boxes, and drag pose
  keypoints. Detect rows (class cx cy w h) and pose rows (class cx cy w h kx ky v ...)
  are editable; any other rows (seg) are preserved verbatim.

  Usage:
      python training/model_eval/edit_labels.py data/eval/<svo_name>

  Editing is click-move-click (no dragging needed): one click grabs a handle, move
  the mouse to position it, and a second click drops it. A press-move-release drag
  still finishes in one gesture, so both styles work.

  Controls:
      click empty space          start a new box (current class; inherits keypoint
                                 count from existing pose boxes, centered); move to
                                 size it, click to place
      click a box body           pick it up; move to reposition, click to drop
      click a corner handle      grab it; move to resize, click to release
      click a keypoint circle    pick it up; move to reposition, click to drop
      k                          keypoint mode on the selected box: each click places the
                                 next keypoint (shift+click marks it occluded); the mode
                                 ends once the box has a full set. Escape cancels
      1-9, 0                     set class of selected box / current draw class
      Delete or x                delete selected box
      a/d or left/right          previous / next image (auto-saves)
      space                      mark reviewed + jump to next unreviewed
      n                          jump to next unreviewed (without marking)
      +/- or ctrl+wheel          zoom in / out at the cursor; 0 (numpad) resets to fit
      ctrl+s                     save
  ```

- `training/model_eval/export_camera_transforms.py`, `export_sub_dataset` (`above "image_stream_index": position,`):
  Position in this mcap's /camera/image stream, used to match images against transforms. Not the .svo2 frame index: ZED_SVO_Editor drops a few frames per file, so the two differ by a small offset.
- `training/model_eval/export_camera_transforms.py` (`above MAX_INTERP_GAP_FRAMES = 8`):
  Interpolating across a long dropout says more about the gap than the frame. Beyond this many camera frames between the bracketing processed frames, report unavailable instead.
- `training/model_eval/export_labels.py`:

  ```text
  Export YOLO datasets with model pre-labels from label_playback MCAP recordings.

  Reads /camera/image, /blob_detections and /keypoint_detections from one or more MCAPs
  produced by `./scripts/build_and_run.sh -c config/label_playback.toml`, pairs them by
  frame stamp, and writes one dataset per model per recording. Images are written once at
  the recording top level and symlinked into each subdataset:

      <output-dir>/<recording>/images/<stamp_ns>.png       (shared, real files)
      <output-dir>/<recording>/blob/images   -> ../images  (symlink)
      <output-dir>/<recording>/blob/labels/<stamp_ns>.txt  (class cx cy w h)
      <output-dir>/<recording>/blob/data.yaml
      <output-dir>/<recording>/keypoint/images -> ../images
      <output-dir>/<recording>/keypoint/labels/<stamp_ns>.txt  (class cx cy w h kx ky v ...)
      <output-dir>/<recording>/keypoint/data.yaml

  Accepts multiple MCAP paths or globs and writes each under --output-dir automatically,
  so `export_labels.py data/recordings/*.mcap` processes a whole batch. Both the blob
  detector (detect rows) and keypoint model (pose rows) are exported by default; restrict
  with --topics.

  Correct the pre-labels with edit_labels.py, then score candidate runs with score.py.

  Usage:
      python training/model_eval/export_labels.py data/recordings/*.mcap         [--output-dir <dir>] [--topics blob keypoint]
  ```

- `training/model_eval/make_eval_dataset.py`:

  ```text
  Build a hand-labeling eval dataset from /camera/image frames in MCAP recordings.

  Unlike export_labels.py (which needs detection topics to write model pre-labels), this
  samples raw frames from recordings that carry no detections, writing empty label files
  for labeling from scratch in edit_labels.py. One subdataset per recording:

      <output-dir>/<recording>/images/<stamp_ns>.png   (left-eye frame)
      <output-dir>/<recording>/labels/<stamp_ns>.txt    (empty, ready to label)
      <output-dir>/<recording>/data.yaml                (names/colors/kpt_shape/flip_idx)

  Frames are named by the /camera/image header stamp_ns, which is the SVO frame stamp. That
  is how score.py aligns this ground truth to a later candidate playback run over the same
  SVO, so the dataset is scorable without any AVI/frame-index bookkeeping.

  Frames before each video's svo_start_frame (from the playback config) are skipped, then a
  fixed number are sampled evenly across the rest (decorrelated, not consecutive). The MCAP
  /camera/image stream is the SVO frames in order, so svo_start_frame applies as an index
  into that stream. Class names/colors and the keypoint schema are seeded from a source
  dataset (default: our_robot_keypoints) and can be extended with --extra-classes for robots
  absent from the training set (opponents, house).

  Usage:
      python training/model_eval/make_eval_dataset.py data/recordings/*.mcap         [--output-dir <dir>] [--per-video 100]         [--start-frames config/playback/svo_start_frames.toml]         [--extra-classes opponent house_bot]
  ```

- `training/model_eval/make_field_projection_check.py`, `pick_frames`:
  Frames with a usable pose, dealt round-robin across recordings.
  Round-robin rather than first-N so one fight cannot vouch for the whole export, and spaced within each recording so the tiles are not consecutive frames of one moment.
- `training/model_eval/predict_all_arms.sh` (`above set -euo pipefail`):

  ```text
  Label one dataset with every domain-mix arm, each arm into its own directory, for a side-by-side
  read of which arm labels the footage best.

  `prelabel_dataset.py` fills only empty label files, on purpose: a second pre-label round must not
  overwrite frames someone has corrected. Comparing arms wants the opposite, every frame labelled by
  every arm, so each arm gets its own copy of the dataset whose images are hardlinked and whose
  `labels/` starts out missing. The source dataset is never written to, and the hardlinks mean
  fifteen arms cost fifteen sets of label files rather than fifteen copies of the images.

    bash training/model_eval/predict_all_arms.sh training/data/nhrl_cage_high_eval \
        training/data/cage_high_arm_labels_2026-09-16 yolo26s-pose 2026-09-13 base d2500 swap_half

  Output: <out_root>/<arm>/<recording>/{images,labels,data.yaml}, each openable in edit_labels.py
  and validate_yolo_dataset.py. Each recording also carries `prelabel_holdout.json`, which records
  the model, conf and class map that wrote its labels.

  Environment: CONF (default 0.15, the pre-label floor: deleting a spurious box is one keypress,
  drawing a missed one is a dozen), CKPT (default last), DEVICE (default 0), IMGSZ (default 640),
  MAP (default "nhrl_robot=opponent", the arms' class for an opponent against this dataset's name),
  FORCE (set to rebuild an arm whose directory already exists).
  ```

- `training/model_eval/prelabel_dataset.py` (`above KEYPOINT_VISIBLE_CONF = 0.5`):
  A keypoint the model is at least this sure of is labelled visible (2), the rest labelled but not visible (1), matching how the eval set flags occluded keypoints.
- `training/model_eval/score.py`:

  ```text
  Score candidate TensorRT detector engines against a YOLO ground-truth dataset.

  Each candidate engine runs inference directly on the GT images (no playback, no stamp
  alignment), matching the C++ pipeline's preprocessing and NMS. Predictions are compared
  against the labels at three label levels:

      agnostic   every label collapses to "robot": pure localization (did it find the robot)
      archetype  labels map through taxonomy.yaml: NHRL archetype naming
      instance   labels as-is: per-robot-instance naming

  The gap between agnostic recall and class-aware mAP is the cost of splitting the
  OPPONENT category. Outputs a summary table (stdout + summary.csv) and plots.

  The GT argument is either a single dataset dir (data.yaml + images/ + labels/) or a root
  containing such subdatasets (e.g. training/data/nhrl_keypoints_eval_test). If a
  validation_state.json is present, only frames it marks `pass` are scored; failing that, an
  edit_labels.py .edit_state.json is used the same way.

  --labels maps engine class indices to GT label names, in class order (mirrors the C++
  label_indices config). When the engine head carries keypoints (YOLO-pose), keypoint
  metrics are added over IoU-matched boxes: mean pixel error, PCK@0.1 of the GT box's
  longer side, and heading error (angle of the front->back keypoint vector, mean degrees
  and accuracy within 10 degrees).

  With two or more candidates, a paired bootstrap reports whether each candidate differs
  significantly from the baseline (the first candidate, or --baseline NAME). Because every
  candidate is scored on the same GT frames, resampling frames paired cancels the
  "this frame is just hard" variance, so a few hundred frames can resolve real differences.
  mAP is dataset-level and left as a point estimate; the detection questions are covered by
  the recall / precision / localization tests, which are frame-decomposable.

  Usage:
      python training/model_eval/score.py training/data/nhrl_keypoints_eval_test         --candidate deployed=data/models/<model>_x86_64_sm89.engine         --labels opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3         [--taxonomy training/model_eval/taxonomy.yaml] [--conf 0.5] [--nms-iou 0.45]         [--iou 0.5] [--baseline NAME] [--bootstrap 1000] [--seed 0] [--alpha 0.05]         [--output <dir>]
  ```

- `training/model_eval/score_domain_mix.sh` (`above set -euo pipefail`):

  ```text
  Score domain-mix arms on nhrl_keypoints_eval_test: pooled, heading, per venue, per recording.

  Step 5 of docs/experiments/perception/synthetic_domain_mix_plan_2026-09-12.md. Reads
  each arm's square-640 engine at data/models/<model>_<arm>_<date>_<ckpt>_x86_64_<sm>.engine, the
  name run_domain_mix_arm.sh gives it. The first arm is the baseline every other arm is bootstrapped
  against.

    bash training/model_eval/score_domain_mix.sh yolo26s-pose 2026-09-13 base d2500 d5000 d10000

  An arm may carry its own checkpoint as `arm:ckpt`, which overrides CKPT for that arm alone and
  names it `arm_ckpt` in the output. The matched-presentation table needs this: each arm is read at
  the checkpoint bracketing the baseline's frame-presentations, which is a different epoch per arm.

    RUNS=opponent bash training/model_eval/score_domain_mix.sh yolo26s-pose 2026-09-13 \
        base d20000:epoch50 d40000:epoch25

  Output: training/data/nhrl_keypoints_eval_test/scores_domain_mix_<model>/
    pooled/        all 688 frames, taxonomy.yaml (every class scored together)
    opponent/      all 688 frames, taxonomy_opponent.yaml: the adoption metric
    heading/       all 688 frames, taxonomy_keypoint_ours.yaml (our robot's keypoints only)
    venue_nhrl/    the 590 NHRL May frames, opponents only
    venue_massd/   the 98 MassD August frames, opponents only
    rec_<name>/    one recording, which is one opponent, opponents only
  Every run keeps the bootstrap. 98 MassD frames is where a point estimate misleads most.

  Environment: CKPT (default last), SM (default sm86, this box), OUT, CONF (default 0.5), and RUNS
  (default all), a comma-separated subset of pooled, opponent, heading, venues and recordings. The
  matched-presentation read wants `opponent` alone: twelve runs over ten candidates is an hour of
  GPU for one column of one table.
  ```

