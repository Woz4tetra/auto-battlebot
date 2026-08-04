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
[docs/adding_eval_recordings.md](../../docs/adding_eval_recordings.md). It covers the full
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

- Reviewed-frame filtering: if `.edit_state.json` exists at the GT dir you pass, only
  frames listed in its `reviewed` array are scored. Without it, every label file counts.
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
  `docs/experiments/perception_performance/baseline_2026-07-07.md`.
- Related: perception reliability over a full fight uses `perception_reliability.py`.
