# model_eval

Offline detector evaluation against SVO recordings, without hand-labeling every frame.
The playback stack pre-labels frames; you correct the mistakes once; the corrected labels
score any number of candidate models.

Alignment is deterministic: message stamps come from the SVO frame stamp, so every run of
the same SVO produces the same per-frame stamps. Raw detections are recorded as JSON on
two topics:

- `/blob_detections`: robot blob model (YOLO-seg) boxes: box, confidence, class, label
- `/keypoint_detections`: keypoint model (YOLO-pose) boxes plus front/back keypoints

`export_labels.py` writes both models' datasets from one recording (side by side, sharing
images). `edit_labels.py` and `score.py` each operate on one subdataset; `score.py --topic`
must match the subdataset you point it at.

## Workflow

1. Record the labeling run (keeps `/camera/image`, `/blob_detections`, `/keypoint_detections`):

```bash
./scripts/build_and_run.sh -c config/label_playback.toml
# edit svo_file_path in config/playback.toml (or override in label_playback.toml) per video
```

2. Export YOLO pre-labels. Pass one or more recordings (or a glob); each is written under
   `--output-dir` (default `training/data/model_eval`) automatically:

```bash
python training/model_eval/export_labels.py data/recordings/*.mcap
```

Each recording produces one directory with shared images and a subdataset per model:

```
training/data/model_eval/<recording>/
    images/                     # shared PNGs (real files)
    blob/     images -> ../images, labels/ (class cx cy w h), data.yaml
    keypoint/ images -> ../images, labels/ (class cx cy w h kx ky v ...), data.yaml
```

Restrict to one model with `--topics blob` or `--topics keypoint`.

3. Correct the pre-labels (move/resize/add/delete/reclass boxes, drag keypoints).
   Point the editor at a subdataset (`blob` or `keypoint`):

```bash
python training/model_eval/edit_labels.py training/data/model_eval/<recording>/keypoint
```

Label at the finest granularity you care about (per robot instance). Edit the subdataset's
`data.yaml` to add class names the pre-labeler did not know.

4. Record each candidate model on the same SVO (no images, smaller files):

```bash
# set [robot_mask_model] model_path + label_indices in config/eval_candidate.toml
./scripts/build_and_run.sh -c config/eval_candidate.toml
```

5. Score:

```bash
python training/model_eval/score.py training/data/model_eval/<recording>/blob \
    --candidate generic=data/recordings/<run_a>.mcap \
    --candidate per_robot=data/recordings/<run_b>.mcap \
    --taxonomy training/model_eval/taxonomy.yaml    # --topic keypoint for the pose model
```

The GT directory is the subdataset (`blob` or `keypoint`); `score.py --topic` must match it.

## Output

- `summary.csv` + stdout table: mAP@.5, mAP@[.5:.95], precision/recall/F1,
  localization recall, wrong-class rate. One row per candidate per level.
  With pose GT: `kp_err_px` (mean keypoint pixel error) and `kp_pck@0.1` (fraction
  of keypoints within 10% of the GT box's longer side), over IoU-matched boxes.
- `headline.png`: agnostic recall ("found the robot") vs archetype/instance mAP
  ("named it right") per candidate. The gap is the cost of splitting `OPPONENT`.
- `confusion_<candidate>_<level>.png`: right-box-wrong-class matrix at the IoU threshold.

## Levels

- `agnostic`: all labels collapse to `robot`. Pure localization.
- `archetype`: labels map through `taxonomy.yaml` (`archetypes:`).
- `instance`: labels as-is.

GT is built once per SVO and reused for every candidate. Detections in the MCAP are
already thresholded by the C++ `confidence_threshold`; use `--conf` to re-threshold higher.

## Notes

- Detection boxes and keypoints are in original-image pixels; the JSON carries image
  width/height, so candidate runs do not need images recorded.
- The editor edits detect and pose rows; anything else (seg polygons) is preserved verbatim.
- New boxes drawn in a pose dataset inherit the keypoint count from existing boxes,
  centered in the box, ready to drag into place.
- Each subdataset's `images/` is a symlink to the recording's shared `images/`, so blob
  and keypoint labels reference one copy of the frames. Downstream tools follow the symlink.
- Editor state (reviewed frames) lives in the subdataset's `.edit_state.json`.
