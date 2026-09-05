# Input geometry: how should the frame reach the network?

`model_size_2026-09-04.md` asked how much model to buy. This asks whether the current
preprocessing is wasting what we already have.

Both corpora are exactly 16:9 - training is 1920x1080, the eval and deployment ZED are
1280x720. Letterboxing 16:9 into a square 640x640 puts the content in 640x360 and fills
the remaining **280 rows, 43.8% of the input tensor, with grey padding**.

## What is already measured

A scouting pass on 2026-09-04 exported the trained `yolo26s` at rectangular geometries and
scored them on the full 688-frame eval set. These used **square-trained weights**, so they
are a floor for what training at the geometry would give:

| input | padding | precision | recall | mAP50 | mAP50-95 | GPU ms |
|---|---:|---:|---:|---:|---:|---:|
| `s` 640x640 | 43.8% | 0.891 | 0.839 | 0.815 | 0.563 | 1.544 |
| `s` 384x640 | 6.3% | 0.881 | 0.837 | 0.812 | 0.555 | 1.222 |
| `s` 576x1024 | 0% | 0.886 | 0.842 | 0.826 | 0.590 | 1.918 |

Two things follow, and the first contradicts the obvious hypothesis:

1. **Padding is not costing accuracy.** Removing 44% of the tensor moved recall by 0.002,
   inside noise. YOLO is fully convolutional, so grey padding costs compute, not detection.
   A single frame where the square engine found 0 boxes and the rectangular one found 3 did
   **not** generalize - the full set shows parity. Do not build this experiment on the
   premise that letterboxing hurts accuracy; it does not.
2. **Padding is costing 21% of GPU time for nothing**, and that is the real prize.
   `yolo26s` at 384x640 runs at 1.222 ms against `yolo26n` at 640x640's 1.223 ms - the
   bigger model at nano cost.

Object scale is the deeper problem. At the input tensor, eval robots have a median
sqrt-area of **33.7 px with 44% below COCO's 32 px "small" threshold**, against 50.6 px and
25% in training. The deployment camera sees robots ~1.5x smaller than the corpus does.
Geometry changes that only insofar as they change the resize scale.

## Question

Which preprocessing makes the best use of a fixed inference budget: keep 640x640
letterbox, drop the padding, raise resolution, stretch, or crop to the field?

## Arms

All trained on `nhrl_robots_bbox_2class`, 100 epochs, batch 96, seed 0, `-d 0 1 2`.
Submit every arm through `training/gpu_queue.py` rather than running `train.py` directly -
several agents share the three GPUs and each arm takes all of them (see "Running the
arms"). The queue sets `NCCL_P2P_DISABLE=1` for multi-device jobs, so it no longer needs
exporting by hand.

| arm | model | input | resize scale (from 1280) | pad | tensor px | why |
|---|---|---|---:|---:|---:|---|
| A | yolo26n | 640x640 letterbox | 0.50 | 43.8% | 409,600 | baseline - **already trained** |
| A2 | yolo26n | 384x640 letterbox | 0.50 | 6.3% | 245,760 | **geometry control** |
| B | yolo26s | 384x640 letterbox | 0.50 | 6.3% | 245,760 | the deployment candidate |
| C | yolo26n | 576x1024 letterbox | 0.80 | 0% | 589,824 | more resolution, zero pad |
| D | yolo26n | 640x640 anisotropic | 0.50 x / 0.89 y | 0% | 409,600 | fill the tensor by stretching |
| E | yolo26n | 640x640, field-cropped | variable | varies | 409,600 | resolution where it matters |

**A2 exists because the arm the question asks for is confounded.** Comparing `s` at
384x640 against `n` at 640x640 varies model size *and* geometry at once, so a win cannot be
attributed. A2 isolates geometry at fixed size; B then answers the deployment question
("can I have `s` quality at `n` cost?") with A2 available to explain why. A2 costs ~1.2 h -
cheap insurance against an uninterpretable headline.

Reuse arm A from `runs/projects/auto_battlebots_2026-09-04_00-56-05_yolo26n`. It was
trained at batch 96 / 100 epochs / seed 0, which is exactly this design.

## Running the arms

A2, B and C are independent and can all be queued at once; the worker runs them one at a
time in priority then submission order.

**Two flags in the commands below do not exist yet.** `train.py` has no `--rect` (called
out under "Implementation per arm") and **no `--imgsz` either** - `imgsz` is baked into the
per-model config dicts at `train.py:110-112`, all at 640. Arm C needs 1024, so add both
flags before queueing anything. `--seed` already defaults to 0, so the design's seed needs
no flag.

```bash
Q="venv/bin/python training/gpu_queue.py"
D="training/data/nhrl_robots_bbox_2class"

$Q submit --name A2_n384x640 --by <agent> -- \
  venv/bin/python training/yolo/train.py $D yolo26n -d 0 1 2 -b 96 -e 100 --rect --imgsz 640
$Q submit --name B_s384x640 --by <agent> -- \
  venv/bin/python training/yolo/train.py $D yolo26s -d 0 1 2 -b 96 -e 100 --rect --imgsz 640
$Q submit --name C_n576x1024 --by <agent> -- \
  venv/bin/python training/yolo/train.py $D yolo26n -d 0 1 2 -b 96 -e 100 --rect --imgsz 1024

$Q status                  # --json to parse
$Q logs <id> --tail 40     # check the first epochs for the shuffle and mosaic questions
```

Do not queue D or E yet. D waits on the A2/B/C verdict, and E waits on its go/no-go field
fraction check - which is cheap enough to submit with `--priority 1` so it jumps ahead of a
queued multi-hour train.

The `train_batch*.jpg` and shuffle checks below need the run to have started. Poll
`$Q status` rather than assuming a submitted job is running; another agent's arm or the
pose sweep may be ahead of it.

## Implementation per arm

**A2, B, C - rectangular training.** `trainer.py:315` calls `check_imgsz(..., max_dim=1)`,
so training `imgsz` **must be a single int**; passing `[h, w]` fails. Rectangular batches
come from `rect=True` instead, which needs a `--rect` flag on `train.py`. With
`rect=True`, ultralytics sizes batches so the long side is `imgsz` and rounds the short
side to stride 32: `imgsz=640` gives 640x384, `imgsz=1024` gives 1024x576.

`detect/train.py:95` disables shuffling under `rect=True` **only when batch shapes differ**.
This corpus is uniformly 16:9, so every batch shape is identical and **shuffle survives** -
verify this in the log rather than assuming it, because a silent `shuffle=False` would
invalidate these arms.

Export with the rectangular `--imgsz` support added to `convert_to_onnx.py` on 2026-09-04
(`--imgsz 384 640`, validated as multiples of 32). **No C++ change is needed:**
`yolo_bbox_robot_blob_model.cpp:73` reads the input size from the engine and letterboxes
with independent `min(ratio_h, ratio_w)`, so a rectangular engine is a drop-in swap.

Open question to check before trusting these arms: mosaic composites four images onto a
square canvas, and its interaction with `rect=True` is unverified. Inspect
`train_batch*.jpg` from the first epochs and confirm the batches look like 16:9 scenes, not
distorted composites.

**D - anisotropic stretch.** Resize every training image to 640x640 ignoring aspect.
**Normalized YOLO labels need no change**: `cx cy w h` are fractions of width and height, so
an anisotropic resize leaves them identical. Copy the label files untouched. Train at
`imgsz=640`, where letterbox becomes a no-op on already-square images.

This arm needs real inference-side work, and it is the reason D is ranked below the
rectangular arms: the C++ and Python preprocessors both letterbox unconditionally, so D
needs a stretch branch in `YoloBboxRobotBlobModel::letterbox`
(`src/robot_blob_model/yolo_bbox_robot_blob_model.cpp:235`) and in
`auto_battlebot/trt_yolo.py::preprocess_frame`, plus a config flag to select it. Do not
start D until A2/B/C have reported - if the rectangular arms already capture the win, D
buys a preprocessing fork for nothing.

Expect an interaction with augmentation: training uses `degrees=45.0`, and rotating a
stretched image is not equivalent to stretching a rotated one. If D underperforms, test it
at `degrees=0` before concluding stretching is the problem.

**E - field crop.** The field is available: `field_filter.track_field` runs at
`runner.cpp:382`, **before** `perception_batch.update` at :390. It is also nearly free -
the Jetson report shows `point_cloud_field_filter.compute_field` with **n=1** over 1,844
ticks, so the field is computed once at init (120 ms) and tracked after, not recomputed
per frame. Projecting it to image space per tick is arithmetic.

The cost is on the training side: the corpus has no depth, so field regions must come from
the DeepLab RGB model (`field_deeplabv3p_r50_2026-07-29`) run over all 32,487 images, then
each image cropped to the field bbox plus margin and **labels transformed** (unlike D, a
crop does change normalized coordinates).

**Go / no-go gate, run this first - it is cheap and can cancel the arm.** Measure the
fraction of the frame the field occupies in training footage versus eval footage. NHRL
overhead cage footage is framed on the cage, so the field likely fills most of the frame
and cropping is close to a no-op there - while at deployment the ZED sees the cage from
inside and the crop is real. If those fractions differ sharply, arm E *widens* the
train/deploy domain gap instead of closing it, and should be cut rather than run.

## Decision rule - register before looking

Rank arms on **agnostic recall on the eval set** against arm A, paired bootstrap 1000x,
95% CI excluding 0 - the same metric as `model_size_2026-09-04.md`, so the two experiments
compose.

Adopt a geometry only if it is at least recall-neutral against A **and** reduces measured
Jetson `runner.perception_batch.update`. Given the scouting numbers, the expected outcome
is that A2 ties A on accuracy and wins on latency; the experiment exists to confirm that
holds when the model is *trained* at the geometry, and to see whether C's resolution gain
survives training.

Report mAP50-95 separately and do not let it drive the decision. C gained +0.027 mAP50-95
for +0.003 recall in scouting - that is localization tightness, and
`mask_centroid_vs_box_2026-08-03.md` established box tightness is not what limits this
application, since targeting uses the centroid.

## Scoring

```bash
venv/bin/python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate A_n640sq=... --candidate A2_n384x640=... --candidate B_s384x640=... \
  --candidate C_n576x1024=... --candidate D_n_stretch=... --candidate E_n_fieldcrop=... \
  --labels "opponent,house_bot" --taxonomy training/model_eval/taxonomy_merged.yaml \
  --conf 0.5 --baseline A_n640sq --bootstrap 1000 \
  --output training/data/nhrl_keypoints_eval_test/scores_input_geometry
```

Arms D and E cannot share this run unmodified: `score.py` letterboxes through
`TrtYoloModel`, so it would preprocess them differently from how they were trained. Both
need their preprocessing mode plumbed into `TrtYoloModel` before their numbers mean
anything. Score A/A2/B/C together first; add D and E once their preprocessing exists.

Latency: `benchmark_engines.py`, 300 iterations, one real eval frame, both levels. Then the
Jetson sequence from `model_size_2026-09-04.md` "What is still missing".

## Cost

Extrapolating from the measured `yolo26n` 640x640 100-epoch time of 1.83 h, scaled by
tensor pixels: **A2 ~1.2 h, B ~1.4 h, C ~2.6 h, D ~1.8 h, E ~1.8 h - about 9 h** plus arm A
reused free. Dataset prep for D is minutes; for E it is a DeepLab pass over 32,487 images.

## Risks / caveats

- **Single seed per arm**, as in the size sweep. The scouting deltas between geometries
  (0.002-0.005 recall) are far below the ~0.048 run-to-run spread `data_epoch_min`
  measured, so this experiment can likely only establish *parity*, not a small win. Say so
  in the report rather than reading a 0.003 difference as a result.
- **Scouting used square-trained weights.** Training at the geometry should be neutral or
  better, but if a rectangular arm comes in *worse* than the scouting number, suspect
  `rect=True` disabling shuffle before suspecting the geometry.
- **Changing input geometry changes the deployed preprocessing contract.** A rectangular
  engine is a drop-in for the C++ blob model, but the keypoint model and DeepLab still take
  their own input sizes; do not assume one shared resize.
- **None of this addresses object scale.** 44% of eval boxes are COCO-small at the current
  resize, and only arm C and arm E change the scale at all. If every arm ties, the
  conclusion is that geometry is a latency lever and not an accuracy one - which is still
  worth having, and points at corpus scene variety as the next thing to attack.

## Deliverable

`docs/experiments/perception_performance/input_geometry_<date>.md`, plus an answer under a
new heading in `my_takeaways.md`.
