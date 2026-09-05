# Input geometry: how should the frame reach the network?

Status: **arms A2, B and C are queued and not yet trained.** Everything below the
"Arms" heading is measured; the arm results section is empty until they run. Plan:
`input_resolution_plan.md`.

## Question

Both corpora are 16:9. Training is 1920x1080, the eval and deployment ZED are 1280x720.
Letterboxing 16:9 into a square 640x640 puts the content in 640x360 and fills the
remaining 280 rows, 43.8% of the input tensor, with grey padding. Which preprocessing
makes the best use of a fixed inference budget: keep the letterbox, drop the padding,
raise resolution, stretch, or crop to the field?

## What each geometry actually hands the detector

![input tensors and per-robot pixel budget for every arm](assets/2026-09-05_input_geometry/geometries.png)

Top row is every arm's tensor at true pixel scale, so the sizes are directly comparable
and the padding shows up as grey. Bottom row zooms the same robot out of each tensor with
nearest-neighbour sampling, so the pixel budget per robot is visible rather than
tabulated. On this eval frame:

| arm | tensor | padding | robot sqrt-area at the tensor |
|---|---|---:|---:|
| A `640x640` letterbox | 409,600 px | 43.8% | 25 px (COCO-small) |
| A2/B `384x640` letterbox | 245,760 px | 6.2% | 25 px |
| C `576x1024` letterbox | 589,824 px | 0.0% | 40 px |
| D `640x640` stretch | 409,600 px | 0.0% | 34 px |
| E `640x640` field crop | 409,600 px | 65.5% | 25 px |

Two things fall out of the picture that the plan's table did not predict.

**The stretch arm buys object scale for free.** Squeezing 16:9 into a square costs 0.5x
horizontally but only 0.89x vertically, so a robot lands at sqrt(0.5 x 0.89) = 0.67 of its
source size against 0.50 for the letterbox. That is 1.33x more robot at the same 409,600
tensor pixels and no padding. Object scale is the one thing the plan says none of the
geometries address, and D addresses it without spending anything. The cost is a
preprocessing fork, and an unverified interaction with the `degrees=45.0` rotation
augmentation.

**The field crop makes the padding worse.** Panel E uses the box the DeepLab model
actually predicts for that frame, not the label boxes. The field is wide and short, so
squeezing it into a square tensor pads more than the uncropped frame does and leaves the
robot at exactly arm A's scale. See "Arm E is cut" below.

## The trap that would have invalidated A2, B and C

The plan flagged this as an open question. It is real, and it fires on this corpus.

Ultralytics sizes rectangular batches from the images' own aspect ratios, then refuses to
shuffle when the resulting batch shapes are not all identical
(`ultralytics/models/yolo/detect/train.py:95`). It logs a warning and continues. A silent
`shuffle=False` feeds the optimizer aspect-sorted batches, which on a corpus built from
recordings means recording-ordered batches, for the whole run.

`nhrl_robots_bbox_2class` is 99.85% 1920x1080. It also carries 39 pre-letterboxed YouTube
frames at 1920x886, from one segment. Those 39 gave one batch of 810 a different shape:

```
imgsz=640:  n_batches=810  shapes=[(320, 640) x 1, (384, 640) x 809]    all equal: False
imgsz=1024: n_batches=810  shapes=[(480, 1024) x 1, (576, 1024) x 809]  all equal: False
```

0.15% of the corpus was enough to turn shuffling off. `training/yolo/make_uniform_aspect_subset.py`
builds a symlinked view without them, and every batch then comes out at one shape:

```
imgsz=640:  n_batches=809  shapes=[(384, 640) x 809]    all equal: True
imgsz=1024: n_batches=809  shapes=[(576, 1024) x 809]   all equal: True
```

Only the train split is filtered. Validation runs with `rect=True` and `shuffle=False`
regardless, so its 113 odd frames are harmless, and leaving val byte-identical keeps val
metrics comparable with arm A.

**This does put A2, B and C on 25,875 training images against arm A's 25,914.** The 0.15%
difference is three orders of magnitude below the ~0.048 run-to-run recall spread
`data_epoch_min` measured, so it cannot carry a result, but it is a difference and it is
recorded here rather than left implicit.

`train.py` gained `--imgsz` and `--rect` to run these arms. Rectangular geometry has to
come from `rect=True`, because `check_imgsz(..., max_dim=1)` rejects an `[h, w]` pair for
training.

Export was verified separately: `convert_to_onnx.py --imgsz 384 640` on the arm A weights
produces `input [1, 3, 384, 640] -> output [1, 6, 5040]`, and
`yolo_bbox_robot_blob_model.cpp:73` reads the input size from the engine, so a rectangular
engine is a drop-in swap.

## Arm E is cut

The plan's gate asked whether the field occupies a similar fraction of the frame in both
corpora, on the theory that NHRL's overhead cage cameras are framed tight on the cage and
a crop would be a no-op there while the ZED sees much less. Measured with
`training/deeplab/field_fraction_gate.py` over 250 train and 688 eval frames, using the
`field_deeplabv3p_r50_2026-07-29` model the arm would use:

| corpus | field mask | field bbox | bbox p10-p90 |
|---|---:|---:|---:|
| train | 56.0% | 62.9% | 28.9-88.7% |
| eval (pooled) | 35.2% | 52.8% | 34.4-70.8% |

Those are close, so the plan's stated cut criterion does not fire. It was the wrong
measurement. What decides the arm is how much margin the crop needs before it stops
slicing robots in half, and how much zoom survives that margin:

| margin | train kept | train whole | train zoom | eval kept | eval whole | eval zoom |
|---:|---:|---:|---:|---:|---:|---:|
| 0.00 | 94.3% | 60.8% | 1.26x | 93.7% | 64.5% | 1.38x |
| 0.10 | 98.9% | 81.9% | 1.10x | 100.0% | 88.8% | 1.27x |
| 0.20 | 99.7% | 96.4% | 1.03x | 100.0% | 95.2% | 1.20x |
| 0.35 | 100.0% | 99.9% | 1.00x | 100.0% | 96.8% | 1.12x |

At margin 0 the crop zooms, but clips 39% of training robots. At the margin that keeps
them whole, the crop covers the median training frame outright and zooms it 1.03x. Arm E
is a no-op on the corpus it would be trained on. It narrows the 1.5x train/deploy
object-scale gap only to 1.29x, and the figure above shows the square tensor giving that
back as extra padding.

Against that: a DeepLab pass over 32,487 images in dataset prep, a crop branch in both the
C++ and Python preprocessors, and a new runtime dependency of the detector on the field
estimate. Arm C buys 1.6x more pixels per robot with a drop-in engine swap. E is cut.

`training/deeplab/build_field_crop_dataset.py` is kept so the arm is one command away if
that call is wrong: `masks` caches a field box per frame, `crop` rewrites images and
labels against it.

## Arms

All trained on the uniform-aspect view of `nhrl_robots_bbox_2class`, 100 epochs, batch 96,
seed 0, three GPUs, submitted through `training/gpu_queue.py`.

| arm | model | input | resize scale from 1280 | pad | tensor px | state |
|---|---|---|---:|---:|---:|---|
| A | yolo26n | 640x640 letterbox | 0.50 | 43.8% | 409,600 | reused, `2026-09-04_00-56-05_yolo26n` |
| A2 | yolo26n | 384x640 letterbox | 0.50 | 6.3% | 245,760 | queued |
| B | yolo26s | 384x640 letterbox | 0.50 | 6.3% | 245,760 | queued |
| C | yolo26n | 576x1024 letterbox | 0.80 | 0% | 589,824 | queued |
| D | yolo26n | 640x640 anisotropic | 0.50 x / 0.89 y | 0% | 409,600 | waits on A2/B/C |
| E | yolo26n | 640x640 field-cropped | variable | varies | 409,600 | cut, see above |

## Results

Pending. Arms A2, B and C are queued behind other work on the shared GPUs.

## Decision rule, registered before looking

Rank arms on agnostic recall on the eval set against arm A, paired bootstrap 1000x, 95% CI
excluding 0, the same metric as `model_size_2026-09-04.md`. Adopt a geometry only if it is
at least recall-neutral against A and reduces measured Jetson
`runner.perception_batch.update`. Report mAP50-95 separately and do not let it drive the
decision: `mask_centroid_vs_box_2026-08-03.md` established that box tightness is not what
limits this application, since targeting uses the centroid.

Single seed per arm. The scouting deltas between geometries were 0.002-0.005 recall,
far below the ~0.048 run-to-run spread `data_epoch_min` measured, so this experiment can
establish parity and not a small win. A 0.003 difference is not a result.
