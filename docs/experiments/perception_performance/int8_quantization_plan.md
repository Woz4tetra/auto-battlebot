# INT8 quantization: what does 8-bit cost, and what does it buy?

Three experiments finished in the last two days and all three point at the same
unspent lever. `model_size_2026-09-04.md` found that capacity is the binding
constraint and that `yolo26x` is the only arm that breaks past the `s`/`l` tie,
then measured it at 58.22 ms of `runner.tick` against a 33.3 ms frame period.
`input_geometry_2026-09-05.md` moved the deployment model to `yolo26s` at
384x640 and showed that geometry is a latency lever, not an accuracy one.
`pose_model_size_2026-09-05.md` found `yolo26x-pose` worth having and asked for
~3.3 ms of tick time to pay for it.

The Orin Nano is already in MAXN_SUPER, so there is no clock headroom left to
buy. INT8 post-training quantization is the remaining lever that needs no new
hardware and no new training data.

## Question

Two questions, one measurement.

1. **What does INT8 cost in recall**, per model size, on the deployment eval set?
2. **Does the tick time it frees pay for something the pipeline wants?** Either a
   bigger detector (`yolo26x`), or the pose upgrade `pose_model_size_2026-09-05.md`
   asked for, or both.

## What is already measured

Detector recall on the 688-frame eval set, agnostic level, `--conf 0.5`. FP16
throughout, since no INT8 engine exists yet.

| model | input | recall | source |
|---|---|---:|---|
| `yolo26n` | 640x640 | 0.780 | `model_size_2026-09-04.md` |
| `yolo26s` | 640x640 | 0.839 | `model_size_2026-09-04.md` |
| `yolo26x` | 640x640 | **0.868** | `model_size_2026-09-04.md` |
| `yolo26n` | 384x640 (A2) | 0.784 | `input_geometry_2026-09-05.md` |
| **`yolo26s`** | **384x640 (B)** | **0.830** | `input_geometry_2026-09-05.md`, **adopted** |

Jetson `mr_stabs_mk2`, same keypoint engine, swapping only the detector:

| arm | tick mean | batch mean | bbox inference mean | loop rate |
|---|---:|---:|---:|---:|
| `n` 640x640 | 32.67 ms | 14.27 ms | 7.95 ms | 30.4 Hz |
| `s` 640x640 | 32.72 ms | 17.99 ms | 11.98 ms | 30.3 Hz |
| `x` 640x640 | 58.22 ms | 56.63 ms | 49.96 ms | 17.0 Hz |

Two numbers set the whole problem. `x` has to lose 24.9 ms of tick to fit the
frame period, and its detector inference alone is 49.96 ms. Nothing except
quantization is in that range: the 384x640 geometry that A2 measured is worth
14.5% of GPU time, which on `x` is about 7 ms.

## Why the baseline moved

`input_geometry_2026-09-05.md` ends on a methodological point that applies
directly here:

> the comparison that governs the decision is against the arm you would
> otherwise ship, not against the arm you happen to be running today.

Arm D looked like the find of that experiment for about a day because it was
scored against arm A. Against arm B, the arm actually being adopted, its gain
mostly evaporated. So every INT8 arm in this plan is compared against **arm B,
`yolo26s` at 384x640, recall 0.830**, and not against the `yolo26n` at 640x640
that is on the robot today.

That reframes question 2 into arithmetic. `yolo26x` at FP16 scores 0.868. Arm B
scores 0.830. **INT8 has 0.038 of recall to spend on `x` before `x` stops being
better than the thing we are already going to ship.** If quantization costs more
than that, `x` has no route into this pipeline and the question is closed.

## Why recall might degrade, and where

INT8 gives 256 values per tensor. Weights are constants the builder can inspect,
but activation ranges depend on the data, so they have to be measured by running
real frames through the network. That is calibration. Entropy calibration
histograms each tensor's activations and picks the clipping threshold that
minimizes KL divergence between the original and quantized distributions,
deliberately clipping rare outliers so the common case keeps resolution.

The failure mode this creates is specific and it is exactly this corpus's weak
spot. A distant robot occupies few pixels and produces low-amplitude features.
If a tensor's scale is set too wide, those activations fall below one
quantization step and round to zero before reaching the head. The eval set has a
median sqrt-area of 33.7 px with **44% of boxes below COCO's 32 px small
threshold** (`input_resolution_plan.md`). So the prediction to test is not a
uniform mAP sag but small and far robots disappearing while close ones survive.

## This experiment has no seed variance

Worth stating up front because it changes what the error bars mean. Every prior
experiment in this series compares two training runs, and `data_epoch_min`
measured ~0.048 run-to-run spread on this corpus, which is larger than most of
the deltas being argued over. `input_geometry_2026-09-05.md` lists single-seed
as its top open caveat for that reason.

Here the FP16 and INT8 arms are **the same weights**. Quantization is a build
step, not a training run. A recall delta between them cannot be seed noise,
so the paired bootstrap covers eval-frame sampling and nothing else, and a
0.005 difference is readable in a way it was not in the geometry report.

The one stochastic input left is which frames go into the calibration sample.
That gets bounded rather than assumed: see arm S below.

## Arms

All letterbox. All scored on the full 688-frame eval set at `--conf 0.5`,
paired bootstrap 1000x.

**Block 1, the capacity ladder at 640x640.** Directly comparable to the
published FP16 column in `model_size_2026-09-04.md`, which is what makes an
INT8 drop readable as a drop rather than as an environment change.

| arm | model | input | precision |
|---|---|---|---|
| n16 | `yolo26n_nhrl_robots_bbox_2class_2026-09-04` | 640x640 | FP16 |
| n8 | same | 640x640 | INT8 |
| s16 | `yolo26s_nhrl_robots_bbox_2class_2026-09-04` | 640x640 | FP16 |
| s8 | same | 640x640 | INT8 |
| x16 | `yolo26x_nhrl_robots_bbox_2class_2026-09-04` | 640x640 | FP16 |
| x8 | same | 640x640 | INT8 |

**Block 2, the deployment arm at 384x640.** Answers what INT8 costs the model
actually being shipped.

| arm | model | input | precision |
|---|---|---|---|
| B16 | `yolo26s_nhrl_robots_bbox_2class_rect384x640_2026-09-05` | 384x640 | FP16 |
| B8 | same | 384x640 | INT8 |

**Arm S, calibration-sample sensitivity.** Build `s8` a second time from a
disjoint 1000-frame calibration sample and score it. One extra build and one
extra score run to put a number on the only stochastic input in the design. If
the two `s8` arms differ by more than the FP16-to-INT8 delta itself, the sample
size is too small and everything else in the table needs re-reading.

`x` exists only at 640x640, while `n` and `s` exist at both. That is a
confound for latency and not for accuracy: `input_geometry_2026-09-05.md`
measured A2 against A at +0.003 recall with a CI spanning zero, so geometry does
not move accuracy at fixed object scale. Both geometries resize the 1280x720
source by exactly 0.50 and differ only in how much grey padding surrounds
identical content, so the ladder's INT8 finding transfers to 384x640 and arm B8
checks that it did.

## Phase 2, gated

Do not train anything up front. If `x8` clears the decision rule below, then and
only then:

1. Train `yolo26x` at 384x640, matching arm B's recipe (100 epochs, batch 96,
   seed 0, `--rect --imgsz 640`).
2. Build its INT8 engine and confirm recall against `x8`.
3. Take it to the Jetson.

If `x8` fails the rule, `x` is dead for this pipeline and the report says so.
That gate is the point of measuring the cheap thing first: it costs about an
hour to find out whether a seven hour training run is worth starting.

## Decision rule, registered before looking

Rank on agnostic recall on the eval set, paired bootstrap 1000x, 95% CI.

- **(a) INT8 on the shipping model.** Adopt `B8` only if its recall against `B16`
  is neutral, meaning the CI includes 0. Quantizing a model you are already
  shipping has no accuracy upside, only speed, so any significant loss is a real
  cost that has to be paid for by something named in advance.
- **(b) INT8 on `x`.** Adopt only if `x8` beats `B16` (0.830) significantly
  **and** measured Jetson `runner.tick` lands under 33.3 ms. Both halves
  required. `model_size_2026-09-04.md` already showed `x` winning on recall and
  failing on tick, and that combination is a reject.
- **(c) The pose funding case.** Independent of which detector wins, INT8 on the
  detector is worth adopting if it is recall-neutral and frees at least **3.3 ms**
  of Jetson tick, because that is the price `pose_model_size_2026-09-05.md` named
  for `yolo26x-pose`. This clause can pass while (b) fails.

Report mAP50-95 separately and do not let it drive the decision, same as the two
preceding experiments, since `mask_centroid_vs_box_2026-08-03.md` established
that box tightness is not what limits this application.

An x86 latency number cannot satisfy (b) or (c). Only the Jetson can.

## The INT8 build path

`training/yolo/convert_to_tensorrt.py` has no INT8 support today: it sets
`BuilderFlag.FP16` and its one precision flag is `--no-fp16`. A repo-wide grep
for `int8|IInt8|calibrat` returns nothing, so this is new code.

Verified against the installed TensorRT 10.14.1.48.post1 rather than assumed:
`IInt8EntropyCalibrator2`, `IInt8MinMaxCalibrator`, `BuilderFlag.INT8` and
`config.int8_calibrator` all exist and build working engines. They are marked
deprecated in favour of explicit Q/DQ, which is a different export pipeline and
out of scope. The deprecation warning fires on assigning
`config.int8_calibrator`, so it gets suppressed narrowly at that one statement.

**Calibrator.** Feeds one preprocessed frame at a time through
`auto_battlebot.trt_yolo.preprocess_frame` rather than reimplementing letterbox.
Calibration measures activation ranges, so a preprocessing mismatch between
calibration and inference poisons every scale factor in the network. Two details
that are easy to get wrong:

- Pass `letterbox_padding=CPP_LETTERBOX_PADDING` (0.1) explicitly.
  `preprocess_frame` defaults it to 0.0, `TrtYoloModel` passes 0.1, and the
  deployed engine sees 0.1.
- `preprocess_frame` returns a transposed, non-contiguous array. `TrtYoloModel._run`
  calls `np.ascontiguousarray` before upload and the calibrator must do the same.

Device memory comes from `torch`, already imported, rather than adding a pycuda
dependency to the converter. The buffer is held on the instance because TensorRT
receives only a raw address and does nothing to keep the object alive.

**Fail loudly.** TensorRT catches and swallows every exception raised inside
`get_batch` and finishes the build with whatever batches it already had,
producing a quietly miscalibrated engine that loads and runs normally. So the
calibrator counts batches in Python and the build asserts the count afterwards.
A corrupt JPEG must not silently become a bad engine.

**Input size** comes from the parsed network input shape, not `--imgsz`, so the
384x640 arms work without a second flag.

**Precision flags.** Set `INT8` and keep `FP16` set, so layers TensorRT cannot or
will not quantize fall back to FP16 rather than to FP32. Refuse `--int8 --no-fp16`.

**Filenames.** FP16 names must stay byte-identical, since `config/_desktop.toml`
and `config/_jetson.toml` reference them. The precision tag goes into the stem
ahead of the existing platform tag, so the trailing `_<arch>_sm<XX>.engine` shape
that the candidate lists match on is unchanged:

```
yolo26s_nhrl_robots_bbox_2class_rect384x640_2026-09-05_x86_64_sm86.engine        # unchanged
yolo26s_nhrl_robots_bbox_2class_rect384x640_2026-09-05_int8_x86_64_sm86.engine   # new
```

**Calibration cache** to `.cache/tensorrt/int8/<onnx-stem>.calib`. Dumped one to
confirm the format: a version header plus one `tensor_name: hex_scale` line per
tensor, with no architecture, no `sm` tag, and no CPU arch. It is genuinely
platform independent, which is what makes the Jetson follow-up short. It is
keyed to the TensorRT version string and the algorithm name, so a JetPack
shipping something other than 10.14 will reject it and need the frames locally.

**New flags:** `--int8`, `--calib-dir`, `--calib-count` (default 1000),
`--calib-cache`, `--calib-algo {entropy2,minmax}` defaulting to `entropy2`.
`--int8` with neither a calib dir nor an existing cache fails fast.

**Calibration sample.** 1000 frames from the `nhrl_robots_bbox_2class` val
split, stratified across all 26 scenes by max-min fair allocation, deterministic
with no RNG. The scenes are heavily imbalanced, so a plain stride over-weights
three cage cameras. The val split is scene-disjoint from train and shares
nothing with `nhrl_keypoints_eval_test`, so no calibration data reaches the
scored set.

`--calib-algo` exists because entropy versus minmax is a plausible argument
nobody should settle by arguing. Entropy is the general-purpose default and
clips outliers; minmax never clips, which is defensible for box regression but
cannot be applied to the head alone since the calibrator is a global setting. If
`x8` degrades badly, flipping this flag is the first thing to try and it costs
one rebuild.

## Running the arms

GPUs are idle and the queue is empty as of 2026-09-06 15:14, so there is no
contention to schedule around. Submit through `training/gpu_queue.py` anyway,
since several agents share the box.

```bash
Q="venv/bin/python training/gpu_queue.py"
M="data/models"
CALIB="training/data/nhrl_robots_bbox_2class/val/images"

$Q submit --name int8_build --by <agent> -d 0 -- \
  venv/bin/python training/yolo/convert_to_tensorrt.py \
    $M/yolo26n_nhrl_robots_bbox_2class_2026-09-04.onnx \
    $M/yolo26s_nhrl_robots_bbox_2class_2026-09-04.onnx \
    $M/yolo26s_nhrl_robots_bbox_2class_rect384x640_2026-09-05.onnx \
    --int8 --calib-dir "$CALIB" --calib-count 1000

$Q submit --name int8_build_x --by <agent> -d 0 -- \
  venv/bin/python training/yolo/convert_to_tensorrt.py \
    $M/yolo26x_nhrl_robots_bbox_2class_2026-09-04.onnx \
    --int8 --calib-dir "$CALIB" --calib-count 1000 --workspace 8
```

Everything runs on megamind (3x RTX A6000, sm86), which is where the published
`model_size` and `input_geometry` numbers were produced and where the corpus
lives. Ship code by rsyncing the individual changed files. Do not pull or push
git state there, and do not overwrite `training/gpu_queue.py`, which is ahead of
the local copy.

## Scoring

`score.py` compares every candidate against one `--baseline`, but the question
here is pairwise FP16 against INT8 within each model. So one run for the shared
table and per-model runs for the verdicts.

```bash
# Cross-arm table and figures
venv/bin/python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate n16=... --candidate n8=... \
  --candidate s16=... --candidate s8=... \
  --candidate x16=... --candidate x8=... \
  --candidate B16=... --candidate B8=... \
  --labels "opponent,house_bot" --taxonomy training/model_eval/taxonomy_merged.yaml \
  --conf 0.5 --baseline B16 --bootstrap 1000 \
  --output training/data/nhrl_keypoints_eval_test/scores_int8

# Paired verdict per model, one run each with --baseline <model>_fp16
```

Confirm every run prints `GT: 688 frames`. A count of 429 means `score.py` fell
back to the stale `.edit_state.json` and the numbers are not comparable to
either published report.

`--labels` must stay two entries long. `score.py` infers `num_classes` from its
length and a wrong count misparses the output tensor into `num_keypoints=0` and
near-zero recall, which looks like a broken engine rather than a bad flag.

## Latency

`benchmark_engines.py`, 300 timed iterations after 50 warmup, one real eval
frame, submitted through the queue so the idle-GPU check guarantees no
contention. Same protocol as the geometry report's latency table, so the rows
compose.

Then the Jetson sequence from `model_size_2026-09-04.md`: build
`aarch64_sm87` INT8 engines on the Orin, `sudo jetson_clocks`, swap into
`config/_jetson.toml` `[robot_mask_model.engine] candidates`, and read
`mcap_latency_report.py --after-field-init`.

**The x86 number does not answer the deployment question and should not be
quoted as if it does.** It is a check that quantization did something. The A6000
has roughly 768 GB/s of memory bandwidth against the Orin Nano's ~102 GB/s, and
`yolo26x` is 223 MB of weights, so the detector is far closer to bandwidth-bound
on the Jetson. INT8 halves weight traffic, which suggests the Orin gains more
than the A6000 does. That is a hypothesis the Jetson run tests, not a result.

## Cost

- Builder code and lint: a few hours, no GPU.
- INT8 builds, cold timing cache, estimated: `n` 3 to 6 min, `s` 5 to 10 min,
  `x` 20 to 45 min. INT8 builds are slower than FP16 because the builder times
  both INT8 and FP16 tactics per layer. The existing timing cache is shared
  safely: tactic entries are precision-keyed, so an INT8 build appends its own
  rather than reusing FP16 ones.
- Nine score runs and one benchmark: under an hour.
- Phase 1 total: about 2 hours of machine time.
- Phase 2, only if gated open: `yolo26x` at 384x640, roughly 7 hours.

## Caveats

- **This is post-training quantization only.** If recall collapses, the finding
  is that implicit PTQ is dead for this model, not that INT8 is. Quantization-aware
  training and explicit Q/DQ export are a different and much more expensive path,
  and nothing here rules them in or out.
- **The calibration cache is TensorRT-version keyed.** If the Jetson's JetPack
  ships a TensorRT other than 10.14, the cache is rejected and the Orin has to
  recalibrate locally, which means getting 1000 corpus frames onto it. Check the
  Jetson's version before relying on cache transport.
- **`x` at 640x640 against B at 384x640** is a geometry confound in the latency
  table only. Accuracy is unaffected per the A2-against-A result, but the two
  cannot be compared on GPU milliseconds without normalizing for tensor size.
- **Single calibration sample per arm**, bounded but not eliminated by arm S.
- **Everything is letterbox.** The stretch arms were rejected by
  `input_geometry_2026-09-05.md`, so no second preprocessing path is in scope and
  `benchmark_engines.py` timing through its default letterbox is correct here,
  unlike in that report.

## Deliverable

`docs/experiments/perception_performance/int8_quantization_<date>.md`, plus:

- an answer under a new heading in `my_takeaways.md`
- a cross-reference in `model_size_2026-09-04.md`, whose recommendation says to
  revisit `x` "only after buying back tick time elsewhere" and names two
  candidates without mentioning quantization
- a cross-reference in `pose_model_size_2026-09-05.md`, which asks for a plan for
  where `yolo26x-pose`'s ~3.3 ms comes from, if clause (c) passes
