# Does model size matter for the keypoint model? - 2026-09-05

Three yolo26 pose sizes (`n`, `s`, `x`) trained 200 epochs on `all_robot_keypoints` (18,447
train / 2,049 val), batch 96, imgsz 640, seed 0, then scored on `nhrl_keypoints_eval_test`
(688 frames, 8 recordings) with `score.py`, `--labels "mr_stabs_mk2,mrs_buff_mk3,opponent"`,
`taxonomy_keypoint.yaml`, four confidence thresholds, paired bootstrap 1000x, baseline `n`.
The currently deployed `our_robots` keypoint model is scored alongside as a reference. Arm C
finished 2026-09-06.

Companion to `model_size_2026-09-04.md`, which asked the same question of the bounding-box
detector. Predecessor: `all_robots_pose_2026-07-14.md`, which found a `yolo26n-pose` trained
on this corpus localized keypoints worse than the dedicated `our_robots` model.

## Headline

1. **Model size matters for keypoints, but only at the top of the range.** `yolo26x-pose`
   improves keypoint pixel error and PCK@0.1 against `yolo26n-pose` significantly at all four
   confidences (conf 0.5: 9.57 px / 0.765 against 11.47 / 0.691). `yolo26s-pose`, at 4x the
   baseline's parameters, improves neither at any confidence. The gain is non-monotonic in
   parameters, the same shape `model_size_2026-09-04.md` found on the box head.
2. **The registered decision rule is split.** Criterion (a) required `kp_heading_err_deg` to
   improve with a 95% CI excluding 0. `x` clears it at conf 0.05 (-5.07 deg, [-7.57, -2.47])
   and conf 0.3 (-4.17 deg, [-6.74, -1.84]); at conf 0.5 and 0.6 heading favours `x` but the
   CI includes 0. `s` fails everywhere and is significantly **worse** at conf 0.5 (+2.48 deg,
   [+0.40, +4.68]).
3. **`x`'s gain is not the threshold artifact that fools this metric.** At conf 0.05 `x`
   matches *more* boxes than `n` (1042 against 977) and scores better on all of them. That is
   the opposite signature to the epoch ladder below, where later checkpoints improved heading
   only by discarding their hard detections.
4. **`x` is the first all-robots pose model to beat the deployed `our_robots` model.** At conf
   0.5 it reaches 9.57 px / 0.765 PCK / 9.09 deg on 699 matched boxes against the deployed
   model's 9.85 / 0.704 / 10.43 on 502. This experiment reproduces
   `all_robots_pose_2026-07-14.md`'s negative result for `n` and `s`, and breaks it at `x`.
5. **Criterion (b) is what stops it.** `x` costs 2.52x `n`'s inference time on the dev box
   (+3.58 ms total, +3.29 ms of GPU time). The Jetson perception batch has roughly 1 ms of
   tick headroom, so the GPU-time increase alone exceeds the budget several times over.
6. **The 200-epoch gain is confidence calibration, not keypoint quality.** Scored at conf 0.5
   the ep100 -> ep200 heading improvement looks enormous (16.87 -> 10.45 deg). At conf 0.05,
   where the checkpoints retain comparable detection counts, keypoint placement is flat:
   14.52 / 14.54 / 14.56 px.
7. **Val is useless on this corpus.** It ranks the arms `n` < `s` < `x` monotonically on every
   metric, including the `n` -> `s` step the eval set says does not exist. The split is a
   random frame-level split of a 97.8% synthetic corpus.

## Setup

| | |
|---|---|
| Corpus | `training/data/all_robot_keypoints` - 18,447 train / 2,049 val, 97.8% synthetic |
| Classes | `mr_stabs_mk2`, `mrs_buff_mk3`, `nhrl_robot`; `kpt_shape [2, 3]`, front then back |
| Arms | `yolo26n-pose` 2.45 M / 5.4 GFLOPs; `yolo26s-pose` 9.75 M / 21.5; `yolo26x-pose` 57.55 M / 201.7 |
| Schedule | 200 epochs, batch 96, imgsz 640, `--save-period 50`, seed 0, single seed |
| Effective weight decay | 0.00075 (`trainer.py` scales the declared 0.0005 by `batch/nbs`) |
| Endpoint | epoch 200 (`last.pt`) for every arm; see "Where the pose plateau is" |
| Eval | `nhrl_keypoints_eval_test`, 688 frames / 8 recordings, `taxonomy_keypoint.yaml` |
| Reference | `yolo26n-pose_our_robots_2026-05-01`, the deployed keypoint model |
| Hardware | megamind, 3x RTX A6000 sm86, `NCCL_P2P_DISABLE=1`, via `training/gpu_queue.py` |
| Run dirs | `runs/projects/auto_battlebots_2026-09-05_{04-22-33_yolo26n-pose, 07-53-44_yolo26s-pose, 23-32-36_yolo26x-pose}` |

```bash
Q="venv/bin/python training/gpu_queue.py"
D="training/data/all_robot_keypoints/data.yml"
for M in yolo26n-pose yolo26s-pose yolo26x-pose; do
  $Q submit --name ${M}_arm --by claude-pose-size -- \
    venv/bin/python training/yolo/train.py $D $M -d 0 1 2 -b 96 -e 200 --save-period 50
done
```

18.90 h on 3 GPUs: `n` 3.49 h, `s` 4.22 h, `x` 11.18 h. `x` held 33.3 GB of each A6000's
48.5 GB at batch 96, close to the 32.4 GB the plan predicted for a `yolo26x` detect head.

`yolo26s-pose` did not exist in `train.py` and was added for this sweep.

### Label mapping

`--labels` is `mr_stabs_mk2,mrs_buff_mk3,opponent`, not the plan's literal `...,nhrl_robot`.
The eval GT vocabulary is `mr_stabs_mk2, mrs_buff_mk3, opponent, house_bot, object` and has
no `nhrl_robot` class, so that spelling would score every third-class detection as a false
positive and leave every `opponent` box unmatched. Class 2 `nhrl_robot` -> GT `opponent` is
the mapping `all_robots_pose_2026-07-14.md` used for this same 3-class model. Keypoint
matching in `score.py` is class-blind, so this choice moves the box metrics and not the
keypoint ones. Every engine parsed as `num_keypoints=2 num_classes=3`.

## Where the pose plateau is - and why one confidence would have lied

Arm A's ep100 / ep150 / ep200 checkpoints, scored at two thresholds. `boxes` is the
IoU-matched box count the keypoint metrics are computed over.

| conf | ckpt | boxes | kp_err_px | PCK@0.1 | heading deg | recall | precision |
|---|---|---:|---:|---:|---:|---:|---:|
| 0.05 | ep100 | 1096 | 14.52 | 0.513 | 24.57 | 0.765 | 0.241 |
| 0.05 | ep150 | 1063 | 14.54 | 0.519 | 23.10 | 0.742 | 0.308 |
| 0.05 | ep200 | 977 | 14.56 | 0.521 | 22.88 | 0.682 | 0.408 |
| 0.50 | ep100 | 739 | 13.67 | 0.603 | 16.87 | 0.516 | 0.673 |
| 0.50 | ep150 | 655 | 12.95 | 0.640 | 14.35 | 0.457 | 0.783 |
| 0.50 | ep200 | 492 | 11.47 | 0.691 | 10.45 | 0.343 | 0.934 |

At conf 0.5 training longer looks like it halves heading error. At conf 0.05 it does nothing:
14.52 -> 14.56 px and 0.513 -> 0.521 PCK across 100 epochs. The difference between the two
readings is the matched-box count, which falls 739 -> 492 at conf 0.5 while barely moving at
conf 0.05. Later checkpoints score their detections higher, so a fixed gate keeps a cleaner
and easier subset, and the keypoint metrics improve on the subset rather than on the model.

This is the control that makes arm C's result believable. `x` improves keypoints *while
matching more boxes*, which no threshold effect can produce.

**Endpoint choice: epoch 200 for every arm.** It is the a priori schedule endpoint, it is no
worse than ep100 on any keypoint metric at matched confidence, and picking ep100 for its
recall would be selection on the test set. `best.pt` is deliberately unused throughout: it is
chosen by val fitness, and val here does not measure generalization.

## Results - keypoint metrics, the ones that decide

Best arm per row group in bold; `deployed` is a reference, not an arm.

| conf | model | boxes | kp_err_px | PCK@0.1 | heading deg | heading acc@10deg |
|---|---|---:|---:|---:|---:|---:|
| 0.05 | deployed | 649 | 12.25 | 0.595 | 22.05 | 0.683 |
| 0.05 | n | 977 | 14.56 | 0.521 | 22.88 | 0.615 |
| 0.05 | s | 1052 | 14.84 | 0.515 | 22.68 | 0.673 |
| 0.05 | **x** | 1042 | **12.53** | **0.630** | **17.81** | **0.734** |
| 0.30 | deployed | 551 | 10.54 | 0.672 | 13.20 | 0.770 |
| 0.30 | n | 698 | 13.34 | 0.602 | 17.22 | 0.713 |
| 0.30 | s | 837 | 13.59 | 0.576 | 17.01 | 0.749 |
| 0.30 | **x** | 842 | **11.05** | **0.705** | **13.05** | **0.808** |
| 0.50 | deployed | 502 | 9.85 | 0.704 | 10.43 | 0.809 |
| 0.50 | n | 492 | 11.47 | 0.691 | 10.45 | 0.803 |
| 0.50 | s | 616 | 11.77 | 0.665 | 12.92 | 0.807 |
| 0.50 | **x** | 699 | **9.57** | **0.765** | **9.09** | **0.858** |
| 0.60 | deployed | 457 | 9.32 | 0.732 | 7.66 | 0.845 |
| 0.60 | n | 382 | 10.19 | 0.740 | 7.43 | 0.846 |
| 0.60 | s | 438 | 10.53 | 0.741 | 8.97 | 0.870 |
| 0.60 | **x** | 589 | **8.51** | **0.813** | **7.17** | **0.896** |

`x` is best on every keypoint metric at every threshold, and does it on the largest matched
sample in three of the four groups.

For scale, `experiment_runbook.md` records 9.0 deg as the good dedicated-model heading figure
and 38.5 deg as unusable for aim assist. At conf 0.5 `x` reaches 9.09 deg, the first model
measured on this corpus to reach that mark.

### Paired bootstrap, `x` against `n`

| conf | metric | n | x | delta | 95% CI | verdict |
|---|---|---:|---:|---:|---|---|
| 0.05 | kp_err_px | 14.559 | 12.528 | -2.031 | [-2.886, -1.188] | better |
| 0.05 | kp_pck@0.1 | 0.521 | 0.630 | +0.109 | [+0.084, +0.137] | better |
| 0.05 | kp_heading_err_deg | 22.882 | 17.814 | -5.068 | [-7.574, -2.471] | better |
| 0.05 | kp_heading_acc@10deg | 0.615 | 0.734 | +0.119 | [+0.090, +0.150] | better |
| 0.30 | kp_err_px | 13.338 | 11.047 | -2.291 | [-3.247, -1.383] | better |
| 0.30 | kp_pck@0.1 | 0.602 | 0.705 | +0.103 | [+0.075, +0.135] | better |
| 0.30 | kp_heading_err_deg | 17.220 | 13.053 | -4.167 | [-6.744, -1.841] | better |
| 0.30 | kp_heading_acc@10deg | 0.713 | 0.808 | +0.094 | [+0.062, +0.129] | better |
| 0.50 | kp_err_px | 11.466 | 9.570 | -1.896 | [-3.150, -0.793] | better |
| 0.50 | kp_pck@0.1 | 0.691 | 0.765 | +0.074 | [+0.044, +0.108] | better |
| 0.50 | kp_heading_err_deg | 10.447 | 9.087 | -1.359 | [-3.631, +0.729] | **ns** |
| 0.50 | kp_heading_acc@10deg | 0.803 | 0.858 | +0.056 | [+0.020, +0.089] | better |
| 0.60 | kp_err_px | 10.193 | 8.514 | -1.678 | [-2.718, -0.692] | better |
| 0.60 | kp_pck@0.1 | 0.740 | 0.813 | +0.074 | [+0.040, +0.113] | better |
| 0.60 | kp_heading_err_deg | 7.432 | 7.169 | -0.264 | [-2.123, +1.503] | **ns** |
| 0.60 | kp_heading_acc@10deg | 0.846 | 0.896 | +0.051 | [+0.016, +0.088] | better |

Fourteen of sixteen are `better` and none is `worse`. The two `ns` results are both mean
heading at the high thresholds, where the matched sample is smallest and the metric is
dominated by a handful of reversals - `kp_heading_acc@10deg` is `better` in both of those
cells, so the core of the distribution has improved even where the mean has not.

### Paired bootstrap, `s` against `n`

| conf | metric | n | s | delta | 95% CI | verdict |
|---|---|---:|---:|---:|---|---|
| 0.05 | kp_err_px | 14.559 | 14.840 | +0.281 | [-0.411, +0.994] | ns |
| 0.05 | kp_pck@0.1 | 0.521 | 0.515 | -0.006 | [-0.030, +0.019] | ns |
| 0.05 | kp_heading_err_deg | 22.882 | 22.681 | -0.201 | [-2.568, +2.266] | ns |
| 0.30 | kp_err_px | 13.338 | 13.592 | +0.254 | [-0.507, +1.012] | ns |
| 0.30 | kp_pck@0.1 | 0.602 | 0.576 | -0.026 | [-0.055, +0.002] | ns |
| 0.30 | kp_heading_err_deg | 17.220 | 17.013 | -0.208 | [-2.501, +2.049] | ns |
| 0.50 | kp_err_px | 11.466 | 11.770 | +0.304 | [-0.736, +1.230] | ns |
| 0.50 | kp_pck@0.1 | 0.691 | 0.665 | -0.026 | [-0.055, +0.006] | ns |
| 0.50 | kp_heading_err_deg | 10.447 | 12.921 | +2.475 | [+0.400, +4.683] | **worse** |
| 0.60 | kp_err_px | 10.193 | 10.534 | +0.342 | [-0.606, +1.292] | ns |
| 0.60 | kp_pck@0.1 | 0.740 | 0.741 | +0.001 | [-0.031, +0.035] | ns |
| 0.60 | kp_heading_err_deg | 7.432 | 8.972 | +1.540 | [-0.715, +3.765] | ns |

`s` buys nothing on keypoints and costs at the operating point. Had the sweep stopped at `s`,
as it nearly did, the conclusion would have been that capacity does not help keypoints at all.

![n, s and x heading on the same robots](assets/2026-09-05_pose_size/heading_mosaic.png)

Six robots all three arms detected at conf 0.5, one row each, sampled across `n`'s
heading-error distribution (the baseline, so the choice is neutral with respect to `s` and
`x`). Dashed arrow is the hand-labeled heading, solid is the prediction, back to front. The
bottom row is the tail: `n` at 179.9 deg, pointing the robot exactly backwards, where `s`
gets 3.4 deg and `x` 1.5 deg. Rows four and five are the honest other side - `x` is worse
than `n` on those two, which is what a mean improvement with a wide CI looks like up close.

## Results - box detection

Agnostic level, "did it find a robot".

| conf | n | s | x |
|---|---|---|---|
| 0.05 | 0.682 / 0.408 | 0.734 / 0.533 | 0.727 / **0.738** |
| 0.30 | 0.487 / 0.755 | 0.584 / 0.865 | **0.588** / **0.935** |
| 0.50 | 0.343 / 0.934 | 0.430 / 0.972 | **0.488** / 0.971 |
| 0.60 | 0.267 / 0.985 | 0.306 / 0.991 | **0.411** / 0.983 |

recall / precision. `x` beats `n` on recall at every threshold (+0.045 / +0.100 / +0.144 /
+0.144, all significant) and on precision at the first three. The precision gain at conf 0.05
is the largest single delta in the experiment (+0.330): `n` emits a great many low-confidence
false positives that `x` does not.

## Val, and how badly it misleads here

| arm | val box mAP50 | val box mAP50-95 | val pose mAP50 | val pose mAP50-95 | eval PCK@0.1 (conf 0.5) |
|---|---:|---:|---:|---:|---:|
| n | 0.963 | 0.676 | 0.949 | 0.902 | 0.691 |
| s | 0.980 | 0.749 | 0.963 | 0.934 | 0.665 |
| x | 0.989 | 0.797 | 0.975 | 0.965 | 0.765 |

Val improves monotonically with model size on every metric. The eval set agrees about `x` and
flatly contradicts val about `s`, which val ranks a clear second and the eval set puts last on
keypoints. Val gets the direction right by accident and the middle of the ordering wrong.

The cause is in the split: **`all_robot_keypoints` val is a random frame-level split, not
scene-disjoint.** All 2,049 val images come from scenes that also appear in train - 2,004
synthetic renders plus 45 real frames interleaved with training frames from the same four
recordings. This was checked for this experiment and is now recorded in the dataset's new
`README.md`, which the corpus previously lacked.

For the 45 real val frames this is the near-duplicate leak `nhrl_robots_bbox_2class` was
re-split to fix on 2026-07-29. For the synthetic majority it matters less - consecutive
renders differ about as much as far-apart ones (mean grey delta 15-86 against 54-71), so they
are independent draws rather than video frames. Either way val measures fit to the renderer.
**Do not rank pose arms on this val split.**

## The corpus is still the reason the absolute numbers are bad

![training corpus against the eval set](assets/2026-09-05_pose_size/corpus_mosaic.png)

Top band: synthetic renders, 19,999 of the 20,496 frames, robots on grass, ice, cobblestone
and blank grey backdrops. Middle band: all 497 real frames, every one a `mrs_buff_mk3` session
in a plywood test box. Bottom band: the eval set, the robot's own ZED inside an NHRL cage -
painted floor, arena logo, coloured lighting, debris, glass.

Arm C shows capacity is not *entirely* wasted on this corpus, which weakens the pure
data-limited reading. But the absolute numbers keep the domain gap in view: the best arm here
manages PCK@0.1 of 0.765 and 9.09 deg heading at the operating point, on a corpus whose val
split says 0.965 pose mAP50-95. A 23x parameter increase over `n` recovers part of that gap;
none of it closes it. The cheapest remaining lever is still real cage footage, and that
lever is now more attractive than a bigger model rather than less, because `x` cannot be
deployed (see below).

## Latency - dev box, idle

`benchmark_engines.py`, 300 iterations after 50 warmup, one 1280x720 eval frame, A6000 sm86,
FP16. Measured on an idle box; an earlier run of this table taken while another agent's
training job held the GPUs reported `n`'s mean GPU time above `s`'s, which is impossible, and
has been discarded.

| model | gpu median | total median | total p90 | total vs n |
|---|---:|---:|---:|---:|
| n | 1.310 ms | 2.350 ms | 3.037 ms | 1.00x |
| s | 1.651 ms | 3.037 ms | 3.413 ms | 1.29x |
| x | 4.599 ms | 5.925 ms | 7.039 ms | 2.52x |
| deployed | 1.306 ms | 2.332 ms | 2.812 ms | 0.99x |

`x`'s 2.52x matches the 2.57x the bbox sweep measured for `yolo26x` against `yolo26n`, so the
cost of the `x` backbone is consistent across heads.

**These are not Jetson numbers.** The ordering transfers; the magnitudes do not. `yolo26n`
runs ~1.3 ms here and ~9.5-11 ms inside the Jetson pipeline.

## Decision rule, as registered

- **(a) heading error improves with a 95% CI excluding 0.** `x` **passes at conf 0.05 and
  0.3**, `ns` at 0.5 and 0.6, never worse. `s` **fails**, and is significantly worse at conf
  0.5. The plan did not say which confidence (a) is evaluated at; on the reading that the
  deployed operating point governs, `x` is `ns` and (a) is not met. On the reading that the
  rule asks whether heading improves at all, `x` meets it at the two thresholds with the
  largest matched samples. Both readings are recorded here rather than resolved after the
  fact, because the rule was under-specified and picking now would be choosing the answer.
- **(b) Jetson tick stays under 33.3 ms.** **Not measured on the Jetson, and the arithmetic
  says no for `x`.** `parallel_yolo_batch/comparison.md` measured the keypoint branch at
  7.33 ms against the blob model's 7.05, a 12.86 ms batch inside a 33.17 ms tick with about
  1 ms of headroom. `x` adds +3.29 ms of pure GPU time on the dev box, and the keypoint model
  is already the slower branch, so the batch grows by at least that. Crossing the frame period
  costs ~25 ms of end-to-end latency.

**Verdict: keep `yolo26n-pose` deployed. `yolo26x-pose` is the better model and does not fit.**

## Answers

### Does a larger pose backbone reduce heading error? - **moderate yes, at `x` only**

`yolo26x-pose` improves every keypoint metric against `yolo26n-pose` at every confidence, with
mean heading significant at the two lower thresholds and `ns` at the two higher ones. It is
the first model trained on `all_robot_keypoints` to beat the deployed `our_robots` model on
keypoint placement, which is the negative result `all_robots_pose_2026-07-14.md` reported and
this sweep reverses.

`yolo26s-pose` does nothing. The shape is the same non-monotonic one the bbox sweep found,
where `s` through `l` tied and only `x` broke past - except here the tie extends all the way
down to `n`, so the useful range is narrower still. **Sizing a pose model by interpolating
between `n` and `x` would give the wrong answer at every point in between.**

### Is the latency trade-off worth it? - **no, at the current tick budget**

`x` costs 2.52x `n`'s inference time for +0.074 PCK and -1.36 deg heading at the operating
point. On a box with spare tick time that is a good trade. On this Jetson, with ~1 ms of
headroom and the keypoint model already the slower of the two parallel branches, +3.29 ms of
GPU time is roughly three times the entire budget, and crossing the frame period costs ~25 ms
end-to-end - far more than the keypoint gain is worth.

That makes `x` a target rather than a rejection. If tick time is bought back elsewhere - a
cheaper blob model, a smaller input tensor, dropping a stage - `x` is the model to spend it on.

### When do I stop training the pose model? - **moderate**

Keypoint placement plateaus by epoch 100; the remaining 100 epochs change confidence
calibration rather than keypoint quality. Running to 200 is not harmful and improves precision
at a fixed gate, but a 100-epoch schedule would have reached the same PCK, and at `x`'s 11.2 h
that is 5.6 h saved per arm. Score any future checkpoint ladder at a low confidence floor as
well as the operating point, or the calibration shift will read as an accuracy gain.

## Caveats

- **Single seed.** One run per arm. `data_epoch_min` measured ~0.048 run-to-run recall spread
  and the equivalent for heading is unmeasured, which is why the plan called for the CI. The
  `ns` verdicts mean *indistinguishable*, not equal. `x`'s keypoint gains are large enough
  (-2.0 px, +0.11 PCK) to clear plausible seed noise; the `s` -> `x` heading difference at
  conf 0.5 rests on a single seed each.
- **(a) was under-specified.** The plan named one metric and one CI but not the confidence to
  evaluate at, and `x`'s verdict changes with that choice. Future plans should register the
  threshold alongside the metric.
- **No Jetson measurement.** Criterion (b) is answered by arithmetic, not by a tick trace.
  The conclusion for `x` is robust to a large error in that arithmetic, but it is still not a
  measurement.
- **The deployed model is not a controlled comparison.** It is a 2-class model on a different
  corpus with a different synthetic generator, scored as a reference point for achievable
  keypoint quality on this eval set, not as an arm.
- **Keypoint metrics rest on a few hundred to a thousand boxes**, from the two of our robots
  the taxonomy keeps. Matched-box counts are reported beside every metric for that reason;
  `score.py` did not emit them before this experiment and now does.
- **Engines are sm86**, built and scored on megamind, not the sm89 dev box or the Jetson.

## Recommendation

- **Keep `yolo26n-pose` deployed.** Nothing that fits the current tick budget beats it.
- **Do not deploy `yolo26s-pose` under any circumstances.** It costs 1.29x `n` and is worse on
  heading at the operating point. It is dominated on both axes.
- **Treat `yolo26x-pose` as the model to buy tick time for.** It is a real improvement and the
  only arm that beats the deployed model. Measure it on the Jetson before committing, and pair
  that with a plan for where the ~3.3 ms comes from. **Not from quantizing the detector**:
  `int8_quantization_2026-09-06.md` registered that funding case as its clause (c), requiring
  a recall-neutral INT8 detector, and no arm was recall-neutral. The deployment detector lost
  0.032 recall at 8 bits, so the 3.3 ms has to come from somewhere else.
- **Add real cage footage with keypoint labels.** Still the cheapest lever, and now the only
  one that helps the model actually running on the robot.
- **Re-split `all_robot_keypoints` by scene**, or keep grading exclusively on the eval set.
  Val ranked `s` second when the eval set ranks it last.
- **Never read a single-confidence pose table.** Three conclusions in this report change
  between conf 0.05 and conf 0.5.

## What is still missing

- **Jetson latency for `x`**: build `aarch64_sm87` engines on the Orin, `sudo jetson_clocks`,
  `trtexec` plus `benchmark_engines.py`, then swap into `config/_jetson.toml`
  `[keypoint_model.engine] candidates` and read `mcap_latency_report.py --after-field-init`.
  This is the measurement that turns the (b) arithmetic into a fact.
- **`yolo26m-pose` and `yolo26l-pose`.** The plan skipped them on the bbox sweep's finding
  that `s` through `l` tied. That finding held for `s` here but the `s` -> `x` gap is now the
  whole result, so the tie may break somewhere inside it. An `l` arm (~7 h) would say whether
  any of `x`'s gain is available at less than 2.52x latency. This is the obvious next arm.
- **A seed replicate of `x`**, if it is ever a deployment candidate.

## Reproduce

```bash
# Arms, repeated at --conf 0.05 / 0.3 / 0.5 / 0.6
venv/bin/python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate n=data/models/yolo26n-pose_all_robot_keypoints_2026-09-05_last_x86_64_sm86.engine \
  --candidate s=data/models/yolo26s-pose_all_robot_keypoints_2026-09-05_last_x86_64_sm86.engine \
  --candidate x=data/models/yolo26x-pose_all_robot_keypoints_2026-09-05_last_x86_64_sm86.engine \
  --labels "mr_stabs_mk2,mrs_buff_mk3,opponent" \
  --taxonomy training/model_eval/taxonomy_keypoint.yaml \
  --conf 0.5 --baseline n --bootstrap 1000 \
  --output training/data/nhrl_keypoints_eval_test/scores_pose_size_abc/conf0.5

# Figures
venv/bin/python training/model_eval/make_pose_arms_mosaic.py training/data/nhrl_keypoints_eval_test \
  --candidate n=... --candidate s=... --candidate x=... \
  --labels "mr_stabs_mk2,mrs_buff_mk3,opponent" \
  --taxonomy training/model_eval/taxonomy_keypoint.yaml --conf 0.5 -n 6 \
  -o docs/experiments/perception_performance/assets/2026-09-05_pose_size/heading_mosaic.png

venv/bin/python training/model_eval/make_pose_corpus_mosaic.py \
  --train training/data/all_robot_keypoints --eval training/data/nhrl_keypoints_eval_test \
  --taxonomy training/model_eval/taxonomy_keypoint.yaml -n 5 \
  -o docs/experiments/perception_performance/assets/2026-09-05_pose_size/corpus_mosaic.png

# Latency (idle box only)
venv/bin/python training/model_eval/benchmark_engines.py \
  --candidate n=... --candidate s=... --candidate x=... --candidate deployed=... \
  --frame <an eval frame> --iterations 300
```

- Scores: `training/data/nhrl_keypoints_eval_test/{scores_pose_size_abc, scores_pose_size,
  scores_pose_plateau, scores_pose_plateau_sweep, scores_pose_deployed}/`
- Report assets: `assets/2026-09-05_pose_size/{heading_mosaic.png, corpus_mosaic.png}`
