# All-robots pose model vs our-robots baseline, 2026-07-14

Both models are synthetic-heavy, so this is not a synthetic-vs-real comparison. The new pose model
adds opponent awareness the deployed baseline cannot provide, and at a low confidence floor it
localizes our robot about as well. But at the deployed operating point it regresses on our own robot in
two ways. It detects mrs_buff_mk3 with lower confidence (recall 0.72 vs 0.85 at conf 0.5, but it
recovers to 0.86 at conf 0.05, so the boxes are found, just under-scored), and it places keypoints less
accurately (9.7px vs 15.2px, PCK@0.1 0.718 vs 0.570 at conf 0.5). The cause is not real-vs-synthetic
data. It is two differences between the training sets: a different synthetic generator, and a class
rebalancing that demoted mrs_buff from the majority class to a minority behind a new, dominant opponent
class.

Recommendation: keep the `our_robots` pose model deployed for our-robot keypoint tracking. The
all-robots model is worth pursuing for opponent awareness, but first close the our-robot gap by raising
mrs_buff's weight in training (fold in the `our_robot_keypoints` set and/or render more mrs_buff-heavy
synthetic) and by investigating why the new BlenderProc synthetic yields lower-confidence, less-precise
our-robot predictions than the older synthetic the baseline trained on.

## Models scored

- New: `data/models/yolo26n-pose_all_robots_2026-07-14_x86_64_sm89.engine`, 3 classes
  (`mr_stabs_mk2, mrs_buff_mk3, nhrl_robot`), 2 keypoints each. Trained on megamind
  (`all_robot_keypoints`, 20,497 frames, ~97% synthetic), 500 epochs in 11.8 hours, DDP across 3 GPUs.
  On-val metrics were high (Box mAP50 0.971, Pose mAP50 0.958), but the val split is same-distribution
  synthetic-heavy data; the held-out real GT below is the honest test. Weights:
  `yolo26n-pose_all_robots_2026-07-14.pt`.
- Baseline: `data/models/yolo26n-pose_our_robots_2026-05-01_x86_64_sm89.engine`, 2 classes
  (`mr_stabs_mk2, mrs_buff_mk3`), the deployed keypoint model, trained on `our_robot_keypoints`.

Both are x86_64 sm89 desktop engines built from the same path (`.pt` -> ONNX -> TensorRT FP16). Treat
this as a model-weights comparison, not a latency or Jetson-parity measurement.

## Training data (the two sets are more different than they look)

Both training sets are ~99% synthetic built on the same small real mrs-buff set, so real-data volume is
not the differentiator. What differs is the synthetic generator and the class mix:

| | Baseline `our_robot_keypoints` | New `all_robot_keypoints` |
|---|---|---|
| Synthetic source | old `synthetic_keypoints` generator | new BlenderProc `synthgen` |
| Synthetic images | 34,946 | 20,000 |
| Real images | 362 (mrs-buff part1) | 497 (part1 + part2) |
| Classes | 2 (mr_stabs, mrs_buff) | 3 (+ nhrl_robot opponent) |
| mrs_buff instances | 30,269 | 18,279 |
| mrs_buff share of all instances | 64.8% | 26.0% |

Two things fall out of this:

- **mrs_buff per image is unchanged** (0.86/img old vs 0.89/img new, and mrs_buff appears in 88.9% of
  the new synthetic frames). The lower absolute count (30k -> 18k) is purely that the new run rendered
  fewer images (20,000 vs 34,946, a `--num-images 20000` choice), not a scene-composition change. At the
  old image count the new pipeline would have produced ~31k mrs_buff.
- **mrs_buff's share of the training signal collapsed** (64.8% -> 26.0%) because the new set adds a
  dominant `nhrl_robot` class (39,760 instances, ~2 per image). In the old 2-class set mrs_buff was the
  best-sampled class; in the new set it is a minority behind opponents.

## Ground truth

`training/data/nhrl_keypoints_eval_test`, 372 reviewed frames, the same set used for the blob
comparisons. Every GT label carries 2 keypoints (all 11-field rows), so keypoint metrics are
computable. Box instances: 359 mrs_buff_mk3, 461 opponent, 206 house_bot (house_bot excluded via the
keypoint taxonomy), and no mr_stabs_mk2 instances. Our robot in this GT is therefore mrs_buff_mk3
only. The opponent boxes are the generic-opponent class; the baseline has no class for them.

Scored with `--conf` sweep and `--taxonomy training/model_eval/taxonomy_keypoint.yaml`. `score.py`
auto-adds the keypoint metrics for pose engines: `kp_err_px` (mean keypoint pixel error on matched
boxes), `kp_pck@0.1` (fraction of keypoints within 0.1*box-size), `kp_heading_err_deg`, and
`kp_heading_acc@10deg` (heading is the vector between the two keypoints).

## Label mapping

- New model: `mr_stabs_mk2,mrs_buff_mk3,opponent` (class 2 `nhrl_robot` -> `opponent`).
- Baseline: `mr_stabs_mk2,mrs_buff_mk3`.

## Keypoint accuracy (the headline comparison)

Keypoint metrics per confidence, on each model's matched detections:

| conf | model | kp_err_px | PCK@0.1 | heading_err_deg | heading_acc@10deg |
|---|---|---|---|---|---|
| 0.05 | baseline | **11.7** | **0.626** | **18.0** | **0.703** |
| 0.05 | new | 20.4 | 0.383 | 33.4 | 0.533 |
| 0.30 | baseline | **10.0** | **0.708** | **10.0** | **0.789** |
| 0.30 | new | 17.9 | 0.470 | 21.7 | 0.666 |
| 0.50 | baseline | **9.7** | **0.718** | **8.9** | **0.801** |
| 0.50 | new | 15.2 | 0.570 | 15.0 | 0.771 |
| 0.60 | baseline | **9.5** | **0.736** | 6.8 | 0.834 |
| 0.60 | new | 13.4 | 0.627 | **8.6** | **0.869** |

The baseline wins keypoint pixel error and PCK at every threshold. The gap narrows as confidence
rises, because at high confidence the new model keeps only its cleanest, mostly our-robot detections;
even there (conf 0.6, where the new model's recall is only 0.26) its keypoint error is 13.4px vs the
baseline's 9.5px. Heading is the one place the new model catches up at high confidence (8.6 deg /
0.869 at conf 0.6), but only after discarding three quarters of its detections. At any usable
operating point the baseline localizes keypoints substantially tighter.

## Box detection

Instance-level box metrics per confidence, over the scored GT (mrs_buff_mk3 + opponent, 820 boxes):

| conf | model | mAP50 | precision | recall | F1 | wrong-class |
|---|---|---|---|---|---|---|
| 0.05 | new | **0.608** | 0.373 | **0.738** | 0.495 | 0.049 |
| 0.05 | baseline | 0.442 | 0.544 | 0.394 | 0.457 | 0.136 |
| 0.50 | new | **0.421** | 0.883 | **0.398** | 0.548 | 0.006 |
| 0.50 | baseline | 0.418 | **0.936** | 0.372 | 0.532 | 0.022 |

Aggregate recall looks like a wash at the operating point (0.398 vs 0.372), but the composition is the
whole story. Per-class correct/localized recall at conf 0.5:

| GT class | model | correct recall | localized recall | total |
|---|---|---|---|---|
| mrs_buff_mk3 (our robot) | baseline | **0.850** | **0.852** | 359 |
| mrs_buff_mk3 (our robot) | new | 0.721 | 0.721 | 359 |
| opponent | new | **0.145** | **0.150** | 461 |
| opponent | baseline | 0.000 | 0.013 | 461 |

This looks like the new model detects our robot worse, but the fuller picture is confidence, not
localization. mrs_buff_mk3 recall by confidence:

| conf | baseline mrs_buff recall | new mrs_buff recall |
|---|---|---|
| 0.05 | 0.90 | 0.86 |
| 0.20 | 0.87 | 0.82 |
| 0.50 | 0.85 | 0.72 |

At a low floor both models find nearly all of our robot (0.86 vs 0.90); the baseline's recall is flat
while the new model's falls off steeply as the threshold rises. That steep-vs-flat shape is the
signature of confidence dilution: the new model localizes mrs_buff but scores it lower, so more true
boxes drop below the 0.5 gate. The new model's "extra" aggregate recall at low confidence is opponents
(a new capability); its apparent our-robot regression is a threshold effect, recoverable by lowering
the confidence.

## What the new model adds: opponent awareness

Scored with the full taxonomy (opponents included), the new model reaches opponent box AP50-95 of
0.193 at conf 0.05, falling to 0.084 at conf 0.5 as the confidence gate tightens.
It is a real but weak capability: the model finds ~15% of opponents at the operating point, and its
keypoints on opponents are poor (including opponents raises mean keypoint error from 15.2 to 20.6px at
conf 0.05). Opponents appear only in the new synthetic data and never in the real GT-labeled frames, so
there is no real opponent appearance to calibrate against. The baseline offers nothing here, so any
opponent awareness is strictly new, just not yet reliable.

## Why our-robot performance regressed

Both models are synthetic-heavy, so the cause is not synthetic-vs-real. It is the two training-data
differences above, and they hit detection confidence and keypoint precision together:

- **Different synthetic generator.** The baseline learned mrs_buff from the old `synthetic_keypoints`
  renders; the new model from the BlenderProc `synthgen` renders. With the real set nearly identical,
  the change in the synthetic appearance distribution is the main lever on how confidently and how
  precisely the model recognizes real mrs_buff. The new synthetic teaches mrs_buff's shape well enough
  to localize it (recall recovers at low conf) but yields lower-confidence, lower-precision predictions
  on real frames than the old synthetic did. Which generator transfers better to real footage is the
  open question; on this GT the old one wins.
- **mrs_buff was demoted from majority to minority class.** 64.8% -> 26.0% of instances, 30k -> 18k
  absolute, behind a new 40k-instance opponent class, in a head widened from 2 to 3 classes. Fewer
  examples, a lower class prior, and a 3-way softmax all reduce mrs_buff's per-detection confidence and
  the training attention it receives. This alone explains the steep recall-vs-confidence curve.
- **Keypoints pay both costs at once.** Keypoint placement needs pixel-accurate supervision and plenty
  of it; a demoted class trained under a different synthetic generator gets less of both. Box IoU
  tolerates a few pixels of error, so detection shows only the confidence dip while keypoints show the
  full precision loss.

The relative weight of the generator change vs the class rebalancing cannot be separated from these
numbers alone; an ablation (retrain the new synthetic at 2 classes, or with mrs_buff up-weighted)
would isolate it.

## Caveats

- Keypoint metrics are computed over each model's own matched detections, so the sets differ (the new
  model matches 259 mrs_buff + 67 opponent; the baseline matches ~305 mrs_buff). The new model's number
  includes opponent keypoints; even restricted to its high-confidence our-robot-heavy subset (conf 0.6)
  it still trails, so the conclusion holds.
- x86 sm89 engines, not the Jetson aarch64 engines.
- GT has no mr_stabs_mk2 instances, so our-robot keypoint accuracy is measured on mrs_buff_mk3 only.
- The baseline's GT boxes were corrected from an earlier our-robots model, a mild home-field advantage
  on exact boxes; it does not explain a 5.5px keypoint-error gap.

## Next steps

1. Keep the `our_robots` pose model deployed for our-robot keypoint tracking. It is the more accurate
   keypoint localizer by a wide margin.
2. If the all-robots model is the goal, raise mrs_buff's weight in training. Fold the
   `our_robot_keypoints` set in (this run excluded it; it adds ~30k more mrs_buff instances from the old
   generator and shifts the class balance back toward our robots) and/or render more mrs_buff-heavy
   BlenderProc synthetic, then re-score. The fix is class balance and synthetic quality, not "more real
   data": both sets already use the same ~400-500 real mrs-buff frames.
3. Treat opponent detection as a separate problem from our-robot keypoints. The synthetic data clearly
   helps opponent coverage but cannot yet deliver usable opponent keypoints; label real opponent frames
   or accept boxes-only for opponents.
4. Before trusting any pose model on our robots, watch `kp_err_px` and `heading_acc@10deg`, not box
   mAP. Box mAP hid the entire regression here.

Scores and plots: `assets/2026-07-14_pose/`. Reproduce with, for the new model:

```bash
python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate pose_all=data/models/yolo26n-pose_all_robots_2026-07-14_x86_64_sm89.engine \
  --labels "mr_stabs_mk2,mrs_buff_mk3,opponent" \
  --conf 0.5 --taxonomy training/model_eval/taxonomy_keypoint.yaml
```

and for the baseline:

```bash
python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate pose_our=data/models/yolo26n-pose_our_robots_2026-05-01_x86_64_sm89.engine \
  --labels "mr_stabs_mk2,mrs_buff_mk3" \
  --conf 0.5 --taxonomy training/model_eval/taxonomy_keypoint.yaml
```
