# Synthetic vs real opponent detector — three clean arms, 2026-07-31

Plan: `synthetic_arms_plan.md`. Replaces the retracted `synthetic_plus_bbox_2026-07-22.md`, whose
"real" baseline was 34.6 % renders.

**Headline:** on the independent eval, adding synthetic to a verifiably-real corpus **significantly
improves precision (+0.047, CI [0.030, 0.065]) and F1 (+0.028), at no cost to recall**, and lifts both
per-class APs. **But the pre-registered adopt criterion was recall, and recall did not move
significantly** — so by the letter of the plan this is "neutral," and the precision gain is an
*unregistered* finding that needs a confirmatory run before it drives a deployment decision. Synthetic
alone remains far behind real (agnostic F1 0.545 vs 0.879).

## Provenance gate (reproduced before training)

The step whose absence invalidated the previous experiment. Counts are by filename; `synthetic` never
appears in a real frame's name, and the synthetic source's 497 real frames were **deleted**, not
filtered.

| dataset | train | synthetic | real | val (real, scene-disjoint) | synthetic in val |
|---|---|---|---|---|---|
| `nhrl_robots_bbox_2class` (real_only) | 25,914 | **0** | 25,914 | 6,573 | **0** |
| `synth_only_2class` | 17,995 | 17,995 | **0** | 6,573 | **0** |
| `mixed_2class` | 43,909 | 17,995 | 25,914 | 6,573 | **0** |

`validate_yolo_integrity --strict`: 0 errors, 0 warnings on both new sets. No training frame in any arm
comes from an eval recording. All three arms share the *same* 6,573-frame real scene-disjoint val.

## Training

`yolo26n`, 100 epochs, batch 128, imgsz 640, seed 0, 3× A6000 DDP, `--save-period 25`. Sequential, one
arm at a time. Total wall clock 5 h 25 m (1 h 36 m + 1 h 09 m + 2 h 40 m).

Val (same-corpus scene-disjoint, secondary — does not decide anything):

| arm | best val mAP50-95 | at epoch |
|---|---|---|
| real_only | 0.6668 | 69 |
| synth_only | 0.1428 | 94 |
| mixed | 0.6567 | 85 |

Every arm peaked before epoch 100 and none was still rising, confirming the 100-epoch budget was
adequate (as `data_scaling_2026-07-27` predicted).

## Result: independent eval (primary)

`nhrl_keypoints_eval_test`, 372 hand-reviewed frames of unseen real fights, `score.py`,
`--labels "opponent,house_bot"`, `taxonomy_merged.yaml`, conf 0.5, paired bootstrap 1000×, baseline
`real_only`. Agnostic level ("did it find a robot"):

| arm | precision | recall | F1 | mAP50-95 |
|---|---|---|---|---|
| real_only | 0.894 | 0.864 | 0.879 | 0.543 |
| **mixed** | **0.944** | 0.847 | **0.893** | **0.560** |
| synth_only | 0.538 | 0.553 | 0.545 | 0.263 |

Significance vs `real_only`:

| arm | metric | delta | 95 % CI | verdict |
|---|---|---|---|---|
| mixed | precision | **+0.049** | [+0.030, +0.069] | **better** |
| mixed | F1 | +0.014 | [−0.002, +0.030] | ns |
| mixed | recall | −0.017 | [−0.036, +0.001] | ns |
| synth_only | every metric | −0.30 to −0.38 | excludes 0 | **worse** |

Per-class AP50-95 (archetype level): opponent **0.494 → 0.513**, house_bot **0.682 → 0.707**. The plan's
guard was "house_bot must not drop more than 0.02"; it rose. Wrong-class rate fell 0.015 → 0.007.

## The step confound, settled

At fixed epochs `mixed` sees 1.7× the frames, so 100 epochs is 1.7× the gradient steps. This was
flagged before results were seen, with the resolution pre-chosen: compare at matched *steps* using the
period-25 checkpoints. `real_only`@100 = 2.59 M frame-presentations, which brackets between
`mixed`@ep50 (2.20 M) and `mixed`@ep75 (3.29 M).

| candidate | frame-presentations | precision | recall | F1 | opponent AP | house_bot AP |
|---|---|---|---|---|---|---|
| real_only @100 | 2.59 M | 0.894 | 0.864 | 0.879 | 0.494 | 0.682 |
| **mixed @ep50** | **2.20 M** | **0.941** (+0.047, better) | 0.874 (+0.011, ns) | **0.907** (+0.028, better) | 0.526 | 0.721 |
| mixed @ep75 | 3.29 M | 0.942 (+0.048, better) | 0.864 (0.000, ns) | 0.901 (+0.022, better) | 0.522 | 0.713 |

**`mixed` at ep50 uses fewer gradient steps than the baseline and still wins precision and F1** — so the
gain is not a step-count artifact. At matched steps the recall point estimate turns slightly *positive*
(+0.011), which is a better picture than the ep100 comparison gives.

## Decision-rule verdict — read this before citing the headline

The plan fixed these rules in advance:

> **Adopt** if `mixed` beats `real_only` on agnostic **recall** with a 95 % CI excluding 0, and
> `house_bot` AP does not drop more than 0.02.

- Recall CI includes 0 in every variant (ep100, ep50, ep75). **The adopt criterion is not met.**
- The house_bot guard passes (it rose).
- So the registered verdict is **"neutral / keep real-only,"** exactly as written.

What actually moved is **precision**, which the plan did not register. Choosing recall as the sole
adopt metric looks in hindsight like the wrong choice — precision is what a targeting stack pays for in
false locks — but changing the criterion after seeing which metric moved is how the previous experiment
got over-read. So: **the precision result is promising and unconfirmed.** To promote it, pre-register
precision/F1 and re-run with a second seed; a single seed cannot separate a +0.047 effect from
run-to-run variance.

## What synth_only establishes

Generic renders alone reach agnostic F1 **0.545** against real's 0.879, and opponent AP **0.317** against
0.494 — a fifth to two-thirds of the way, depending on metric. Its `house_bot` AP is 0.000 because the
synthetic contains no house bot; that is by construction, not failure, and is why its val mAP is not
comparable to the other arms'.

This bounds the mechanism: the renders place robots in HDRI rooms and on carpet, and the corrected
cut-paste probe found the detector needs a scene coherent with the robot. A model trained only on that
transfers poorly — yet mixed in with real data it still sharpens precision, which suggests the synthetic
contributes negative evidence ("this shape in this context is *not* a robot") more than positive
appearance coverage. That reading is a hypothesis, not a measurement.

## Cut-paste probe: synthetic trades context-reliance for appearance-reliance

The probe from the retracted report (`interpret_context_vs_appearance.py --probes cut_paste`, donor-fix
applied, 100 eval frames), now run on all three clean arms. Opponent score at the GT box, `--opp-channels
0` because `robot` is channel 0 in the 2-class head. The revealing column is **gray retention** — how
much of the original score survives when the surroundings are replaced by a neutral canvas, leaving only
the robot's pixels:

| arm | original | crop on other arena | crop on gray | robot removed | **gray retention** |
|---|---|---|---|---|---|
| real_only | 0.731 | 0.484 | 0.217 | 0.030 | **30 %** |
| mixed | 0.700 | 0.515 | 0.683 | 0.040 | **98 %** |
| synth_only | 0.604 | 0.687 | 0.770 | 0.072 | **127 %** |

**A real-only detector needs context; a synthetic-trained one does not.** Strip the arena from
`real_only` and 70 % of its evidence goes with it. Strip it from `mixed` and essentially nothing changes
(98 % retained). `synth_only` goes *further* — it scores a robot on blank gray **higher** than the same
robot in its real arena (127 %), which is exactly its training distribution: robots on plain floors in
rendered rooms. Real arena context is out-of-distribution for it.

**Except the aggregate hides a reversal.** Retention depends strongly on how big the robot is on screen,
and the arms differ only where the robot is small. Ratio of means within box-area quartiles:

| stratum | box area | n | real_only | mixed | synth_only |
|---|---|---|---|---|---|
| Q1 smallest | ≤ 923 px² | 25 | **0.12** | **1.28** | 1.32 |
| Q2 | ≤ 1,426 px² | 25 | 0.06 | 1.18 | 1.48 |
| Q3 | ≤ 2,712 px² | 25 | 0.29 | 0.96 | 1.45 |
| Q4 largest | > 2,712 px² | 25 | **0.63** | **0.60** | 0.90 |
| all | — | 100 | 0.30 | 0.98 | 1.27 |

For the **largest** quarter the three arms are effectively the same (0.63 / 0.60 / 0.90) — a close robot
is detectable from its own pixels regardless of what it trained on. The entire 30 %-vs-98 % gap lives in
the three smaller quartiles, where `real_only` collapses to 0.06–0.29 and `mixed` does not.

So the sharper claim is: **for distant robots, `real_only` is almost entirely context-driven, and adding
synthetic gives the model the ability to detect them from their own pixels.** That is the operationally
important case — a distant opponent is exactly what you want detected early — and it is a more useful
result than the aggregate suggested. It also plausibly explains the precision gain, since context-only
evidence is what produces a confident box around no robot.

**Two statistical caveats on the retention number.** It is a ratio of *means*, so high-scoring frames
dominate; computed instead as a mean of per-frame ratios it reads 1.26 (`real_only`) and 3.80 (`mixed`),
because frames where the model barely fires on the original produce enormous ratios. Neither summary is
robust on its own — the quartile table is the honest version. And the gray canvas is out-of-distribution
at its seam for every arm, so absolute values are a floor, not a point estimate.

Example inputs: `assets/cutpaste_mosaic.png`, four rows spanning the box-area range (757 → 4,443 px²),
one frame per bin, restricted to frames `real_only` detects at ≥ 0.15 — a cut-paste ratio on a box no
arm found illustrates nothing. That selection governs only what is *shown*; every number above is over
all 100 frames.

This is a coherent mechanism for the headline precision gain. An appearance-keyed detector fires less on
context-alone evidence, and `robot_removed` stays near zero for every arm (0.03–0.07), so context alone
never produces a detection in any of them. Trading context-reliance for appearance-reliance is exactly
what raises precision (+0.049) while leaving recall flat.

**It also explains the retracted report's intermediate reading.** Its "real" baseline retained 50 % on
gray — between this clean `real_only`'s 30 % and its own `mix_all`'s 69 %. That is what a
34.6 %-synthetic corpus should produce, and it is independent confirmation of the contamination from a
direction that has nothing to do with filenames. Cross-experiment magnitudes are not directly comparable
(5-class vs 2-class, 500 vs 100 epochs), but the ordering within each is the same: **more synthetic in
training → more appearance-reliance.**

Whether that trade is good depends on the failure mode you care about. It bought precision here. It also
means `mixed` would degrade less gracefully if an opponent appeared in an unfamiliar arena — and more so
if an arena-shaped distractor appeared without a robot in it.

### The other three probes

Same run, `--frames 100 --crops 200 --gradcam-k 40` (the retracted report's sample sizes). Concentration
= share of saliency ÷ share of pixels; 1.0 means no preference. See that report's "How to read the
saliency probes" for the method.

**Grad-CAM** (gradient saliency at neck layer 16):

| arm | inside box | context ring | background | n |
|---|---|---|---|---|
| real_only | 7.01 | 1.93 | 0.98 | 40 |
| mixed | 4.57 | 0.44 | 1.01 | 39 |
| **synth_only** | **28.84** | 2.25 | 0.92 | 36 |

`synth_only` at **28.8×** is the pure-appearance detector made visible — an order of magnitude above the
others, and a direct hit on the calibration written into the method note ("a detector that keyed purely
on the robot's own pixels would show an inside concentration in the tens"). Nothing in the earlier
5-class work ever produced a model like this, because no arm was ever synthetic-only.

**RISE** (occlusion saliency):

| arm | inside box | context ring | background | n |
|---|---|---|---|---|
| real_only | 1.335 | 1.266 | 0.995 | 96 |
| mixed | 1.281 | 1.221 | 0.996 | 93 |
| synth_only | 1.205 | 1.162 | 0.997 | 93 |

RISE barely separates the arms — a 0.13 spread across models whose cut-paste behaviour differs by a
factor of four. This is the resolution limit stated in the method note doing exactly what it was
predicted to do: 640/8 = 80 px mask cells against ~90×60 px robot boxes cannot resolve inside-box from
just-outside-it. **RISE should not be used to rank these arms.**

**Feature space** (linear probe separating real from synthetic opponent crops in backbone embeddings):

| arm | probe acc (5-fold) | real↔synth centroid cos-dist |
|---|---|---|
| real_only | 0.944 | 0.005 |
| synth_only | 0.918 | 0.004 |
| **mixed** | **0.962** | 0.003 |

**Training on the synthetic does not close the domain gap — it widens it.** `mixed` saw 17,995 rendered
frames and separates real from synthetic crops *better* than the arm that never saw one (0.962 vs
0.944). The retracted report claimed this ("the synthetic is a persistent off-distribution mode") but
could not support it, since its baseline was already a third synthetic and the probe was really
separating two render batches. On clean arms the claim holds, and holds more strongly than stated.

### Where the probes disagree, and what to trust

Grad-CAM and cut-paste agree emphatically on `synth_only` (28.8× inside, 127 % gray retention: pure
appearance) and **contradict each other on `real_only` vs `mixed`**. Cut-paste says `mixed` is far more
appearance-reliant (98 % vs 30 % gray retention); Grad-CAM says `mixed` is *less* box-focused (4.57 vs
7.01), with its ring collapsing to 0.44 while `real_only`'s sits at 1.93.

This is not resolved here. Two candidate explanations, neither tested: gradient saturation — `mixed` is
the higher-confidence model, and a saturated sigmoid flattens the gradients Grad-CAM weights by, which
would depress its CAM without any change in what the model uses; or the probes genuinely measure
different things — cut-paste is a causal intervention (delete the context, observe the score),
Grad-CAM is a local linearization that never removes anything.

**Weight the causal probe.** Cut-paste changes the input and reads the consequence; Grad-CAM infers
from gradients at a single point and is the probe the earlier work already flagged as "coarse and known
to be unreliable alone." The appearance-reliance conclusion rests on cut-paste, with Grad-CAM
corroborating only the `synth_only` extreme. Settling the `real_only`-vs-`mixed` disagreement needs a
probe that is causal and resolution-independent — e.g. sliding an occluder at fixed size and reading
score decay per distance from the box.

## Caveats

- **One seed per arm.** Every delta here is a single training run. The `synth_only` gap is far too large
  to be seed noise; the `mixed` precision gain (+0.047) is not.
- **Same-corpus val disagrees with the independent eval, again.** `mixed` is slightly *worse* on val
  mAP50-95 (0.6567 vs 0.6668) and clearly better on the eval that matters. Third time this pattern has
  appeared; treat val as a training-health readout only.
- **`best.pt` selection differs by arm** (ep69 vs ep85), since both were chosen by val fitness. The
  matched-step table sidesteps this by scoring fixed epochs.
- **2-class vocabulary.** `robot` covers our machines and opponents alike; instance identity is the
  keypoint model's job. Opponent AP here is the taxonomy's `opponent` archetype.

## Artifacts

- Datasets: `training/data/{nhrl_robots_bbox_2class, synth_only_2class, mixed_2class}`;
  `synth_only_2class/val_synthetic_heldout/` holds the parked synthetic val (unused).
- Runs: `runs/projects/arm_{real_only,synth_only,mixed}_2026-07-31_*_yolo26n`, logs in
  `training/projects/synth_arms_logs/`, driver `training/yolo/run_synthetic_arms.sh`.
- Models: `data/eval_models/yolo26n_arm_{real_only,synth_only,mixed}_2026-07-31.{pt,onnx,_x86_64_sm86.engine}`
  plus `mixed_ep50` / `mixed_ep75` for the matched-step control.
- Scores: `nhrl_keypoints_eval_test/scores_synthetic_arms_2026-07-31/` and
  `.../scores_synthetic_arms_matchedsteps_2026-07-31/` (`summary.csv`, `significance.csv`,
  `headline.png`, `confusion_*.png`).
