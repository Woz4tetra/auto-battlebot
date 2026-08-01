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
