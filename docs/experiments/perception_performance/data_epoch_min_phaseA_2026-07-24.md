# Data + epoch minimization — Phase A (bbox opponent detector), 2026-07-24

Executing `data_epoch_min_plan.md` Phase A on megamind (3× A6000, sm86). Baseline
`yolo26n_nhrl_robots_bbox_2026-07-16` (500 ep, real-only). Substrate `nhrl_robots_bbox`
(train 49086 / val 5454, real). Verdicts on the independent eval `nhrl_keypoints_eval_test`
(372 reviewed frames), paired bootstrap 1000×, `--labels "opponent,opponent,house_bot,
mr_stabs_mk2,mrs_buff_mk3"`, `taxonomy.yaml`, conf 0.5. Primary parity metric: agnostic recall,
δ-gate = recall Δ CI lower bound ≥ −δ with precision/F1 not-significantly-worse.

## Terms used in this report

**recall** — of all robots actually present in the ground truth, the fraction the model found.
Misses hurt recall. *"Did it see the robot?"*

**precision** — of all the detections the model emitted, the fraction that were real robots. False
positives hurt precision. *"When it called something a robot, was it right?"* For aim-assist this is
the safety-side metric: low precision means shooting at the wrong object.

**F1** — the harmonic mean of precision and recall, `2·(P·R)/(P+R)`: a single blended score that only
stays high when *both* are high. It is a summary, not a substitute — a model can hold F1 while trading
precision away for recall (fraction 0.5 in Exp 3 does exactly this), which is why the parity gate checks
recall and precision separately rather than trusting F1 alone.

**Δ (delta)** — candidate metric minus baseline metric, measured **paired** (both models scored on the
same frames, same thresholds, in the same `score.py` run). Positive = candidate better.

**CI** — 95% confidence interval on Δ, from a 1000-resample paired bootstrap over the 372 eval frames.
It expresses how much of Δ is real signal versus resampling noise on a small eval set.

**ns / better / worse** — the verdict column in `significance.csv`, decided purely by where the CI sits:

| verdict | CI vs zero | meaning |
|---|---|---|
| `better` | entirely above 0 | candidate significantly beats baseline |
| **`ns`** | **straddles 0** | **"not significant"** — difference indistinguishable from noise |
| `worse` | entirely below 0 | candidate significantly loses |

Because this is a **non-inferiority** study (goal: show a cheap recipe is *no worse*, not better), `ns`
is a **pass**, not a disappointment.

**δ (delta-gate, = 0.04 here)** — the equivalence margin: the largest recall drop still callable
"parity." `ns` alone is weak evidence, since a very wide CI is "not significant" merely because the eval
set is small. So the recall gate additionally demands the **CI lower bound ≥ −δ**: the result must be
both statistically indistinguishable *and* tight enough to rule out a real loss bigger than 4 points.
δ is floored by the bootstrap's own resolution — see Step 0 for how 0.04 was derived.

**agnostic** — the scoring level where every robot class collapses to one blob, i.e. "is there a robot
here at all," ignoring which robot. (`archetype`/`instance` are the finer per-class levels.)

**annealing** — gradually decaying the learning rate as training proceeds, so the model takes large,
exploratory steps early and progressively finer ones later, settling into a minimum. These runs use
Ultralytics' default `cos_lr=False`, i.e. **linear** decay from `lr0` to `lr0 × lrf` (here 0.01 → 0.0001)
— `lf(x) = (1 − x/epochs)·(1 − lrf) + lrf` (`trainer.py`) — **stretched across the run's *total* epoch
budget**, so the schedule's shape depends on the epoch count you asked for, not on absolute epoch number.

- **fully annealed** — the run reached the end of its own schedule; LR has bottomed out and the weights
  have settled into a sharp minimum.
- **under-annealed** — a mid-run checkpoint; LR is still high and the weights are still moving.

This distinction is the crux of Exp 1. `epoch150` of a 500-epoch run is **not** the same model a
dedicated 150-epoch run produces: the former is only 30% through its decay (under-annealed, LR still
high), while the latter fully anneals within its own 150-epoch budget. Exp 1 found the under-annealed
early checkpoints generalize **better** on unseen fights (recall 0.765 at ep50 of the long schedule) than
fully-annealed short runs (~0.69 for dedicated 50-epoch runs) — which is why the recommended recipe is
"early-stop the long schedule," not "train a short one."

## Step 0 — instrumentation + noise floor

- Wired `--save-period`, `--fraction`, `--seed` into `train.py` (and `--save-period`/`--fraction`
  into `fine_tune_train.py`); added `training/yolo/pool_datasets.py` (Recipe C). All validated.
- **δ = 0.04.** Prior paired run's agnostic-recall delta CI half-width 0.0227 → max(0.04, 1.5×0.0227)
  = 0.04. Baseline reproduced exactly on the sm86 engine (recall 0.742, precision 0.962, F1 0.838,
  mAP50-95 0.504).

## Exp 1 — cold-start epoch floor

### Ladder: one 500-epoch run, checkpoints every 50 (COCO-pretrained, full real data)

500 epochs in **14.6 h**. Scored `epoch{50,100,150,200,300,400,500=last.pt}` vs baseline. Agnostic
recall on the external eval:

| epoch | eval recall | Δ vs base 0.742 | recall 95% CI | precision | F1 | δ-gate |
|---|---|---|---|---|---|---|
| 50  | **0.765** | +0.023 | [+0.002, +0.047] | ns | ns | ✅ better |
| 100 | 0.729 | −0.013 | [−0.035, +0.010] | ns | ns | ✅ |
| 150 | 0.731 | −0.011 | [−0.035, +0.011] | ns | ns | ✅ |
| 200 | 0.748 | +0.006 | [−0.014, +0.024] | ns | ns | ✅ |
| 300 | 0.758 | +0.017 | [−0.002, +0.034] | ns | ns | ✅ |
| 400 | 0.746 | +0.004 | [−0.007, +0.014] | ns | ns | ✅ |
| 500 | 0.694 | −0.048 | [−0.069, −0.028] | ns | **worse** | ❌ worse |

Every checkpoint from ep50–400 clears parity; **ep500 (full run) regresses below it** (recall
0.694, F1 worse). Meanwhile same-corpus val (`results.csv metrics/mAP50-95(B)`) rose monotonically to
its max at ep400–500 (~0.707) — **val and external eval move in opposite directions past ~ep300**, the
val-trap the plan warned of, confirmed live.

### Confirm: dedicated fully-annealed short runs do NOT reach parity

Dedicated cold-start runs at `-e 50` (×3 seeds) and `-e 30`, each fully annealed over its own
budget:

| run | eval recall | Δ vs base | recall 95% CI | δ-gate |
|---|---|---|---|---|
| dedicated e30       | 0.710 | −0.032 | [−0.056, −0.006] | ❌ worse |
| dedicated e50 seed0 | 0.707 | −0.035 | [−0.061, −0.011] | ❌ worse |
| dedicated e50 seed1 | 0.688 | −0.054 | [−0.078, −0.030] | ❌ worse |
| dedicated e50 seed2 | 0.675 | −0.066 | [−0.091, −0.041] | ❌ worse |

The three e50 seeds cluster tightly (mean **0.690**, std **0.016**) — **seed variance is small**. A
fully-annealed 50-epoch run reliably lands ~0.05 below baseline. This is the LR-schedule caveat biting
in the *opposite* direction the plan anticipated: the plan expected a dedicated short run to reach
parity *at or below* the ladder estimate, but full annealing to a sharp minimum by ep50 generalizes
**worse** than the under-annealed ep50 checkpoint of a 500-epoch schedule (0.765).

### Exp 1 conclusion

- **The epoch floor is not reached by shortening the schedule.** A dedicated fully-annealed run needs
  well more than 50 epochs; at 50 it reliably underperforms (~0.69).
- **The cheap path to parity is early-stopping the standard long schedule.** ep100–150 checkpoints of
  the 500-epoch run clear δ (Δ −0.013 / −0.011, ns); ep500 overfits. So the minimal cold-start
  recipe is *run the standard (long) schedule but stop at ~150 epochs* → **~70% compute cut at
  parity** (150/500), with the added benefit of avoiding the ep500 overfit regression.
- **E_cold (operational) ≈ 150 epochs, early-stopped from the standard schedule.** A crisp "dedicated
  E-epoch run" floor below 500 does not exist for full annealing on this data.
- **Methodology refinement carried forward:** the dominant uncertainty here is *checkpoint / early-stop
  selection* (one run's ep50→ep500 spans 0.765→0.694), not seed (std 0.016). Two 500-ep runs
  (baseline 0.742 vs this run's ep500 0.694) differ by 0.048, so cross-run 500-ep spread ~0.05.
  Grade warm-start and data-floor arms on early-stopped checkpoints, and treat any single full-anneal
  point within ~0.05 of baseline as indistinguishable.

Artifacts: engines `runs/epoch_min_exports/phaseA_cold/`, scores
`nhrl_keypoints_eval_test/scores_data_epoch_min_phaseA_exp1{,_confirm,_seeds}`.

## Exp 2 — warm-start base checkpoint: not the Phase A lever (concluded, no new training)

Both Phase A warm-base candidates already saw **100% of the real `nhrl_robots_bbox` training data**:
`C_base` is the baseline itself, and `C_real` is an early checkpoint of the Exp 1 cold run. Consequences:

- **Circular for cheap reproduction.** Fine-tuning from either trivially starts at/above parity (they
  already are the target), so E_warm is ~0 by construction — it measures adaptation speed, not a floor.
- **Invalid for the data floor.** A base that already memorized the full real set cannot reveal how
  little real data suffices — the knowledge is already in the weights. The data-floor sweep (Exp 3) must
  therefore run **cold** (from COCO) with `--fraction`, not warm.
- **No `C_synth`.** Phase A trains real-only (2026-07-23 result: synthetic opponent appearance is
  wrong-feature/wrong-context), so there is no synthetic-pretrained base to build.

**Decision:** Phase A stays **cold-start, early-stopped** (the Exp 1 recipe). Warm-start-from-`C_base`
remains the right tool for the *separate* "adapt the deployed baseline to a new event/robot" task (out of
scope here) and is the ceiling anchor only. `E_warm` is not reported for Phase A. This matches the plan's
anticipated "possibly none, stay cold-start for Phase A" outcome.

## Exp 3 — real-image data floor (cold-start, `--fraction` sweep)

Cold-start (COCO), real-only, `--fraction {0.125, 0.25, 0.5}` of the 49086-image train split (1.0 = the
Exp 1 full run), val fixed to the full real held-out split. Schedule: `epochs=250 --save-period 25` (the
standard early-stop recipe; score the epoch ladder, take the best early checkpoint per fraction, since
Exp 1 showed the external-eval peak is early and the tail overfits). Verdict from the external eval,
δ-gate = 0.04. Smallest fraction whose best early checkpoint clears is the **real-image floor** `N_real*`.

Three cold runs (`epochs=250 --save-period 25`, `--fraction {0.5,0.25,0.125}`), best early checkpoint per
fraction scored vs baseline (full gate: recall CI lb ≥ −0.04 **and** precision/F1 not sig. worse):

| fraction | images | best early | recall (Δ, CI) | precision Δ | F1 Δ | full parity |
|---|---|---|---|---|---|---|
| 1.0 (Exp 1) | 49086 | ep50–150 | +0.023 … −0.011, clears | ns | ns | ✅ |
| **0.5** | 24543 | ep100 (0.778) | **+0.036** [+0.010,+0.061] | **−0.054** [−0.077,−0.034] worse | +0.000 ns | ❌ precision |
| 0.25 | 12272 | ep150 (0.712) | −0.030 [−0.057,−0.002] | −0.053 worse | −0.039 worse | ❌ |
| 0.125 | 6136 | ep150 (0.460) | −0.282 worse | worse | worse | ❌ |

Key result: **recall is robust to halving the data** (fraction 0.5 ep100 *beats* baseline on recall +0.036
and ties on F1), **but precision needs the full real set** — at 0.5, precision falls 0.962 → 0.91
(significant), and no single 0.5 checkpoint holds both recall and precision at once (the late 0.5 checkpoint
recovers precision to −0.003 ns but recall collapses to 0.391 — severe overfit on half data). Below 0.5,
everything falls apart (0.25 fails all three gates; 0.125 catastrophic, recall 0.27–0.47). The extra real
data past ~50% buys **precision** (sharpening the not-us / not-house-bot boundary), not recall — exactly
what "opponent is an open, context-recognized category" predicts.

Also note the **overfit cliff scales with data scarcity**: at fraction 0.5 the ep250 endpoint recall
collapses to 0.391 (from 0.778 at ep100); early-stopping is not optional at reduced data, it is essential.

**Output: `N_real* = 1.0`** (full 49086 real) for the strict recall+precision gate. **F1-parity floor =
0.5** (24.5k) if a precision-for-recall trade is acceptable (it is not, for aim-assist: precision = not
shooting at the wrong object). So Phase A's real data does **not** compress without losing precision.

## Exp 4 — combined minimal recipe (resolved by Exp 1 ladder + Exp 3; no new run needed)

The data floor (Exp 3) leaves **no reduced-data corner** to confirm — full real data is required for
precision. So Exp 4's "frontier corner" collapses onto full data, and the only remaining lever is epochs,
which the Exp 1 ladder already measured on full data. The `epoch100` checkpoint of the standard
500-epoch schedule (full data) clears **all three** gates paired vs baseline:

- recall Δ −0.013 CI[−0.035,+0.010] ns · precision Δ −0.007 CI[−0.023,+0.009] ns · F1 Δ −0.011
  CI[−0.027,+0.006] ns. (ep50 is stronger still: recall Δ +0.023.)

**Minimal Phase A recipe (confirmed):** *cold-start (COCO), full real `nhrl_robots_bbox`, the standard
500-epoch schedule, early-stopped/checkpointed at ~ep100 (as low as ep50).* This matches the
500-epoch full-data baseline on recall, precision, and F1 at **~1/5 the epochs (~5× compute cut, ~3 h vs
14.6 h)** and avoids the ep500 overfit regression.

**Explicitly not** "train a dedicated 100-epoch run" — Exp 1's confirm step showed a fully-annealed short
run underperforms (recall ~0.69). The recipe is *early-stop the long schedule*, whose early checkpoints are
under-annealed and generalize best. Data cannot be reduced (Exp 3). One caveat carries: parity here is a
single run's checkpoint and run-to-run variance is ~0.05, so the robust claim is "early-stopping at
ep50–150 reaches baseline parity to within run noise while cutting ~70–80 % of compute"; a second
full-schedule run's ep100 would tighten the reproducibility claim (optional follow-up).

## Phase A summary (one line)

**From COCO, full real data, standard 500-ep schedule early-stopped at ~ep100 → matches the 500-ep
baseline (recall/precision/F1) at ~5× less compute. Real data does not compress (precision needs the full
set); the epoch budget compresses ~5×; the default 500-ep full run mildly overfits the external eval.**


