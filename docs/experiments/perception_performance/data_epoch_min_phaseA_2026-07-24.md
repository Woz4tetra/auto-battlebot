# Data + epoch minimization — Phase A (bbox opponent detector), 2026-07-24

> **Corrected 2026-07-25.** The substrate was audited on megamind and is **not real-only** — 34.6 % of
> `nhrl_robots_bbox/train` is synthetic renders. This invalidates Exp 2 outright and re-scopes Exp 3/4.
> See §Correction. Exp 1's measurements survive; their *labels* do not.

Executing `data_epoch_min_plan.md` Phase A on megamind (3× A6000, sm86). Baseline
`yolo26n_nhrl_robots_bbox_2026-07-16` (500 ep). Substrate `nhrl_robots_bbox`
(train 49086 / val 5454 — **mixed real + synthetic, see §Corpus composition**). Verdicts on the
independent eval `nhrl_keypoints_eval_test` (372 reviewed frames), paired bootstrap 1000×,
`--labels "opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3"`, `taxonomy.yaml`, conf 0.5.
Primary parity metric: agnostic recall, δ-gate = recall Δ CI lower bound ≥ −δ with precision/F1
not-significantly-worse.

## Correction (2026-07-25) — the substrate is not real-only

Phase A was designed and written up on the premise, inherited from `synthetic_plus_bbox_2026-07-22`
(2026-07-23 update), that **the bbox detector trains real-only**. A direct audit of
`training/data/nhrl_robots_bbox` on megamind shows that premise is false. What this does and does not
change:

**Still valid — every measurement in Exp 1.** All arms were scored paired, in the same `score.py`
runs, against the same baseline engine, on the same frames and thresholds. The substrate error is
*common to every arm including the baseline*, so the deltas, CIs, and verdicts stand. The epoch
findings (early checkpoints beat late ones; ep500 regresses; dedicated short runs underperform) are
real.

**Invalid — Exp 2 in full**, for two independent reasons:

1. **Same-corpus circularity.** Both warm bases (`C_base` = the deployed baseline `.pt`; `C_real` =
   ep50 of the Exp 1 cold run) were fine-tuned on the *same corpus they were trained on*. `E_warm = 0`
   is then true by construction, not by measurement — there is no information in the fine-tuning data
   that the base has not already seen. The plan predicted exactly this for `C_base`
   (`data_epoch_min_plan.md` §Risks, "warm-from-baseline is circular"); it applies equally to `C_real`.
   **"Warm-start buys nothing, stay cold-start" is a property of the experiment's design, not a result
   about warm-starting, and must not be carried forward as guidance.**
2. **The stated reason `C_synth` could not exist is false.** Exp 2 recorded `C_synth` as
   "❌ does not exist — Phase A trains real-only … there is no synthetic bbox corpus to pretrain on."
   There is: 16997 synthetic frames carrying 56801 boxes sit inside the substrate already. A
   synthetic-pretrained bbox base was buildable the whole time.

**Re-scoped — Exp 3 and Exp 4.** `--fraction` subsamples the *merged, shuffled* train split, so it cut
real and synthetic together in roughly fixed 65/35 proportion. Exp 3 therefore measured a **corpus
floor, not a real-image floor**, and cannot attribute the precision collapse at fraction 0.5 to either
component. `N_real* = 1.0` is restated as `N_corpus* = 1.0`. See §Exp 3.

**Also affected, not re-run here.** `synthetic_plus_bbox_2026-07-22` graded "adding synthetic" on top of
a substrate that was already 35 % synthetic. Its null result is a **marginal-dose** finding ("more
synthetic on top of existing synthetic adds nothing"), not the presence/absence finding
("synthetic opponents do not help") that the 2026-07-23 update read it as — and that reading is what
removed the synthetic axis from this plan. Flagged for that report's owner; not corrected here.

## Corpus composition (audited 2026-07-25, megamind)

`nhrl_robots_bbox`, boxes by source. Real frames are `nhrl_seg_remap__*`; synthetic are
`synthetic__*` (CAD renders of our robots on rendered floors).

| split | frames | real frames | synthetic frames | synthetic share |
|---|---|---|---|---|
| train | 49086 | 32089 | 16997 | **34.6 %** |
| val | 5454 | 3529 | 1925 | **35.3 %** |

Boxes by class and source (train):

| class | real | synthetic | synthetic share |
|---|---|---|---|
| `object` (debris) | **0** | **15568** | **100 %** |
| `robot` (generic opponent) | 67976 | 16111 | 19.2 % |
| `house_bot` | 25413 | 0 | 0 % |
| `mr_stabs_mk2` | 3213 | 9502 | **74.7 %** |
| `mrs_buff_mk3` | 2088 | 15620 | **88.2 %** |

**There is no real-only bbox dataset on megamind.** `nhrl_seg/nhrl_robots` (the segmentation source of
`nhrl_robots_bbox`) carries the identical 16997/1925 synthetic split, and `synth_bbox_from_keypoints` is
97.5 % synthetic. The only real-only substrate in `training/data/` is **`nhrl_robots_indiv`** — 35745
frames across the same 56 recordings, zero synthetic, but labelled as **84-class instance
segmentation**, not bboxes. It is the ancestor of the whole chain; synthetic enters one step
downstream, at the `nhrl_seg` merge. A clean real-only bbox dataset (`nhrl_robots_bbox_real`) is
therefore derivable from it with no relabelling and no collection — see `data_epoch_min_plan.md`
§Step 1a, which designates it the substrate for all future bbox training.

Three consequences worth stating plainly:

- **`object` is a synthetic-only class.** The detector's class 0 was learned entirely from renders; no
  real frame carries an `object` label.
- **Synthetic already supplies generic opponents.** 16111 synthetic `robot` boxes contradict the
  "bbox trains real-only, so synthetic opponent appearance is out of scope" premise.
- **Our own robots are mostly synthetic.** The instance classes the aim-assist actually consumes are
  75–88 % rendered.

### Val is a doubly-broken proxy

Exp 1 observed val rising monotonically while the external eval fell past ~ep300, and attributed it to
the same-corpus trap. The audit shows *two* mechanisms, both worse than assumed:

1. **Val shares every recording with train.** 56 distinct real recordings appear in train; the *same
   56* appear in val. The split is frame-level random, so val frames are neighbouring frames of the
   same fights — near-duplicates, not held-out scenes.
2. **Val is 35 % synthetic.** Roughly a third of the val metric measures synthetic-domain performance,
   which the external eval does not contain at all.

Val was never measuring generalization. Exp 1's "grade on the external eval only" conclusion is
correct and now has a mechanism.

### Scoring caveat — `object` predictions are not excluded

`taxonomy.yaml` carries `exclude: [object]`, intended to drop debris from scoring. `score.py:244-245`
applies that exclusion to the **mapped** label, i.e. after `--labels` renames engine classes. Phase A
passed `--labels "opponent,opponent,…"`, renaming engine class 0 (`object`) to `opponent`, so:

- **GT** debris is excluded (eval GT literally names it `object` — 8 boxes across the eval set), but
- **predicted** debris is scored as an opponent detection.

The asymmetry inflates the false-positive opponent count for every model with a class-0 head. It
applies identically to the baseline and every candidate, so **paired deltas are unaffected**; absolute
precision and recall are biased and should not be compared against reports using a different
`--labels`. Fixing it means passing `--labels "object,opponent,…"` so the taxonomy exclusion fires on
both sides.

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
- **Not done, and it should have been: a corpus audit.** The instrumentation step validated the
  *levers* but never checked what the substrate contained. Counting frames and boxes by source is a
  30-second check that would have caught the real-only error before 14.6 h of training. It is now a
  mandatory step in the reworked plan.

## Exp 1 — cold-start epoch floor

### Ladder: one 500-epoch run, checkpoints every 50 (COCO-pretrained, full substrate)

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
its max at ep400–500 (~0.707) — **val and external eval move in opposite directions past ~ep300**.
The plan called this the "val trap"; §Corpus composition gives it two concrete mechanisms (val shares
all 56 recordings with train, *and* val is 35 % synthetic).

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

**Scope note (2026-07-25):** these conclusions describe training on the *mixed* substrate. Whether the
early-checkpoint advantage and the ep500 regression persist on a real-only corpus is untested — the
synthetic third may itself be driving the late-epoch overfit. The reworked plan measures this.

Artifacts: engines `runs/epoch_min_exports/phaseA_cold/`, scores
`nhrl_keypoints_eval_test/scores_data_epoch_min_phaseA_exp1{,_confirm,_seeds}`.

## Exp 2 — warm-start base checkpoint — **INVALID, superseded**

**Question as posed.** Does starting from an existing domain checkpoint instead of COCO reach parity in
fewer epochs than cold-start — i.e. is `E_warm ≪ E_cold`?

**Why the answer obtained is not an answer.** Both bases were fine-tuned on the corpus they were
already trained on:

| base | what it is | fine-tuned on | information gain |
|---|---|---|---|
| `C_base` | the deployed baseline `.pt` | full `nhrl_robots_bbox` | **none** — trained on this exact corpus |
| `C_real` | ep50 of the Exp 1 cold run | full `nhrl_robots_bbox` | **none** — same corpus |
| `C_synth` | synthetic-pretrained checkpoint | — | recorded "does not exist"; **it was buildable** (16997 synthetic frames in-substrate) |

Measured epoch-0 scores, retained for the record:

| base | agnostic recall | Δ vs baseline | verdict |
|---|---|---|---|
| `C_base` | 0.742 | 0.000 by construction | at parity |
| `C_real` (= `cold_e050`) | 0.765 | +0.023 [+0.002, +0.047] | `better` |

`E_warm = 0` follows tautologically: both bases already encode the entire fine-tuning set, so they
start at or above the gate. **This measures nothing about warm-starting.** The fine-tune ladder from
`C_base` (`-e 50 --save-period 10`) was never scored and is abandoned rather than completed — a ladder
from a base that is already the target cannot become informative.

**What the experiment should have asked.** Warm-starting is worth measuring only when the fine-tuning
data contains something the base has not seen. The 2026-07-24 writeup claimed that needs footage
"that does not exist in this dataset." It does not: the substrate holds **56 distinct real recordings**,
so held-out *scenes* supply genuinely unseen data without collecting anything new. The reworked
experiment in `data_epoch_min_plan.md` splits by scene and asks the operationally real question —
**given new footage, is it cheaper to retrain cold on everything, or to warm-start from the checkpoint
trained on the old footage?**

**What survives.** One design decision from this section is correct and carries forward: the data floor
must be measured **cold**, because a base that already trained on the full corpus would make a
reduced-data arm measure "small data *plus a checkpoint that saw everything*," which is useless to
anyone starting fresh.

## Exp 3 — corpus data floor (cold-start, `--fraction` sweep)

> **Re-scoped 2026-07-25.** Titled "real-image data floor" when written. `--fraction` subsamples the
> merged shuffled train split, so every arm cut real *and* synthetic together at the substrate's ~65/35
> ratio. This is a **corpus** floor. No arm here isolates the real axis, and the plan's real×synthetic
> question remains unmeasured for bbox.

Cold-start (COCO), `--fraction {0.125, 0.25, 0.5}` of the 49086-image train split (1.0 = the
Exp 1 full run), val fixed to the full held-out split. Schedule: `epochs=250 --save-period 25` (the
standard early-stop recipe; score the epoch ladder, take the best early checkpoint per fraction, since
Exp 1 showed the external-eval peak is early and the tail overfits). Verdict from the external eval,
δ-gate = 0.04.

| fraction | images (real + synth) | best early | recall (Δ, CI) | precision Δ | F1 Δ | full parity |
|---|---|---|---|---|---|---|
| 1.0 (Exp 1) | 49086 (32089 + 16997) | ep50–150 | +0.023 … −0.011, clears | ns | ns | ✅ |
| **0.5** | 24543 (≈16k + ≈8.5k) | ep100 (0.778) | **+0.036** [+0.010,+0.061] | **−0.054** [−0.077,−0.034] worse | +0.000 ns | ❌ precision |
| 0.25 | 12272 (≈8k + ≈4.3k) | ep150 (0.712) | −0.030 [−0.057,−0.002] | −0.053 worse | −0.039 worse | ❌ |
| 0.125 | 6136 (≈4k + ≈2.1k) | ep150 (0.460) | −0.282 worse | worse | worse | ❌ |

Key result: **recall is robust to halving the corpus** (fraction 0.5 ep100 *beats* baseline on recall
+0.036 and ties on F1), **but precision needs the full corpus** — at 0.5, precision falls 0.962 → 0.91
(significant), and no single 0.5 checkpoint holds both recall and precision at once (the late 0.5
checkpoint recovers precision to −0.003 ns but recall collapses to 0.391 — severe overfit on half data).
Below 0.5, everything falls apart (0.25 fails all three gates; 0.125 catastrophic, recall 0.27–0.47).

**The attribution is now open.** The original writeup concluded "the extra *real* data past ~50 % buys
precision, exactly what 'opponent is an open, context-recognized category' predicts." That reading is
unsupported: the arms removed real and synthetic in lockstep, so the precision loss could come from
either, or from the ratio shift as absolute counts shrink. Note the competing hypothesis the audit
raises — precision failures are false-positive opponents, and 100 % of the `object` (debris) head and
19 % of the generic-`robot` boxes are synthetic, so **losing synthetic supervision is at least as
plausible a cause as losing real**. Untangling them requires separate real and synthetic axes, which is
now in the plan.

Also note the **overfit cliff scales with data scarcity**: at fraction 0.5 the ep250 endpoint recall
collapses to 0.391 (from 0.778 at ep100); early-stopping is not optional at reduced data, it is essential.

**Output: `N_corpus* = 1.0`** (full 49086 mixed frames) for the strict recall+precision gate.
**F1-parity floor = 0.5** (24.5k) if a precision-for-recall trade is acceptable (it is not, for
aim-assist: precision = not shooting at the wrong object). The claim "Phase A's *real* data does not
compress" is **withdrawn** — it was never measured.

## Exp 4 — combined minimal recipe

The corpus floor (Exp 3) leaves no reduced-data corner, so Exp 4's frontier corner collapses onto full
data and the only remaining lever is epochs, which the Exp 1 ladder already measured. The `epoch100`
checkpoint of the standard 500-epoch schedule clears **all three** gates paired vs baseline:

- recall Δ −0.013 CI[−0.035,+0.010] ns · precision Δ −0.007 CI[−0.023,+0.009] ns · F1 Δ −0.011
  CI[−0.027,+0.006] ns. (ep50 is stronger still: recall Δ +0.023.)

**Minimal Phase A recipe (as measured):** *cold-start (COCO), the full `nhrl_robots_bbox` substrate as
it exists today (65 % real / 35 % synthetic), the standard 500-epoch schedule, early-stopped at ~ep100
(as low as ep50).* This matches the 500-epoch baseline on recall, precision, and F1 at **~1/5 the
epochs (~5× compute cut, ~3 h vs 14.6 h)** and avoids the ep500 overfit regression.

**Explicitly not** "train a dedicated 100-epoch run" — Exp 1's confirm step showed a fully-annealed short
run underperforms (recall ~0.69). The recipe is *early-stop the long schedule*, whose early checkpoints
are under-annealed and generalize best.

Two caveats carry:

- Parity here is a single run's checkpoint and run-to-run variance is ~0.05, so the robust claim is
  "early-stopping at ep50–150 reaches baseline parity to within run noise while cutting ~70–80 % of
  compute."
- The recipe is **substrate-specific**. It is validated on the mixed corpus, not on a real-only one.
  Anyone rebuilding the dataset with a different real/synthetic mix must re-measure the epoch floor.

## Phase A summary

**What holds.** From COCO, on the full `nhrl_robots_bbox` substrate, the standard 500-epoch schedule
early-stopped at ~ep100 matches the 500-epoch baseline (recall/precision/F1) at ~5× less compute. The
default 500-epoch full run mildly overfits the external eval. The corpus does not compress below 100 %
without losing precision. Same-corpus val is unusable as a verdict signal.

**What does not.** Phase A did **not** measure warm-start value (the experiment was circular), did
**not** measure a real-image floor (the sweep cut real and synthetic together), and was **not**
real-only training (34.6 % of the train split is synthetic renders, including a 100 %-synthetic
`object` class and 16111 synthetic generic-opponent boxes).

**Next.** `data_epoch_min_plan.md` is reworked around the scene-split warm-start experiment: build a
real-only bbox substrate (`nhrl_robots_bbox_real`, derived from `nhrl_robots_indiv`), hold out whole
recordings so "new data" is genuinely unseen, and train every arm real-only so the synthetic axis is
measured separately instead of being silently baked in.
