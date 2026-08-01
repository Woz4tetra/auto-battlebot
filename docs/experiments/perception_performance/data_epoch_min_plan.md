# Adding a robot category: fine-tune vs cold retrain — experiment plan

Status: **planned**; Steps 1a and 1b are **built**. Rewritten 2026-07-25, superseding the "minimum data
+ epochs" framing and then the first "warm-start on unseen scenes" draft. Follows the
pipeline in `experiment_runbook.md` (collect → prepare megamind → train → export → score). Each phase's
writeup goes in `data_epoch_min_<phase>_<date>.md` once scored.

## Why this plan was rewritten

The previous plan asked "what is the smallest (real images, synthetic images, epochs) that reaches
parity, and can a reusable warm-start checkpoint get there cheaply?" Phase A ran and produced two
usable results (an epoch floor, a corpus floor) and one non-result: **the warm-start experiment was
circular.** Both warm bases were fine-tuned on the corpus they were already trained on, so
`E_warm = 0` was true by construction. See `data_epoch_min_phaseA_2026-07-24.md` §Exp 2.

The 2026-07-24 writeup concluded that the useful warm-start question needed footage "that does not
exist in this dataset." That is false. `nhrl_robots_bbox` contains **56 distinct real recordings**, so
holding out whole recordings supplies genuinely unseen data at zero collection cost.

A second correction forced by the same audit: **the substrate is not real-only.** 34.6 % of
`nhrl_robots_bbox/train` is synthetic renders (16997 of 49086 frames), including a 100 %-synthetic
`object` class and 16111 synthetic generic-opponent boxes. Every "real-only", "real substrate", and
"real-image floor" statement in the previous plan was wrong, and the 2026-07-23 decision to delete the
synthetic axis from Phase A rested on that error.

## Question

**When a new robot category appears in the data, is it cheaper to fine-tune the deployed checkpoint
than to retrain from scratch — and does the new category arrive without costing the old ones?**

This is the retrain decision in its concrete form. You have a detector trained on the roster you had.
A new robot — ours, `mrs_buff_mk3` — starts showing up in footage, and it is a *class the deployed
model does not have*, not just more examples of classes it knows. Two paths:

- **fine-tune** the deployed checkpoint on the enlarged corpus, growing the head from 4 classes to 5
- **retrain cold** from COCO on the enlarged corpus

The deliverable is a comparison on three axes: **GPU-hours to reach parity**, **whether the new
category is learned as well by each path**, and **whether the pre-existing categories regress** —
the failure mode fine-tuning is prone to and a cold retrain is not.

The scene split (§Step 1b) makes this measurable without collecting anything: it is temporal, and all
three real `mrs_buff_mk3` scenes are 2026-04, so the `old` half contains **zero** instances of the
class. `old` is a genuine "before the new robot existed" corpus, and `old ∪ new` is "after."

## Definitions

- **Scene** — one source recording, identified by the filename stem before `_yolo_seg__frame_NNNNNN`
  (e.g. `nhrl_seg_remap__clyde-colossus-Cage-5-Overhead-High-2026-03-07_21-08-19.654_720p`). Frames
  within a scene are consecutive video frames of the same fight and are near-duplicates of each other;
  frames across scenes are independent. **All splitting in this plan is at scene granularity.** Any
  frame-level split leaks.
- **OLD / NEW** — the two scene groups. OLD is what the warm base trained on; NEW is the "just
  captured" footage.
- **Parity** — non-inferiority vs the deployed baseline, defined in §Baselines below.

## The canonical real-only substrate — `nhrl_robots_indiv`

**No dataset on megamind is a real-only *bbox* dataset today.** Every 5-class bbox/seg tree has
synthetic pooled into it. The audit of `training/data/`:

| dataset | frames (train/val) | synthetic | labels | verdict |
|---|---|---|---|---|
| **`nhrl_robots_indiv`** | 32170 / 3574 (35745 total) | **0** | 84-class instance **segmentation** | ✅ **canonical real source** |
| `nhrl_seg/nhrl_robots` | 49086 / 5454 | 16997 / 1925 (34.6 %) | 5-class segmentation | ❌ contaminated |
| `nhrl_robots_bbox` | 49086 / 5454 | 16997 / 1925 (34.6 %) | 5-class bbox | ❌ contaminated |
| `synth_bbox_from_keypoints` | 18447 / 2049 | 17995 / 2004 (97.5 %) | 5-class bbox | synthetic source |

**`nhrl_robots_indiv` is the dataset all future bbox training must derive from.** It is the real-only
ancestor of the whole chain — 35745 frames across the **56 recordings**, zero synthetic, labelled at
per-robot-instance granularity (84 classes: `Clyde`, `Colossus`, `Mako`, `House Bot`, `Mr Stabs MK2`,
`Mrs Buff MK2`, …). The current pipeline is:

```
nhrl_robots_indiv  --remap_labels.py (remap_config_seg.toml)-->  5-class real seg
                   --+ synthetic renders (remap_config_synthetic.toml)-->  nhrl_seg/nhrl_robots
                   --seg_to_bbox.py-->  nhrl_robots_bbox        # ← synthetic enters here
```

The synthetic merge is the *second* step. Dropping it yields a clean real-only bbox dataset from the
same source, with no relabelling and no data collection.

### Step 1a — `nhrl_robots_bbox_real` — **BUILT 2026-07-25**

```
seg_to_bbox.py   nhrl_robots_indiv -o nhrl_robots_bbox_real          # 84-class polygons -> boxes
remap_labels.py  remap_config_indiv_to_bbox.toml nhrl_robots_bbox_real   # 84-class -> 5-class, in place
pose_to_bbox.py  <real subset of our_robot_keypoints> --class-map 0:3,1:4 # + real our-robot boxes
```

**Result: 36107 frames, 109970 boxes, zero synthetic.** `validate_yolo_integrity.py --strict` →
**0 errors, 0 warnings, exit 0.**

| split | frames | `object` | `robot` | `house_bot` | `mr_stabs_mk2` | `mrs_buff_mk3` |
|---|---|---|---|---|---|---|
| train | 32498 | 228 | 69785 | 25417 | 3288 | 328 |
| val | 3608 | 18 | 7688 | 2799 | 382 | 34 |
| test | 1 | 0 | 2 | 1 | 0 | 0 |

Images are **hardlinked** to `nhrl_robots_indiv` / `our_robot_keypoints` (500/500 sampled have
`nlink ≥ 2`), so the tree costs ~146 MB of labels, not 13 GB of pixels.

**Five decisions baked into the build:**

1. **`remap_config_seg.toml` was stale and is not used.** It carries 111 index entries for a source
   that now has 84 classes, mapping `44-47 → house_bot`, `69 → mr_stabs_mk2`, `70 → mrs_buff_mk3`,
   while in the current `nhrl_robots_indiv/data.yml` those names sit at **35-37**, **55**, and **56**.
   Applying it as-is silently mis-labels. Replaced by **`training/yolo/remap_config_indiv_to_bbox.toml`**,
   generated by *name* lookup (all 84 sources mapped explicitly: 2 → `object`, 3 → `house_bot`,
   1 → `mr_stabs_mk2`, 78 → `robot`). Regenerate by name if the class order ever changes; never
   hand-edit indices.
2. **`Flight Controller 1`/`2` → `object`, not `robot`.** These two source classes (135 + 111 = 246
   boxes, both confined to the single `mini_bot_2024-10-26T16-41-23` scene) are not competitors.
   Routing them to `object` — which `taxonomy.yaml` excludes from scoring — keeps them out of both the
   opponent GT and the false-positive count, rather than teaching the detector that they are opponents.
   The other odd-sounding source names *were* checked against the pixels and are genuine competitors:
   `power on` (276 boxes) and `usawgi` (1517) stay in `robot`.
3. **`Mrs Buff MK2` → `robot` (opponent), not `mrs_buff_mk3`.** The NHRL footage shows a different
   robot than our deployed Mrs Buff MK3, so folding it into our own-robot class — as the superseded
   config did, contributing 2088 boxes to the Phase A substrate — mislabels an opponent as us. Its
   2327 boxes are now opponents.
4. **Real `mrs_buff_mk3` supervision comes from `our_robot_keypoints` instead.** Its real (non-synth)
   component — **362 frames** from three April recordings (`2026-04-19T17-01-18`,
   `2026-04-19T18-42-50`, `2026-04-20T18-02-50`) — was pose→bbox converted (`--class-map 0:3,1:4`) and
   merged with an `our_robot_kp__` filename prefix, adding **362 `mrs_buff_mk3` + 108 `mr_stabs_mk2`**
   boxes. These scenes are disjoint from the `main_2026-05-*` eval recordings.
5. **127 empty label files are kept as genuine background frames.** All 127 are empty in the source
   too (zero were emptied by degenerate-polygon dropping), so they are true negatives — useful for
   precision, which Phase A identified as the metric that degrades first. The historical
   `nhrl_robots_bbox` pipeline discarded them.

**Conversion loss is negligible:** 22 of 109992 source polygons (0.02 %) were degenerate and dropped.
Class totals reconcile exactly — `house_bot` 28217 = the full `House Bot*` source count;
`mr_stabs_mk2` 3670 = 3563 source + 108 keypoint − 1 degenerate.

**Two consequences to carry into Step 1b:**

- **`object` carries only 246 boxes, all from one scene.** That is enough to define the class but not
  to train a reliable debris head, so treat class 0 as a *scoring exclusion channel* rather than a
  detection target. Score with `--labels "object,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3"` — the
  corrected mapping this plan already requires — so `taxonomy.yaml`'s `exclude: [object]` fires on both
  GT and predictions.
- **Real `mrs_buff_mk3` is now 362 boxes across 3 scenes, down from 2088.** That is the honest count:
  the previous 2088 were a *different robot*. Our-robot supervision in real footage is far scarcer than
  Phase A's numbers implied, which strengthens the case for Phase B's synthetic-substitution study. It
  also means the our-robot classes cannot be meaningfully stratified across scene groups — with 3
  `mrs_buff_mk3` scenes and the `mr_stabs_mk2` concentration already noted below, expect and report
  imbalance rather than designing around it.
- **`split_by_scene.py` must handle two filename schemes.** NHRL frames key on
  `…_yolo_seg__frame_NNNNNN`; merged keypoint frames key on
  `our_robot_kp__<name>__<timestamp>-NNNNNN_jpg.rf.<hash>`. Match both, and fail on anything that
  matches neither.

**Not merged, deliberately:** `all_robot_keypoints` also holds a 497-frame real component, but it comes
from `2026-05-01T14-17-26` — the same day as the `main_2026-05-01` eval recording. It is probably a
different session, but "probably" is not good enough for a verdict set. Confirm it is a distinct
recording before adding it.

## Corpus as trained in Phase A, for reference (megamind, 2026-07-25)

| | frames | real | synthetic |
|---|---|---|---|
| `nhrl_robots_bbox/train` | 49086 | 32089 | 16997 (34.6 %) |
| `nhrl_robots_bbox/val` | 5454 | 3529 | 1925 (35.3 %) |

Real boxes by class (train): `robot` 67976, `house_bot` 25413, `mr_stabs_mk2` 3213, `mrs_buff_mk3`
2088, `object` **0**. Synthetic boxes: `object` 15568, `robot` 16111, `mrs_buff_mk3` 15620,
`mr_stabs_mk2` 9502, `house_bot` 0.

Real scene structure: **56 recordings, 35618 real frames** (train+val combined), 9 to 1453 frames per
scene (median 575).

**The binding constraint on splitting:** only **12 of 56 scenes contain our own robots**, and one scene
(`nhrl_seg_remap__2026-03-27T21-48-29`, 1363 frames) alone holds ~42 % of real `mr_stabs_mk2` and ~65 %
of real `mrs_buff_mk3` boxes. A scene split balanced on frame count will *not* be balanced on our-robot
supervision — and this carries over unchanged to `nhrl_robots_bbox_real`, which inherits the same
56 scenes. Stratify explicitly (§Step 1b) and report the residual imbalance.

**Current train/val is a frame-level random split** — the same 56 recordings appear in both. Val frames
are neighbouring frames of train fights. Rebuilding val at scene granularity is part of Step 1b, not
optional.

## Baselines and parity definition

"Same results as baseline" is a **non-inferiority** claim and must be measured paired. In *every*
scoring run, include the baseline engine as `--baseline` alongside the candidates so parity is a
paired-bootstrap delta under identical frames/thresholds — never compare against a remembered number
from another run.

- **Baseline:** `yolo26n_nhrl_robots_bbox_2026-07-16.pt` (detect, 500 ep, batch 128).
- **Verdict set:** `nhrl_keypoints_eval_test` (372 reviewed frames, `main_2026-05-*` recordings —
  scene-disjoint from every training recording). Paired bootstrap 1000×, conf 0.5.
- **Reference numbers** (re-measured live each run): agnostic recall **0.742**, precision **0.962**,
  F1 **0.838**, opponent AP50-95 0.305, agnostic mAP50-95 0.504.

**Parity gate.** A candidate reaches parity when, at the agnostic level vs the baseline:

- recall delta CI **lower bound ≥ −δ**, and
- precision and F1 are **not significantly worse**.

**δ = 0.04**, already derived and confirmed: the prior paired run's agnostic-recall delta CI half-width
was 0.0227, so δ = max(0.04, 1.5 × 0.0227) = 0.04. Do not re-derive.

**Fix the `--labels` mapping before anything else.** `taxonomy.yaml` carries `exclude: [object]`, but
`score.py:244-245` applies exclusion to the **mapped** label. Phase A passed
`--labels "opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3"`, renaming engine class 0 (`object`)
to `opponent`, so GT debris was excluded while *predicted* debris was scored as an opponent detection.
Switch to `--labels "object,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3"` so the exclusion fires on
both sides. This shifts absolute numbers, so **re-anchor the baseline under the fixed mapping in
Step 0** and use only re-anchored numbers thereafter.

## Method

### Step 0 — re-anchor, and audit before you train

1. **Corpus audit (mandatory, ~30 s).** Count frames and boxes by source and class for any dataset
   before training on it. Phase A skipped this and spent 14.6 h training on a substrate it had
   mis-described. Print the table into the writeup.
2. **Re-anchor the baseline under the corrected `--labels` AND under both taxonomies** (§Step 4): one
   baseline-only `score.py` run per view on the sm86 engine. Record recall/precision/F1/mAP and the
   recall CI half-width for each. The common-class view drops `mrs_buff_mk3` from GT, so its reference
   numbers will *not* be the familiar 0.742 / 0.962 / 0.838 — those were measured with the class
   included. Confirm δ still holds at 0.04 per view; if a half-width grew, re-derive
   δ = max(0.04, 1.5 × half-width) for that view.
3. **Report all mappings once** so this experiment's numbers can be reconciled with Phase A's.
4. **`old_4class.yml` — written 2026-07-25.** `nc: 4`, `names` truncated after `mr_stabs_mk2`,
   `train: old/images`, `val: hold_old/images`. Both groups were asserted to contain **zero** class-4
   label lines before it was written, since ultralytics would not catch a stray one.

Instrumentation from Phase A is already wired and validated: `--save-period`, `--fraction`, `--seed` in
`train.py`; `--save-period`/`--fraction` in `fine_tune_train.py`; `training/yolo/pool_datasets.py`.

### Step 1b — the scene split — **BUILT 2026-07-25**

```
split_by_scene.py --src training/data/nhrl_robots_bbox_real \
                  --out training/data/scenesplit_2026-07-25 \
                  --mode temporal --cutoff 2025-11 --holdout-frac 0.2 \
                  --stratify-class robot,house_bot
```

**Split on recording date, not stratified — a deliberate change from this plan's first draft.** A
stratified split makes OLD and NEW the *same distribution*, and then there is nothing unique for a
new-data-only fine-tune to forget: retention and acquisition measure the same thing and the forgetting
probe is vacuous. Real new footage arrives from a *later event* and genuinely differs in cage,
lighting, and robot roster. Scene names carry dates (explicit `YYYY-MM-DD`, NHRL tournament tags like
`nhrl_apr25_`, or unix epoch stems), so all 59 scenes date cleanly across **2024-06 → 2026-04** and a
temporal cut at **2025-11** lands near 50/50. Class balance is then whatever the calendar gives; it is
**reported, not corrected**. `--mode stratified` remains available for a same-distribution control.

**Result: 59 scenes, 36107 frames, four groups, zero scene overlap** (verified pairwise on scene-key
sets across all six group pairs; every label paired with an image; 36107/36107 frames accounted for).

| group | scenes | frames | share | `object` | `robot` | `house_bot` | `mr_stabs_mk2` | `mrs_buff_mk3` |
|---|---|---|---|---|---|---|---|---|
| `old` | 20 | 14464 | 40.1 % | 246 | 34166 | 13607 | 502 | **0** |
| `new` | 21 | 14207 | 39.3 % | 0 | 28600 | 8920 | 2173 | 252 |
| `hold_old` | 8 | 3893 | 10.8 % | 0 | 7993 | 3346 | 460 | 0 |
| `hold_new` | 10 | 3543 | 9.8 % | 0 | 6716 | 2344 | 535 | 110 |

Roles: `old` = what the "deployed" base trains on; `new` = footage it has never seen; `hold_old` /
`hold_new` = scene-disjoint probes of each half, for the **retention** and **acquisition** diagnostics.
Ultralytics `val` for every arm = `hold_old ∪ hold_new`, identical across arms, for live
plateau-watching only. Verdicts always come from `nhrl_keypoints_eval_test`.

Emitted alongside the groups: **`old.yml`, `new.yml`, `old+new.yml`, `old_4class.yml`** (train on those groups, validate
on both holdouts) and **`split_manifest.json`** (per-group scene lists, periods, and class counts).
Images are hardlinked, so the tree costs label bytes only.

**The imbalances the calendar imposed — read these before interpreting any arm:**

- **`old` has zero real `mrs_buff_mk3`.** All three our-robot keypoint scenes are 2026-04, the newest
  data in the corpus. `C_old` therefore cannot detect our own robot from real footage at all, and
  `hold_old` cannot probe it either. This is faithful to reality — our robot *is* newer than most of
  the footage — but it means our-robot performance is an **acquisition-only** signal in this
  experiment, never a retention one.
- **`mr_stabs_mk2` is 13.7 % old / 59.2 % new.** Same direction, less extreme.
- **`object` is 100 % in `old`.** The two `Flight Controller` classes live in a single 2024-10 mini-bot
  scene. Since `taxonomy.yaml` excludes `object` from scoring this does not affect verdicts, but a
  model trained on `new` alone has never seen the class.
- **`robot` and `house_bot` came out close to target** (44.1/36.9/10.3/8.7 and 48.2/31.6/11.9/8.3
  against a 40/40/10/10 goal) — the stratified holdout carve inside each half did its job on the two
  classes that carry the agnostic metric.

**Reconcile GT class names** before scoring the holdouts: `nhrl_robots_bbox_real` names class 1 `robot`
while the eval GT names it `opponent`. Remap the holdout labels (`edit_yolo_classes.py`) or add a
`taxonomy.yaml` archetype entry so holdout and eval scoring share one vocabulary.

Pooling synthetic (for the follow-on only) stays in `pool_datasets.py`, sourced from
`synth_bbox_from_keypoints` — never by reaching back into the contaminated `nhrl_robots_bbox`.

### Step 2 — Run 1 (`base4`): the pre-existing-roster model

Cold from stock `yolo26n.pt` (COCO) on **`old`**, **4 classes** — `object, robot, house_bot,
mr_stabs_mk2`, with `mrs_buff_mk3` genuinely absent from the vocabulary, not merely unpopulated.
**150 epochs, fully annealed** over its own budget, `--save-period 25`. Val = `hold_old`.

**No relabeling is required, which is a property of the split rather than luck.** `mrs_buff_mk3` is
the *last* class index, and `old` and `hold_old` both contain **zero** `mrs_buff_mk3` boxes. So the
4-class dataset is the same hardlinked frames under a `data.yml` with `nc: 4` and a truncated `names`
list — no `edit_yolo_classes.py` pass, no dropped annotations, no reindexing. Run 1's vocabulary is a
strict prefix of run 2's, which is exactly what makes the head grow cleanly in Step 3.

Val is `hold_old` alone, **not** `hold_old ∪ hold_new`: `hold_new` carries 110 `mrs_buff_mk3` boxes,
a class index this model does not have. Runs 2 and A use both holdouts.

Output: the best checkpoint by external eval becomes **`C_old4`**, committed to
`data/models/yolo26n_scenesplit_old4_<date>.pt`.

> **The 150-epoch schedule is a bet this step must also settle.** Phase A did *not* find that a
> dedicated annealed short run works — it found the opposite at every length it tested: dedicated
> fully-annealed runs scored 0.710 (e30) and 0.707 / 0.688 / 0.675 (e50 × 3 seeds), all failing the
> gate, while *early-stopped checkpoints of the 500-epoch schedule* reached 0.729–0.765. Its
> `E_cold ≈ 150` was an early-stop point on a long schedule, not a dedicated 150-epoch run. A
> dedicated 150 is genuinely **untested** — plausibly the length where annealing stops hurting, but
> unproven. The `--save-period 25` ladder settles it for free: score ep{25,50,75,100,125,150} and
> check whether the fully-annealed ep150 endpoint beats its own earlier checkpoints. **If the endpoint
> is worse than an earlier one, Phase A's pattern has repeated at 150** and every run here should
> revert to early-stopping a 500-epoch schedule. Record this verdict before running Step 3.

### Step 3 — Run 2 (`warm5`) and Run A (`cold5`)

Both train on **`old+new`** (28671 frames, 41 scenes) with the full **5-class** vocabulary, 150
epochs, `--save-period 25`, val = `hold_old ∪ hold_new`. They differ only in initialization.

| run | init | trains on | classes | what it answers |
|---|---|---|---|---|
| **1 — `base4`** | COCO | `old` | 4 | the deployed model, before the new robot existed |
| **2 — `warm5`** | `C_old4` | `old+new` | 5 | can the category be added by fine-tuning? |
| **A — `cold5`** | COCO | `old+new` | 5 | **control** — what a full retrain achieves |

- Run 2 uses `fine_tune_train.py -c <C_old4> --epochs 150 --save-period 25` (`lr0 0.001`,
  `warmup_bias_lr 0.01`). Run A uses `train.py --epochs 150 --save-period 25` (`lr0 0.01`).
- **Expect a partial head transfer, and record it.** `fine_tune_train.py` does `YOLO(ckpt)` then
  `train(data=…)`; ultralytics builds the model at the *data's* `nc` and loads only shape-matching
  parameters. Backbone, neck, and the box-regression branch transfer; the classification projection
  changes shape (4 → 5 outputs) and is reinitialized. The printed transferred-items count is the
  evidence that the head grew rather than the checkpoint being silently ignored — log it.
- **The LR difference is a confound.** Warm runs at `lr0 0.001`, cold at `0.01`. Run **Run 2 at both**
  to bound it; if the two agree, drop one from future work.

### Step 4 — scoring: two views, and one hard constraint

**Constraint: the 4-class engine cannot share a `score.py` invocation with the 5-class models.**
`score.py` passes a single `--labels` list to every candidate as `num_classes`, and `trt_yolo.py`
*trusts* that number rather than inferring it from the engine (`_infer_raw_head_dims`: "num_classes > 0
is trusted"), so a 5-label list would misparse a 4-class output tensor. Therefore:

- **Invocation A (the load-bearing one):** baseline + run 2 + run A, all 5-class, one paired bootstrap,
  `--labels "object,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3"`. Every warm-vs-cold comparison lives
  here and is properly paired.
- **Invocation B:** run 1 alone, `--labels "object,opponent,house_bot,mr_stabs_mk2"`. Its numbers are
  comparable to the others **descriptively only** — not paired. That is acceptable because run 1's job
  is to produce `C_old4` and establish the starting deficit, neither of which needs a paired CI.

Run both invocations under **two taxonomies**:

| view | taxonomy `exclude` | measures | applies to |
|---|---|---|---|
| **common-class** | `object`, `mrs_buff_mk3` | no-regression on the pre-existing roster | all runs + baseline |
| **new-class** | `object`, `robot`, `house_bot`, `mr_stabs_mk2` | acquisition of the added category | runs 2 and A only |

The common-class view is the only way to compare a 4-class model against 5-class ones fairly: without
it, run 1 takes 389 automatic misses on the eval set's `mrs_buff_mk3` boxes and looks catastrophically
worse for a reason that has nothing to do with its quality. **Re-anchor the baseline under each
taxonomy in Step 0** — excluding `mrs_buff_mk3` changes the reference numbers away from the 0.742 /
0.962 / 0.838 anchor, which was measured with it included.

**Also score every chosen checkpoint on `hold_old` and `hold_new` separately.** `hold_old` is the
retention probe (zero `mrs_buff_mk3`, so it reads as a pure "did the old roster survive" signal);
`hold_new` is acquisition. Reconcile GT class names first — the groups name class 1 `robot` while the
eval GT names it `opponent`.

**Cost in GPU-hours, not epochs.** All three runs are the same length, so what differs is wall-clock
to the first checkpoint clearing the gate. Log per-epoch wall-clock. Anchor: Phase A's 500-epoch run on
49086 frames took 14.6 h, so a 150-epoch run on ~28.7k frames should land near 2.5 h.

### Step 5 — the answer

One table: run × (common-class recall/precision/F1 Δ, `mrs_buff_mk3` AP, hours-to-first-passing-
checkpoint, `hold_old` Δ, `hold_new` Δ). The recommendation is a sentence of the form *"to add a robot
category, do X; it reaches parity in N hours versus M for a cold retrain, and costs the existing
classes Y."*

Outcomes, all publishable:

- **Warm wins** — run 2 matches run A on both views in materially fewer hours → adopt fine-tuning for
  category additions, and commit `C_old4`'s successor as the standing base.
- **Warm learns the class but regresses the old ones** — visible as common-class worse for run 2 than
  run A, and/or a `hold_old` drop. Quantify the replay fraction that fixes it before adopting.
- **Warm underperforms on the new class** — a reinitialized head on a converged backbone may not fit a
  brand-new category in 150 epochs at `lr0 0.001`. Then category *additions* need cold retrains even if
  ordinary data additions do not — a sharp, useful distinction.
- **Tie** — stay cold-start; it is simpler and carries no checkpoint provenance.

## Follow-on — synthetic attribution (optional, secondary)

Phase A's Exp 3 concluded "the extra real data past ~50 % buys precision." That claim was **withdrawn**:
`--fraction` cut real and synthetic together, so the precision loss is unattributed. Now that the
primary arms are real-only on `nhrl_robots_bbox_real`, the axes are cleanly separated and two extra
arms settle it, using the Step 3 recipe on `OLD ∪ NEW`:

- **real + synthetic** — pool `synth_bbox_from_keypoints` in via `pool_datasets.py` and compare against
  the real-only Arm A. This is the presence/absence test that `synthetic_plus_bbox_2026-07-22` was read
  as having run but did not (it added synthetic *on top of* an already-35 %-synthetic substrate, so it
  measured a marginal dose).
  > **Run 2026-07-31** as its own experiment on the audited 2-class corpus — see
  > `synthetic_arms_2026-07-31.md` (real_only / synth_only / mixed, provenance gate reproduced before
  > training). Result: precision +0.047 and F1 +0.028 for mixed, recall flat, pre-registered
  > recall-based adopt criterion not met. This item is covered; do not re-run it here.
- **synthetic-only** — builds the `C_synth` base that Phase A recorded as impossible. If it has
  standalone value, it is the *reusable* base worth committing (synthetic is free to regenerate and
  carries no eval leakage, unlike a base derived from real fights).

Competing hypothesis worth testing directly: precision failures are false-positive opponents, and
100 % of the `object` head plus 19 % of the generic-`robot` boxes are synthetic — so **losing synthetic
supervision is at least as plausible a cause of the precision collapse as losing real.**

Run this only after Step 5 lands. It is a separate question and should not delay the warm-start answer.

## Phases B and C — on hold, and they inherit the corrections

- **Phase B (keypoint pose model, `yolo26n-pose`).** On hold pending this experiment's result. When it
  runs it must (a) audit its substrate first, (b) split `all_robot_keypoints` by scene, (c) rebuild val
  scene-disjoint. Its load-bearing question is unchanged — *how little real our-robot data is needed on
  top of abundant exact-CAD synthetic to hold recall and heading accuracy* — and it adds a mandatory
  second gate: **heading acc @10° delta CI lower bound ≥ −0.03**. Recall can hold while heading rots.
  Reference numbers: agnostic recall 0.852, precision 0.939, `kp_err` 9.6 px, PCK@0.1 0.729, heading
  error 7.6°, heading acc @10° 0.814 (`baseline_2026-07-07` addendum).
- **Phase C (model-scale sweep, `yolo26{n,s,l,x}`).** On hold. Its premise was "fix the Phase A cheap
  recipe as a budget and sweep backbone size." That recipe survives (early-stop the long schedule at
  ~ep100) but is **substrate-specific** — validated on the 65/35 mixed corpus, not a real-only one. If
  the follow-on above changes the corpus, re-measure the recipe before spending Phase C compute.
  Latency is the binding constraint: report accuracy *against* measured Jetson latency, never accuracy
  alone, against the <60 ms end-to-end budget. Prerequisite: `yolo26{s,l,x}` entries in `train.py`'s
  `configs` with VRAM-scaled batch sizes.

## Execution notes (per `experiment_runbook.md`)

- **Where it runs:** training and scoring both on megamind (3× A6000, sm86). `nhrl_keypoints_eval_test`
  is local, so the full train→export→score loop runs here with `_x86_64_sm86` engines. Engines are
  arch-specific; build to match the box you score on. Never score desktop-playback detections against
  live-labeled GT (rectification warp, see `baseline_2026-07-07`).
- **Do not run scoring/inference IO while a training arm is live** — it evicts the page cache and spikes
  epoch time ~30×. Score after arms finish or on an idle GPU.
- **Export per scored checkpoint:** `convert_to_onnx.py` → `convert_to_tensorrt.py --workspace 1`
  (`yolo26n` ~1 GiB). Score a sensible subset (6–8 checkpoints per curve), not every `save_period` dump.
- **Do not modify** `nhrl_robots_indiv`, `nhrl_robots_bbox`, or `all_robot_keypoints` in place.
  `nhrl_robots_indiv` is the only real-only source in the tree and has no upstream to rebuild from —
  treat it as read-only. `split_by_scene.py` hardlinks into new trees under
  `training/data/scenesplit_<date>/`.
- **Train bbox models on `nhrl_robots_bbox_real` from here on.** `nhrl_robots_bbox` is retained only to
  reproduce Phase A and the deployed baseline; it is 34.6 % synthetic and must not be used for new work
  without saying so explicitly in the writeup.
- **score.py discipline:** `--labels` length must equal the engine class count (check the printed
  `num_keypoints=N num_classes=M`); candidates in one run must share class order; always pass
  `--baseline` and `--conf` matching the deployed config (blob 0.6 / keypoint 0.5).

## Risks / caveats

- **Degeneracy is structurally impossible this time, and that is the point.** Phase A's Exp 2 died
  because its warm base had already seen everything it was fine-tuned on. Here run 1 is a **4-class**
  model that does not contain `mrs_buff_mk3` at all, so it cannot start at parity on the new class no
  matter how good it is. The thing being added is guaranteed absent from the base.
- **The 150-epoch annealed schedule is unproven, not established.** Phase A tested dedicated annealed
  runs only at 30 and 50 epochs and both failed the gate; its `E_cold ≈ 150` was an *early-stop point
  on a 500-epoch schedule*, a different model. Run 1's checkpoint ladder tests the assumption directly
  (§Step 2). If the annealed endpoint loses to its own earlier checkpoints, revert every run to
  early-stopping a long schedule before drawing conclusions about warm vs cold.
- **A reinitialized head on a converged backbone may simply not converge at `lr0 0.001`.** Run 2 asks a
  freshly-random class projection to learn a brand-new category while the low fine-tuning LR holds the
  rest nearly still. If run 2 underperforms on `mrs_buff_mk3`, check the LR arm before concluding that
  fine-tuning cannot add categories.
- **The new class is thin: 252 training boxes across 21 scenes in `new`, 110 in `hold_new`.** Both
  paths are learning `mrs_buff_mk3` from very little, so a small measured difference between them may
  be noise on the new-class view specifically. Read `mrs_buff_mk3` AP as directional.
- **"New scenes" are a proxy for "new event."** Held-out recordings from the same corpus share cage,
  lighting, and camera rig with the training half. This understates the difficulty of genuinely new
  footage, so a warm-start win here is an **upper bound** on warm-start value. State it in the writeup.
- **Our-robot supervision could not be balanced across halves, and the temporal split made it
  absolute.** All three real `mrs_buff_mk3` scenes are 2026-04, so `old` and `hold_old` contain none at
  all (§Step 1b). Our-robot performance is therefore an acquisition-only signal here. The split
  stratifies `robot` and `house_bot` — the classes that carry the agnostic gate — and reports the rest.
- **Eval set is small (~372 frames)** → wide parity CIs. δ must exceed the bootstrap CI half-width. Do
  not claim parity inside the noise. Per-class AP is directional only, not bootstrapped.
- **Run-to-run variance is ~0.05 recall** (Phase A: two 500-epoch runs differed by 0.048), and
  checkpoint selection within one run spans 0.765→0.694. Treat any single arm within ~0.05 of another as
  indistinguishable, and prefer differences in *hours-to-parity*, which are much larger than the noise.
- **LR-schedule ≠ epoch-prefix.** Ultralytics anneals LR linearly over the run's *total* budget, so
  `epoch150` of a 500-epoch run is not what a dedicated 150-epoch run produces. Phase A found the
  under-annealed early checkpoint generalizes **better**. Never substitute a dedicated short run for an
  early checkpoint without measuring both.
- **Same-corpus val is proven unsafe.** Phase A's val rose monotonically while the external eval fell —
  two mechanisms: shared scenes and 35 % synthetic content. The scene-disjoint real-only val in Step 1b
  fixes both, but the verdict still comes from `nhrl_keypoints_eval_test`, never val.
- **Same-run comparability.** Absolute AP is comparable only within one `score.py` run under one
  `--labels` mapping. The `--labels` fix in Step 0 means **this plan's absolute numbers are not
  comparable to Phase A's** — only to each other and to the re-anchored baseline.

## Success criteria

- **Primary met** if Step 5 yields a paired comparison of fine-tuning vs cold retraining for a
  *category addition* — hours-to-parity, new-class acquisition, and old-class regression — whichever
  direction it goes.
- **Regression quantified** — whether growing the head from 4 to 5 classes on a converged checkpoint
  costs the pre-existing categories anything, measured on both the eval set (common-class view) and
  `hold_old`.
- **The annealing question settled as a by-product** — run 1's `--save-period 25` ladder says whether a
  dedicated 150-epoch annealed schedule beats its own early checkpoints, closing the gap Phase A left
  between its tested e30/e50 failures and its untested e150.
- **Neutral results are results** — if fine-tuning ties a cold retrain, future category additions stay
  cold-start for simplicity, documented rather than assumed.

## Artifacts / locations

- Instrumentation: `--save-period` / `--fraction` / `--seed` in `training/yolo/train.py` and
  `fine_tune_train.py` (done, Phase A); **new** `training/yolo/split_by_scene.py` (built);
  `training/yolo/pool_datasets.py` (done, Phase A); **regenerated** `remap_config_seg.toml` (the
  committed one is index-stale, §Step 1a).
- **Canonical real-only bbox dataset: `training/data/nhrl_robots_bbox_real/`** — derived from
  `nhrl_robots_indiv` per §Step 1a. This is the substrate all future bbox training uses.
  `nhrl_robots_bbox` (34.6 % synthetic) is superseded for training and kept only for reproducing the
  deployed baseline.
- Scene-split datasets: **`training/data/scenesplit_2026-07-25/{old,new,hold_old,hold_new}/`** plus
  `old.yml` / `new.yml` / `old+new.yml` and `split_manifest.json` (hardlinked; sources unmodified).
- Run 1 base: `data/models/yolo26n_scenesplit_old4_<date>.{pt,onnx,engine}` (`C_old4`, 4-class).
- Run models: `data/models/yolo26n_scenesplit_{warm5,cold5}_<date>.{pt,onnx,engine}`
  (`_x86_64_sm86`, 5-class).
- Dataset yamls: `scenesplit_2026-07-25/{old_4class,old+new}.yml`; taxonomies
  `taxonomy_common_class.yaml` / `taxonomy_new_class.yaml` under `training/model_eval/`.
- Scores: `training/data/nhrl_keypoints_eval_test/scores_scenesplit_<view>/{summary.csv,
  significance.csv, headline.png, confusion_*.png}` — one dir per taxonomy view, plus the
  separate 4-class invocation; holdout scores alongside.
- Writeup: `docs/experiments/perception_performance/category_addition_<date>.md`.
- Superseded: `data_epoch_min_phaseA_2026-07-24.md` (Exp 1 results stand; Exp 2 invalid, Exp 3
  re-scoped to a corpus floor).
