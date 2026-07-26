# Merge our robots into the generic robot class (Option B) — experiment plan

Status: **Step 0 confirmed 2026-07-26** (playback verified: the pipeline works with our robots set to
opponent in the blob/mask model). **Step 1 revised 2026-07-26** to rebuild the dataset from archived
sources. Writeup goes in `merged_robot_class_<date>.md` once scored.

## Question

**Should the bbox model detect our own robots as generic robots rather than as their own classes,
leaving instance identity entirely to the keypoint model?**

`category_addition_2026-07-25` measured what the current 5-class vocabulary buys: **nothing**.

| | value |
|---|---|
| `mrs_buff_mk3` recall on the external eval, every 5-class arm | **0.000** |
| instance wrong-class rate, 5-class arms | 0.35–0.39 |
| instance wrong-class rate, deployed baseline | 0.114 |
| `mrs_buff_mk3` share of training boxes | 252 / 63,018 = **0.4 %** |

Same failure `indiv_blob_2026-07-09` and `nhrl_robots_7class_2026-07-13` documented — imbalance,
softmax dilution, no generalization — at 0.4 % representation.

`ParallelModelBatch` (`src/perception_batch/parallel_model_batch.cpp:41-47`) runs the keypoint and blob
models **in parallel on the full frame**; the keypoint model does not consume bbox crops. So the bbox
model never needed to identify our robot for the keypoint path to work.

## Step 0 — pipeline safety — **CONFIRMED**

Playback verified that setting our robots to opponent in the blob/mask model leaves the pipeline
working, with identity supplied by the keypoint model.

Two items still worth doing before deploying a merged model, neither blocking training:

1. **Change `group_for_label()`'s fallback from `THEIRS` to `NEUTRAL`**
   (`include/robot_filter/label_group_utils.hpp:9`). It currently defaults *unknown* labels to
   `Group::THEIRS`, i.e. shootable. A labelling gap should fail closed.
2. **Record the keypoint model's per-frame miss rate on our robot** during that replay. That miss rate
   is the residual risk `our_robot_hold_window_s` has to absorb, and it is the number that decides
   whether the merge is safe in a match, not the detector metrics below.

## Step 1 — rebuild the dataset from the archived indiv source

**Single source: `/media/storage/auto-battlebots-archive/nhrl_seg_indiv_labeled`.** Sourced from the
archive rather than `nhrl_robots_indiv` to guarantee provenance; audited 2026-07-26 and confirmed
**zero synthetic content**.

56 per-scene segmentation datasets, **36,012 frames, 199,119 polygons**.

After the Option B name-based remap:

| target | polygons | from |
|---|---|---|
| `object` (0) | **246** | `Flight Controller 1` (135), `Flight Controller 2` (111) |
| `robot` (1) | **81,732** | all competitors, incl. `Mrs Buff MK2` (2,327) and `Mr Stabs MK2` (3,563) |
| `house_bot` (2) | **28,318** | `House Bot` (25,093) + Orange (1,717) + White (1,508) Halloween |
| _dropped_ | _88,823_ | `Floor` |
| **kept** | **110,296** | |

> **No keypoint data is merged in.** Deliberate, and safe under Option B: the bbox model no longer
> needs to identify our robot, so real MK3 footage is not required for it. The corpus still contains
> 2,327 boxes of `Mrs Buff MK2` — a visually similar predecessor — plus 3,563 of `Mr Stabs MK2`, all as
> generic `robot`. `category_addition_2026-07-25` showed a model with **zero** `mrs_buff_mk3`
> supervision still reached **0.780 agnostic recall** on the eval: it detects our robot fine, it just
> could not name it. Naming is the keypoint model's job now.

**Three things make this harder than remapping `nhrl_robots_indiv`, and all three will silently corrupt
the dataset if missed:**

1. **The scenes do not share a class vocabulary.** There are **two** distinct `data.yml` class lists —
   105 names (50 scenes) and 111 names (6 scenes) — and the indices *do not line up*:

   | class | 105-class scenes | 111-class scenes |
   |---|---|---|
   | `Floor` | 36 | 39 |
   | `House Bot` | 41 | 44 |
   | `Mrs Buff MK2` | 67 | 70 |

   A single index map therefore **cannot** be used. `remap_labels.py` takes one `[label_map]` and is
   the wrong tool here. Remap **by name, per scene**, reading each scene's own `data.yml`.

2. **`Floor` is 88,823 polygons — 45 % of all annotations — and is not a robot.** It belongs to the
   field/deeplab task. Dropping it leaves ~110,296 robot polygons, consistent with the 109,523 in
   `nhrl_robots_indiv`. Drop `Floor` and `Floor letter word`.

3. **There is no `Mrs Buff MK3` class anywhere in the archive.** Our robot appears in this footage only
   as `Mrs Buff MK2` (2,327 boxes) — independently confirming the finding in
   `category_addition_2026-07-25` that every real `mrs_buff_mk3` box in the old lineage was a
   relabelled MK2. Under Option B this stops mattering: MK2 is a competitor robot and maps to `robot`
   like everything else.

**New helper required: `training/yolo/remap_labels_by_name.py`.** Reads each scene's `data.yml`, maps
source class *names* to the target vocabulary, drops unmapped classes, and writes one merged dataset.
Must fail loudly on any name it has no rule for, rather than defaulting — that is how `Floor` would
otherwise slip in.

Target rules (Option B):

| source names | target |
|---|---|
| `Floor`, `Floor letter word` | **dropped** |
| `House Bot*` (all Halloween variants) | `house_bot` (2) |
| `Flight Controller 1`, `Flight Controller 2` | `object` (0) |
| everything else, **including `Mrs Buff MK2`, `Mr Stabs MK2`, `Stab Rave`, `power on`, `usawgi`** | `robot` (1) |

### Build sequence

```
remap_labels_by_name.py  (per-scene, name-based)  -> 3-class real seg
seg_to_bbox.py                                    -> 3-class real bbox
split_by_scene.py --mode temporal --cutoff 2025-11 --holdout-frac 0.2 \
                  --stratify-class robot,house_bot
validate_yolo_integrity.py --strict
```

**Assertions before training** — each has already caught a real defect once:

- no label file contains a class id > 2, and **zero frames named `synthetic*`**
- **36,012 frames**, **110,296 polygons** kept after dropping `Floor`
- **`house_bot` = 28,318.** This is the load-bearing check that the name-based remap landed correctly,
  because `house_bot` is the one class that is real-only, well-populated, and untouched by the merge.
  Note it is **not** the 28,217 seen in `nhrl_robots_indiv` — the archive carries 267 more frames, so a
  mismatch against the old number is expected and 28,318 is the correct target.
- `robot` = 81,732, `object` = 246
- zero scene overlap across the four split groups

## Step 2 — arms

**The rebuild invalidates the existing `cold5` as a control.** It trained on `scenesplit_2026-07-25`,
derived from `nhrl_robots_bbox_real`; the new split comes from a different source with different frame
counts. Both arms below must therefore be trained fresh on the new data.

**The control is 4-class, not 5.** With the labels corrected, `mrs_buff_mk3` has **zero** boxes in this
corpus — the archive contains no MK3 class, and the 2,327 boxes that used to populate it were
`Mrs Buff MK2` mislabelled. A 5-class arm would therefore carry a permanently empty head, which is not
a control but a degenerate model. The only reproducible non-merged vocabulary is 4-class, keeping the
one our-robot class that has real data:

| arm | vocab | classes | purpose |
|---|---|---|---|
| **`cold4`** | 4-class | `object`, `robot`, `house_bot`, `mr_stabs_mk2` (3,563) | control — our-robot class retained where data exists |
| **`cold3`** | 3-class | `object`, `robot`, `house_bot` | treatment — all our robots merged into `robot` |

Cold from COCO, 150 epochs, `--save-period 25`, val `hold_old ∪ hold_new`, current `train.py` defaults
(`lrf` 0.1, `flipud` 0.0 as of `a4ff7dc`). ~2.8 h each.

**What this comparison can and cannot show.** The eval set has **zero `mr_stabs_mk2` GT instances**, so
this does not test whether that class is detected well. What it does test is the softmax-dilution
question directly: does carrying a 4.3 %-representation our-robot head *cost* the generic `robot` class
anything? That is the mechanism `indiv_blob` and `7class` blamed, and it is measurable here on
agnostic recall/precision.

**Comparison against `category_addition_2026-07-25`'s arms is confounded** by the dataset rebuild and is
indicative only. The `train.py` default change (`a4ff7dc`) cannot be cleanly measured here and should
not be claimed from this experiment.

## Step 3 — scoring

**A 3-class engine cannot share a `score.py` invocation with 5-class ones.** `score.py` passes one
`--labels` list to every candidate as `num_classes` and `trt_yolo.py`'s `_infer_raw_head_dims` trusts
it rather than reading the engine. Score `cold3` separately with `--labels "object,opponent,house_bot"`.

**A merged taxonomy is required or the comparison is rigged.** A 3-class model calling our robot
`opponent` is correct under its own vocabulary but reads as a wrong-class error under `taxonomy.yaml`.
This is the same trap as the common-class view abandoned in `category_addition_2026-07-25`. Add
`training/model_eval/taxonomy_merged.yaml`:

```yaml
archetypes:
  opponent: opponent
  house_bot: house_bot
  mr_stabs_mk2: opponent     # our robots are competitor robots at this level
  mrs_buff_mk3: opponent     # eval GT still carries this; maps to the merged target
exclude:
  - object
```

| view | taxonomy | measures |
|---|---|---|
| agnostic | `taxonomy.yaml` | "did it find the robots" — vocabulary-neutral, the primary gate |
| merged instance | `taxonomy_merged.yaml` | naming quality on the vocabulary `cold3` actually targets |

**Gate:** earliest ladder checkpoint whose agnostic recall Δ CI lower bound ≥ −0.04 with precision and
F1 not significantly worse, paired vs the deployed baseline on `nhrl_keypoints_eval_test`. Baseline
anchor under `taxonomy.yaml`: recall 0.742 / precision 0.962 / F1 0.838. Re-anchor under
`taxonomy_merged.yaml` before scoring arms.

Also score both arms on `hold_old_scoreable` / `hold_new_scoreable` (numeric-stemmed shims —
`score.py` requires integer `stamp_ns` filenames).

## Metrics

- **Primary:** agnostic recall / precision / F1 vs baseline, δ = 0.04.
- **The merge's own hypothesis:** wrong-class rate under `taxonomy_merged.yaml`. Removing a class the
  model could not predict should drop this well below the 5-class arms' 0.35–0.39.
- **`house_bot` AP** — the one remaining non-generic class. Merging should not disturb it.
- **Latency** on the Jetson. A smaller head is marginally cheaper; free to record, feeds the <60 ms budget.

## Success criteria

- **Merge adopted** if `cold3` is non-inferior to `cold4` on agnostic recall and precision **and**
  improves merged-taxonomy wrong-class rate.
- **Merge rejected** if agnostic recall or precision degrades significantly. Simplification is not
  worth a detection regression.
- **A tie still favours merging:** fewer classes, no dead head, identity in the model that can learn
  it, one less way for the pipeline to be wrong.

## Risks / caveats

- **Safety is the real risk, not metrics.** Step 0's playback confirms the pipeline *functions*; it
  does not establish how often the keypoint model misses our robot. Get that number (Step 0.2) before
  deploying, and make the `group_for_label` fallback `NEUTRAL` first.
- **`mr_stabs_mk2` detection cannot be measured.** The eval set has zero `mr_stabs_mk2` GT instances.
  The arms measure whether keeping that class *costs* the generic `robot` class, not whether the class
  itself works. Do not claim the latter was tested.
- **`mrs_buff_mk3` is entirely absent from this corpus** once MK2 is labelled correctly. Nothing here
  speaks to how well our current robot is detected — only that a model without any MK3 supervision
  still reached 0.780 agnostic recall in the previous experiment.
- **Per-scene vocabularies are the biggest build hazard.** 50 scenes at 105 classes and 6 at 111, with
  shifted indices. An index-based remap will look like it worked and produce a silently mislabelled
  corpus — exactly how `remap_config_seg.toml` went stale and how `Mrs Buff MK2` came to be labelled as
  our robot in the first place. Remap by name, fail loudly on unknown names.
- **`Floor` is 45 % of the archive's annotations.** Including it would swamp the detector with a
  non-robot class.
- **The dataset rebuild breaks continuity with all prior arms.** Numbers here are comparable to each
  other and to the deployed baseline, not to `category_addition_2026-07-25`'s arms.
- **One seed per arm**, and δ = 0.04 is tighter than the 0.048 cross-run spread Phase A measured. Read
  the crossing epoch as ±1 ladder step.
- **The domain gap is untouched.** Training is NHRL overhead cage footage; the eval is the robot's own
  camera. This experiment does not address that, and the late-epoch collapse may recur.
- **Do not read the 5-class arms' 0.000 as proof the merge is *needed*.** It shows the current class is
  useless *on the deployment camera*, where all our-robot training data is garage-rig footage. A
  5-class model with in-domain our-robot data might work; this plan does not test that.

## Artifacts / locations

- Source: `/media/storage/auto-battlebots-archive/nhrl_seg_indiv_labeled` (56 scenes, 36,012 frames,
  real-only). No keypoint data is used.
- New helper: `training/yolo/remap_labels_by_name.py`
- Dataset: `training/data/nhrl_robots_bbox_3class/` → `training/data/scenesplit_3class_<date>/{old,new,hold_old,hold_new}/`
- Taxonomy: `training/model_eval/taxonomy_merged.yaml`
- Models: `data/models/yolo26n_merged3_<date>.{pt,onnx,_x86_64_sm86.engine}`
- Scores: `training/data/nhrl_keypoints_eval_test/scores_merged3_{agnostic,merged}/`
- Pipeline change: `include/robot_filter/label_group_utils.hpp` (fallback → `NEUTRAL`),
  `config/_desktop.toml` / `_jetson.toml` (`label_indices` → `["OBJECT","OPPONENT","HOUSE_BOT"]`)
- Writeup: `docs/experiments/perception_performance/merged_robot_class_<date>.md`
