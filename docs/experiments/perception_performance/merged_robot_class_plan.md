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

After the Option B name-based remap — **two classes, not three**:

**BUILT 2026-07-26** — `training/data/nhrl_robots_bbox_2class`, **35,750 frames, 107,615 boxes**,
`validate_yolo_integrity.py --strict`: 0 errors, 0 warnings. Images are real copies (2,000 sampled:
`nlink` 1, no symlinks), so the tree is standalone and survives the source being archived.

| target | boxes | from |
|---|---|---|
| `robot` (0) | **79,585** | every competitor, our own machines (`Mrs Buff MK2`, `Mr Stabs MK2`), and the two `Flight Controller` objects |
| `house_bot` (1) | **28,030** | `House Bot` + Orange/White Halloween liveries |
| _dropped_ | _88,823_ | `Floor` (field segmentation, belongs to deeplab) |

> **`object` is gone as a class, deliberately.** It held **246 boxes from a single 2024 mini-bot
> scene** — 0.2 % of annotations, far too few to learn, and the same class-imbalance failure
> `indiv_blob` and `7class` documented. More to the point, the distinction it encoded is not one the
> detector is asked to make.
>
> **`robot` now means "anything in the field that is not the house bot."** That is the honest
> description of what this head does and what the downstream consumer needs: the filter wants to know
> where the things in the arena are, the keypoint model says which one is ours, and `house_bot` stays
> separate only because it is neutral and must never be targeted. Naming the class for what it
> actually is beats carrying two heads that encode distinctions the model cannot make and the
> pipeline does not use.

> **No keypoint data is merged in.** Deliberate, and safe under Option B: the bbox model no longer
> needs to identify our robot, so real MK3 footage is not required for it. The corpus still contains
> 2,327 boxes of `Mrs Buff MK2` — a visually similar predecessor — plus 3,563 of `Mr Stabs MK2`, all as
> generic `robot`. `category_addition_2026-07-25` showed a model with **zero** `mrs_buff_mk3`
> supervision still reached **0.780 agnostic recall** on the eval: it detects our robot fine, it just
> could not name it. Naming is the keypoint model's job now.

**Five things make this harder than remapping `nhrl_robots_indiv`. All five silently corrupt the
dataset if missed, and two of them were only caught by spot-checking frames after the first build:**

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

4. **Each scene carries a human review verdict that must be honored.** `validation_state.json`
   (written by `validate_yolo_dataset.py`) marks **1,680 frames `fail`** across the 56 scenes. The
   first build ignored it and pulled in every one of them. `--require-pass` keeps only `pass` frames;
   262 of the failed frames had labels and are now excluded, leaving exactly the 35,750 that passed.

5. **Segmentation masks shatter, and a per-polygon box conversion turns each sliver into a target.**
   Spot-checking the highest-box-count frames found one with **277 separate `Snowdrift` polygons** —
   a single robot whose mask fragmented — rendering as ~270 tiny boxes strewn across the crowd and
   venue while the actual robots sat unboxed. The frame is marked `pass` because the *segmentation*
   overlay looks correct; only the box conversion is wrong. `--min-box-side 0.0125` (8 px at 640)
   suppresses the slivers.

   **This affects `seg_to_bbox.py` generally**, and therefore every bbox dataset in the previous
   lineage including `nhrl_robots_bbox_real` and the `category_addition_2026-07-25` arms. The scale
   was small there — 18 frames of 36,012 held >10 boxes, 0.7 % of all boxes — so it does not
   invalidate those results, but it is a standing defect in that converter worth fixing.

**New helper required: `training/yolo/remap_labels_by_name.py`.** Reads each scene's `data.yml`, maps
source class *names* to the target vocabulary, drops unmapped classes, and writes one merged dataset.
Must fail loudly on any name it has no rule for, rather than defaulting — that is how `Floor` would
otherwise slip in.

Target rules (Option B):

| source names | target |
|---|---|
| `Floor`, `Floor letter word` | **dropped** (88,823 polygons of field segmentation) |
| `House Bot*` (all Halloween variants) | `house_bot` (1) |
| everything else — **all 103 competitor names, plus `Mrs Buff MK2`, `Mr Stabs MK2`, `Flight Controller 1/2`** | `robot` (0) |

All 103 defaulted names were reviewed and are competitor robots; the tool prints every name that hits
the default so a new non-robot class appearing upstream is visible rather than silently folded in.

### Build sequence

```
remap_labels_by_name.py --to-bbox --require-pass --min-box-side 0.0125
    -> training/data/nhrl_robots_bbox_2class  (standalone: no hardlinks, no symlinks)
split_by_scene.py --mode temporal --cutoff 2025-11 --holdout-frac 0.2 \
                  --stratify-class robot,house_bot
validate_yolo_integrity.py --strict
```

**Assertions before training** — each has already caught a real defect once:

- no label file contains a class id > 1, and **zero frames named `synthetic*`**
- **35,750 frames** — exactly the `pass` count in `validation_state.json`
- **`robot` = 79,585, `house_bot` = 28,030, total 107,615**
- images are real copies: sampled `nlink == 1`, no symlinks
- zero scene overlap across the four split groups (after `split_by_scene.py`)

Counts differ from the unfiltered build (36,012 frames / 110,273 boxes) by exactly the review-failed
frames and the sub-8px slivers. If a rebuild reports 36,012, one of the two filters was not applied.

## Step 2 — arms

**The rebuild invalidates every prior arm as a control.** They trained on `scenesplit_2026-07-25`,
derived from `nhrl_robots_bbox_real`; the new corpus comes from a different source with a different
vocabulary. Both arms below are trained fresh on the new data.

**The control is 4-class, not 5.** With the labels corrected, `mrs_buff_mk3` has **zero** boxes in this
corpus — the archive contains no MK3 class, and the 2,327 boxes that used to populate it were
`Mrs Buff MK2` mislabelled. A 5-class arm would therefore carry a permanently empty head, which is not
a control but a degenerate model. The only reproducible non-merged vocabulary is 4-class, keeping the
one our-robot class that has real data:

| arm | vocab | classes | purpose |
|---|---|---|---|
| **`cold3`** | 3-class | `robot`, `house_bot`, `mr_stabs_mk2` (3,563) | control — our-robot class retained where data exists |
| **`cold2`** | 2-class | `robot`, `house_bot` | treatment — everything but the house bot is `robot` |

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

**A 2-class engine cannot share a `score.py` invocation with a 3-class one.** `score.py` passes one
`--labels` list to every candidate as `num_classes` and `trt_yolo.py`'s `_infer_raw_head_dims` trusts
it rather than reading the engine. Score `cold2` separately with `--labels "opponent,house_bot"`.

**A merged taxonomy is required or the comparison is rigged.** A 2-class model calling our robot
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
  - object          # eval GT still declares it; no model here predicts it
```

| view | taxonomy | measures |
|---|---|---|
| agnostic | `taxonomy.yaml` | "did it find the robots" — vocabulary-neutral, the primary gate |
| merged instance | `taxonomy_merged.yaml` | naming quality on the vocabulary `cold2` actually targets |

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

- **Merge adopted** if `cold2` is non-inferior to `cold3` on agnostic recall and precision **and**
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
- New helper: `training/yolo/remap_labels_by_name.py` + `remap_config_2class.toml`
- Dataset: `training/data/nhrl_robots_bbox_2class/` (standalone copies, no links) → `training/data/scenesplit_3class_<date>/{old,new,hold_old,hold_new}/`
- Taxonomy: `training/model_eval/taxonomy_merged.yaml`
- Models: `data/models/yolo26n_merged3_<date>.{pt,onnx,_x86_64_sm86.engine}`
- Scores: `training/data/nhrl_keypoints_eval_test/scores_merged3_{agnostic,merged}/`
- Pipeline change: `include/robot_filter/label_group_utils.hpp` (fallback → `NEUTRAL`),
  `config/_desktop.toml` / `_jetson.toml` (`label_indices` → `["OPPONENT","HOUSE_BOT"]`)
- Writeup: `docs/experiments/perception_performance/merged_robot_class_<date>.md`
