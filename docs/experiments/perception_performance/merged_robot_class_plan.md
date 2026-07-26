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

## Step 1 — rebuild the dataset from archived sources

Sourcing from the archive rather than `nhrl_robots_indiv` to guarantee provenance. Both sources
audited 2026-07-26 and confirmed **zero synthetic content**.

### Source A — `/media/storage/auto-battlebots-archive/nhrl_seg_indiv_labeled`

56 per-scene segmentation datasets, **36,012 frames, 199,119 polygons**, no synthetic anywhere.

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

### Source B — `training/data/mrs_buff_mk3_keypoints/*.zip`

Two archives, **497 real frames / 605 boxes total**, no synthetic:

| zip | frames | classes | note |
|---|---|---|---|
| `mrs-buff-mk3-keypoints.zip` | 362 | `[mr_stabs_mk2, mrs_buff_mk3]` | 108 + 362 boxes |
| `mrs-buff-mk3-keypoints-part-2.zip` | 135 | `['auto-battlebots', mrs_buff_mk3]` | 135 boxes; **class 0 is a Roboflow workspace name, declared but unused** |

Labels are pose format (11 fields = box + 2 keypoints); convert with `pose_to_bbox.py`. Index 1 is
`mrs_buff_mk3` in both, but index 0 differs, so map by name here too. Under Option B both our robots
fold into `robot`, so both classes in both zips → class 1.

> **Open question, flagged rather than decided.** Under Option B these 497 frames no longer carry the
> thing they were added for. Their purpose in `category_addition` was our-robot *identity*, which the
> keypoint model now owns. What remains is 497 frames (1.4 % of the corpus) of a robot on a plywood
> test rig in a garage — a setting matching neither the training footage (NHRL overhead cage) nor the
> eval (robot camera in a cage). They may add useful robot-appearance variety or may just add
> off-domain background.
>
> **Build them as their own group** (`kp/`) rather than folding them into `old`/`new`, so
> `old+new_3class.yml` and `old+new+kp_3class.yml` differ by a config line. That makes the ablation one
> optional arm instead of a rebuild.

### Build sequence

```
remap_labels_by_name.py  (per-scene, name-based)  -> 3-class real seg
seg_to_bbox.py                                    -> 3-class real bbox
pose_to_bbox.py  (keypoint zips, --class-map 0:1,1:1) -> kp/ group
split_by_scene.py --mode temporal --cutoff 2025-11 --holdout-frac 0.2 \
                  --stratify-class robot,house_bot
validate_yolo_integrity.py --strict
```

**Assertions before training** — each has already caught a real defect once:

- no label file contains a class id > 2
- total polygons after dropping `Floor` ≈ 110,296 (±the 605 keypoint boxes)
- `house_bot` total is **28,217** — the class that was real-only in every prior lineage and reconciled
  exactly through the last rebuild, so it is the check that the remap did not shift
- zero scene overlap across the four split groups

## Step 2 — arms

**The rebuild invalidates the existing `cold5` as a control.** It trained on `scenesplit_2026-07-25`,
derived from `nhrl_robots_bbox_real`; the new split comes from a different source with different frame
counts. Both arms below must therefore be trained fresh on the new data.

| arm | vocab | trains on | purpose |
|---|---|---|---|
| **`cold5_new`** | 5-class | new split, `old+new` | control — current vocabulary on the new corpus |
| **`cold3`** | 3-class | new split, `old+new` | treatment — our robots merged into `robot` |
| _`cold3_kp`_ | 3-class | + `kp/` group | _optional_, answers the flagged question above |

Cold from COCO, 150 epochs, `--save-period 25`, val `hold_old ∪ hold_new`, current `train.py` defaults
(`lrf` 0.1, `flipud` 0.0 as of `a4ff7dc`). ~2.8 h each.

`cold5_new` vs `cold3` isolates the vocabulary merge with everything else held fixed. **Comparison
against the old `cold5` is confounded by the dataset rebuild and is indicative only** — the `train.py`
default change cannot be cleanly measured here, and should not be claimed from this experiment.

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
  mrs_buff_mk3: opponent
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

- **Merge adopted** if `cold3` is non-inferior to `cold5_new` on agnostic recall and precision **and**
  improves merged-taxonomy wrong-class rate.
- **Merge rejected** if agnostic recall or precision degrades significantly. Simplification is not
  worth a detection regression.
- **A tie still favours merging:** fewer classes, no dead head, identity in the model that can learn
  it, one less way for the pipeline to be wrong.

## Risks / caveats

- **Safety is the real risk, not metrics.** Step 0's playback confirms the pipeline *functions*; it
  does not establish how often the keypoint model misses our robot. Get that number (Step 0.2) before
  deploying, and make the `group_for_label` fallback `NEUTRAL` first.
- **`mr_stabs_mk2` cannot be measured.** The eval set has zero `mr_stabs_mk2` GT instances, so its
  merge rests on the architectural argument alone. Do not claim it was tested.
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

- Sources: `/media/storage/auto-battlebots-archive/nhrl_seg_indiv_labeled` (56 scenes, real),
  `training/data/mrs_buff_mk3_keypoints/*.zip` (497 frames, real)
- New helper: `training/yolo/remap_labels_by_name.py`
- Dataset: `training/data/nhrl_robots_bbox_3class/` → `training/data/scenesplit_3class_<date>/{old,new,hold_old,hold_new,kp}/`
- Taxonomy: `training/model_eval/taxonomy_merged.yaml`
- Models: `data/models/yolo26n_merged3_<date>.{pt,onnx,_x86_64_sm86.engine}`
- Scores: `training/data/nhrl_keypoints_eval_test/scores_merged3_{agnostic,merged}/`
- Pipeline change: `include/robot_filter/label_group_utils.hpp` (fallback → `NEUTRAL`),
  `config/_desktop.toml` / `_jetson.toml` (`label_indices` → `["OBJECT","OPPONENT","HOUSE_BOT"]`)
- Writeup: `docs/experiments/perception_performance/merged_robot_class_<date>.md`
