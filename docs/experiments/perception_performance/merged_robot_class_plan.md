# Merge our robots into the generic robot class (Option B) — experiment plan

Status: **planned** (2026-07-26). Follows `experiment_runbook.md` (prepare → train → export → score).
Writeup goes in `merged_robot_class_<date>.md` once scored.

## Question

**Should the bbox model detect our own robots as generic robots rather than as their own classes,
leaving instance identity entirely to the keypoint model?**

Today the bbox model carries five classes and tries to tell our robots apart from opponents.
`category_addition_2026-07-25` measured what that buys: **nothing**.

| | value |
|---|---|
| `mrs_buff_mk3` recall on the external eval, every 5-class arm | **0.000** |
| instance wrong-class rate, 5-class arms | 0.35–0.39 |
| instance wrong-class rate, deployed baseline | 0.114 |
| `mrs_buff_mk3` share of training boxes | 252 / 63,018 = **0.4 %** |

The class never fires on the deployment camera and coincides with a tripled wrong-class rate. This is
the same failure `indiv_blob_2026-07-09` and `nhrl_robots_7class_2026-07-13` already documented —
severe class imbalance, softmax dilution, no generalization — reproduced at 0.4 % representation.

**The architecture already supports moving identity elsewhere.** `ParallelModelBatch`
(`src/perception_batch/parallel_model_batch.cpp:41-47`) runs the keypoint model and the blob model
**in parallel on the full frame**; the keypoint model does not consume bbox crops. So the bbox model
does not need to identify our robot for the keypoint path to work, and the keypoint model already
emits `MR_STABS_MK2` / `MRS_BUFF_MK3` (`config/_common.toml:35`) which
`robot_keypoint_tracker.cpp:88` resolves to `Group::OURS`.

This matches the standing takeaway: *the keypoints model learns specific robot appearance; the bbox
model generalizes all NHRL robots.* Our robot **is** a robot in arena context — the thing the bbox
model is for.

## What is deliberately not being done

**Our robots are not removed from the labels — they are relabelled.** Dropping them would leave a
visible robot unannotated, teaching false negatives on a robot-shaped object in arena context. That
is the exact defect that made `our_robot_keypoints`' 34,946 renders unusable in
`category_addition_2026-07-25`, and it would corrupt the very cue the opponent detector relies on.

**`house_bot` stays its own class.** 28,217 boxes, well represented, functionally distinct (neutral,
never a target). Nothing in the evidence argues for merging it.

**"Opponent" stops being a predicted class and becomes a downstream decision.** The detector answers
"is there a competitor robot here"; the filter answers "which one is ours" from the keypoint model and
subtracts it. This is cleaner than asking one head to encode a not-us distinction it demonstrably
cannot make at 0.4 % representation — especially given `synthetic_plus_bbox_2026-07-22` showed the
opponent concept is already partly negative-space ("not us, not the house bot").

## Vocabulary

| idx | 5-class (today) | 3-class (proposed) |
|---|---|---|
| 0 | `object` | `object` |
| 1 | `robot` | `robot` ← **+ `mr_stabs_mk2` + `mrs_buff_mk3`** |
| 2 | `house_bot` | `house_bot` |
| 3 | `mr_stabs_mk2` | — |
| 4 | `mrs_buff_mk3` | — |

Merged box counts, from `scenesplit_2026-07-25/split_manifest.json`:

| group | frames | `object` | `robot` | `house_bot` |
|---|---|---|---|---|
| `old` | 14464 | 246 | 34668 | 13607 |
| `new` | 14207 | 0 | 31025 | 8920 |
| `hold_old` | 3893 | 0 | 8453 | 3346 |
| `hold_new` | 3543 | 0 | 7361 | 2344 |
| **total** | 36107 | 246 | **81507** | 28217 |

2927 of our-robot boxes fold into `robot` across the two training groups. `object` keeps its 246
`Flight Controller` boxes and stays excluded at scoring, so class order 0–2 is a strict prefix of the
current vocabulary — the same additive property that made the 4→5 growth clean.

## Step 0 — pipeline safety, before any training

**This gates the whole experiment.** `group_for_label()`
(`include/robot_filter/label_group_utils.hpp:9`) defaults unknown labels to **`Group::THEIRS`**, and
`config/_desktop.toml` maps bbox class 4 → `MRS_BUFF_MK3` → `Group::OURS`. Merge the class without
changing that and our own robot arrives labelled `OPPONENT`, resolves to `THEIRS`, and becomes a
targetable candidate.

1. **Confirm the keypoint path alone is sufficient.** Replay an SVO where our robot is in frame with
   the bbox model's `label_indices` temporarily reduced to `["OBJECT", "OPPONENT", "HOUSE_BOT"]`, and
   verify the filter still tags our robot `OURS` on every frame it is visible:
   `./scripts/build_and_run.sh -c config/playback.toml`. **If identity ever falls back to the blob
   label, stop** — the merge is unsafe until the keypoint path covers it.
2. **Change the fallback from `THEIRS` to `NEUTRAL`** so a labelling gap fails closed (not a target)
   rather than open (shootable). This is worth doing regardless of the experiment's outcome.
3. Record how often the keypoint model misses our robot per frame across the replay. That miss rate is
   the residual risk the `our_robot_hold_window_s` filter has to absorb.

## Step 1 — build the merged dataset

`remap_labels.py` rewrites ids in place; `nc` and `names` come from the data.yml.

```bash
cat > training/yolo/remap_config_merge_our_robots.toml <<'EOF'
# 5-class nhrl_robots_bbox vocabulary -> 3-class: our robots fold into generic `robot`.
[label_map]
0 = 0  # object     -> object
1 = 1  # robot      -> robot
2 = 2  # house_bot  -> house_bot
3 = 1  # mr_stabs_mk2  -> robot
4 = 1  # mrs_buff_mk3  -> robot
EOF
```

Build a 3-class copy of each scene group by hardlink, then remap, so `scenesplit_2026-07-25` is left
untouched. Emit `old+new_3class.yml` with `nc: 3`, `names: [object, robot, house_bot]`,
train `old`+`new`, val `hold_old`+`hold_new`.

Assert after remapping that no label file contains a class id > 2.

## Step 2 — arms

Three comparisons come out of two new runs plus one already trained.

| arm | vocab | train.py defaults | purpose |
|---|---|---|---|
| `cold5` (**exists**) | 5-class | old (`lrf` 0.01, `flipud` 0.5) | the arm scored in `category_addition_2026-07-25` |
| **`cold5_new`** | 5-class | new (`lrf` 0.1, `flipud` 0.0) | isolates the `train.py` default change |
| **`cold3`** | 3-class | new | isolates the vocabulary merge |

Both new arms: cold from COCO, `old+new`, 150 epochs, `--save-period 25`, val `hold_old ∪ hold_new`.
~2.8 h each.

- `cold5` vs `cold5_new` → did `lrf 0.1` / `flipud 0.0` help, and does the ep150 endpoint stop failing
  the gate?
- `cold5_new` vs `cold3` → did merging help, with everything else held fixed?

Running `cold5_new` is what keeps the merge comparison clean; without it the two changes are
confounded.

## Step 3 — scoring

**A 3-class engine cannot share a `score.py` invocation with 5-class ones.** `score.py` passes one
`--labels` list to every candidate as `num_classes` and `trt_yolo.py`'s `_infer_raw_head_dims` trusts
it rather than reading the engine. Score `cold3` separately with
`--labels "object,opponent,house_bot"`.

**A new taxonomy is required for a fair instance-level comparison.** A 3-class model calls our robot
`opponent`, which is correct under the merged vocabulary but reads as a wrong-class error under
`taxonomy.yaml`. Add `training/model_eval/taxonomy_merged.yaml`, mapping our robots to `opponent` in
GT so both vocabularies are scored against the same target:

```yaml
archetypes:
  opponent: opponent
  house_bot: house_bot
  mr_stabs_mk2: opponent     # our robots are competitor robots at this level
  mrs_buff_mk3: opponent
exclude:
  - object
```

Score every arm under **both**:

| view | taxonomy | what it measures |
|---|---|---|
| agnostic | `taxonomy.yaml` | "did it find the robots" — already vocabulary-neutral, the primary gate |
| merged instance | `taxonomy_merged.yaml` | naming quality on the vocabulary the 3-class model actually targets |

**Gate, unchanged:** earliest ladder checkpoint whose agnostic recall Δ CI lower bound ≥ −0.04 with
precision and F1 not significantly worse, paired vs the deployed baseline on
`nhrl_keypoints_eval_test`. Baseline anchor under `taxonomy.yaml`: recall 0.742 / precision 0.962 /
F1 0.838. Re-anchor under `taxonomy_merged.yaml` in Step 0.

Also score both arms on `hold_old_scoreable` / `hold_new_scoreable` for retention and acquisition.

## Metrics

- **Primary:** agnostic recall / precision / F1 vs baseline, δ = 0.04.
- **The merge's own hypothesis:** wrong-class rate under `taxonomy_merged.yaml`. If removing a class
  the model could not predict is genuinely helpful, this should fall well below the 5-class arms'
  0.35–0.39.
- **`house_bot` AP** — the one remaining non-generic class. Merging should not disturb it; if it moves,
  the effect is not what it appears.
- **Latency**, measured on the Jetson. A smaller head is marginally cheaper; free to record and it
  feeds the <60 ms budget.

## Success criteria

- **Merge adopted** if `cold3` is non-inferior to `cold5_new` on agnostic recall and precision **and**
  improves merged-taxonomy wrong-class rate — *and* Step 0 confirmed the keypoint path holds identity.
- **Merge rejected** if agnostic recall or precision degrades significantly. Simplification is not
  worth a detection regression.
- **Neutral is still actionable:** if the two tie, prefer the 3-class model. Fewer classes, no dead
  head, identity in the model that can actually learn it, and one less way for the pipeline to be
  wrong.
- **`train.py` defaults validated separately** by `cold5` vs `cold5_new`, whichever way the merge goes.

## Risks / caveats

- **Safety, not metrics, is the real risk.** If the keypoint model misses our robot and the bbox model
  no longer distinguishes it, our robot becomes a valid target. Step 0's replay check and the
  `NEUTRAL` fallback are mitigations, not proofs. Do not deploy a merged model until the miss rate
  from Step 0.3 is known and the hold window covers it.
- **`mr_stabs_mk2` cannot be measured.** The eval set has zero `mr_stabs_mk2` GT instances, so its
  merge rests on the architectural argument alone. Note it; do not claim it was tested.
- **One seed per arm**, and δ = 0.04 is tighter than the 0.048 cross-run spread Phase A measured. Read
  the crossing epoch as ±1 ladder step.
- **The eval remains the robot's camera; training remains overhead cage footage.** This experiment does
  not address that domain gap — it will show up again as the ep150 collapse unless `lrf 0.1` fixes it,
  which `cold5_new` is there to check.
- **Do not read the 5-class arms' 0.000 on `mrs_buff_mk3` as evidence the merge is *needed*.** It is
  evidence the current class is useless on the deployment camera. A 5-class model trained with enough
  in-domain our-robot data might work; nobody has tried that, and this plan does not.

## Artifacts / locations

- Class map: `training/yolo/remap_config_merge_our_robots.toml`
- Dataset: `training/data/scenesplit_2026-07-25_3class/{old,new,hold_old,hold_new}/` + `old+new_3class.yml`
- Taxonomy: `training/model_eval/taxonomy_merged.yaml`
- Models: `data/models/yolo26n_merged3_<date>.{pt,onnx,_x86_64_sm86.engine}`
- Scores: `training/data/nhrl_keypoints_eval_test/scores_merged3_{agnostic,merged}/`
- Writeup: `docs/experiments/perception_performance/merged_robot_class_<date>.md`
- Pipeline change: `include/robot_filter/label_group_utils.hpp` (fallback `THEIRS` → `NEUTRAL`),
  `config/_desktop.toml` / `_jetson.toml` (`label_indices` → 3 entries)
