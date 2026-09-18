# The cage-high eval set

`training/data/cage_high_x50_conf044`, the second eval set the domain-mix plan asked for:
broadcast footage from a cage mount, where the grid had only been scored on ZED footage from the
robot. Hand-corrected 2026-09-17 and scored 2026-09-18.

What the arms scored on it is in
[synthetic_domain_mix_2026-09-18.md](synthetic_domain_mix_2026-09-18.md). This file is the set
itself: what is in it, how it was labelled, and what that does to a number scored against it.

## What this set is

636 frames over nine recordings, all Mrs Buff matches, hand-corrected in `edit_labels.py` and
marked `pass` in `validation_state.json`.

| Venue | Frames | Recordings | Opponents |
| --- | --- | --- | --- |
| NHRL cage-high (Brettzone broadcast) | 516 | 6 | clyde, ironwarrior, johnundercutter, sphinx, stingoperation, wreckcreation |
| MassD (MassDestruction broadcast) | 120 | 3 | beeroll, stardust, sirslicey |

Ground truth holds 1,750 boxes: 629 `mrs_buff_mk3`, 609 `opponent`, 512 `house_bot`, and no
`mr_stabs_mk2` or `object` at all. Mr Stabs never fought in these matches, so every
`mr_stabs_mk2` prediction is a false positive with no ground truth needed to call it. Keypoints
are near-complete: 3,440 visible, 57 occluded, 3 out-of-frame.

Both venues are broadcast footage, so neither is the ZED the existing eval set uses nor the
e-CAM25 the render targeted. The MassD frames were cropped to 1920x895 on 2026-09-18 to cut the
scoreboard and ticker banners; the NHRL frames stay 1920x1080.

## How it was labelled, and what that costs

**The ground truth was seeded by one of the arms being scored.** `prelabel_holdout.json` records
the seed as `yolo26x-pose_d40000_2026-09-16_last.pt` at conf 0.44, and `--holdout 0`, so no blind
audit frames exist. The plan's own guard reads "never pre-label with an arm and then score that
arm as the headline without the audit number beside it." That audit number does not exist for
this set, so `x_d40000_ep50`'s numbers here are the least trustworthy in the table, and every
arm benefits from GT that started as a strong model's output and was corrected rather than drawn
from scratch.

What can be measured is the correction volume. The seed wrote 1,829 boxes; the corrected GT holds
1,750, over 14 fewer frames.

| Venue | Model boxes | Final GT boxes | Delta |
| --- | --- | --- | --- |
| NHRL | 1,397 | 1,532 | +135 |
| MassD | 432 | 218 | -214 |

Hand correction added boxes on NHRL footage and deleted about half of them on MassD, so the seed
under-detected at NHRL and over-detected at MassD. Roughly 40 of the deletions are the 14 frames
that later failed validation and were moved to `validation_backup/`, which the model-box column
still counts.

## The frame-key collision it exposed

**The frame loader used to drop 65 of these 636 frames, and was fixed before the scoring run.**
`auto_battlebot/eval/dataset.py` keyed GT frames by `int(stem)`. Every recording in
this set starts at stamp 0, so the nine frames named `0000000000000000000` collapsed into one
and `score.py` scored 571 frames while reporting it in a line nobody reads twice. Frames are
now keyed by `FrameKey(dataset, stamp_ns)`, which pairs the subdataset with the stamp, and
review state is matched per subdataset for the same reason. `tests/python/test_eval_dataset.py`
locks both in. The existing `nhrl_keypoints_eval_test` was never affected: its stems are SVO
stamps, unique across recordings, and it still loads all 688 frames.

Every number in the results writeup comes from the fixed loader reading the dataset directly. The
tables were first produced through a symlink view that sidestepped the collision, and re-running
all seven scoring runs afterwards reproduced all 395 published numbers exactly, so the fix changed
the path to the numbers and not the numbers.

## Scoring it

`score.py` at the pre-registered `--conf 0.5`, `--nms-iou 0.45`, 1000-sample paired bootstrap
against `base`, with the arms' class 2 (`nhrl_robot`) read as GT `opponent`.

| View | Taxonomy | Frames |
| --- | --- | --- |
| `opponent` | `taxonomy_opponent.yaml` | 636 |
| `pooled` | `taxonomy.yaml` | 636 |
| `heading` | `taxonomy_keypoint_ours.yaml` | 636 |
| `venue_nhrl` | `taxonomy_opponent.yaml` | 516 |
| `venue_massd` | `taxonomy_opponent.yaml` | 120 |

Output lands in `training/data/cage_high_x50_conf044/scores_domain_mix/`.

Add `opponent_conf025` and `opponent_conf075` for the threshold read. One run, for the headline
metric:

```bash
venv/bin/python training/model_eval/score.py training/data/cage_high_x50_conf044/d40000 \
    --candidate base=data/models/yolo26s-pose_base_2026-09-13_last_x86_64_sm86.engine \
    ... --labels mr_stabs_mk2,mrs_buff_mk3,opponent,house_bot \
    --taxonomy training/model_eval/taxonomy_opponent.yaml --conf 0.5 \
    --baseline base --bootstrap 1000 --output <out>/opponent
```

Point it at the `d40000/` subdirectory, not the set root: `_dataset_dirs` looks one level deep for
a `data.yaml` and the recordings sit two levels down.
