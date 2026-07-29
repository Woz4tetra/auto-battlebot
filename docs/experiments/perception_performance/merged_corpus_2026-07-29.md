# yolo26n on the merged corpus — 2026-07-29

One cold-start yolo26n detect run on `nhrl_robots_bbox_2class` after the 2026-07-29 merge (32,487
frames, 71 scenes, re-split 80/20 by scene), graded on `nhrl_keypoints_eval_test` (372 reviewed
robot-camera frames) with a paired 1000× bootstrap at conf 0.5 under `taxonomy_merged.yaml`.

Predecessor: `data_scaling_2026-07-27.md`, whose `base100` arm is the comparison point here.

## Headline

1. **The merged model is worse than the pre-merge model on the eval set, at every checkpoint.** Best
   merged checkpoint (`ep75`) scores F1 **0.875 vs 0.902** for `base100_ep75`, mAP50-95 **0.540 vs
   0.570**. Nothing in the ladder reaches the model it was meant to improve on.
2. **The loss is precision, not recall.** At ep50 and ep75 recall is statistically unchanged
   (−0.008 and −0.013, both `ns`) while precision drops **−0.048 and −0.042, both significant**. The
   merged model finds robots about as well and fires on non-robots more often.
3. **This run cannot attribute that to the new data.** The 80/20 re-split that accompanied the merge
   pushed **14 NHRL scenes out of train**, cutting NHRL scene diversity from 47 to 39. The prior
   experiment established scene count as the dominant driver of exactly this failure mode. See
   §The confound — it is disqualifying, not a footnote.
4. **The merged model still beats the deployed baseline on recall** (0.869 vs 0.742) but gives up
   precision (0.881 vs 0.962). It is not a regression against what is deployed; it is a regression
   against what `data_scaling_2026-07-27` recommended deploying.

## Setup

| | |
|---|---|
| corpus | `nhrl_robots_bbox_2class` post-merge — 32,487 frames, 71 scenes |
| classes | `robot` (70,131 boxes), `house_bot` (24,347) |
| train | 45 scenes / 25,914 frames (`robot` 56,293, `house_bot` 19,541) |
| val | 26 scenes / 6,573 frames (`robot` 13,838, `house_bot` 4,806), scene-disjoint |
| model | `yolo26n` detect, cold from COCO |
| schedule | 100 epochs, batch 128, imgsz 640, `lr0` 0.01, `lrf` 0.1, `degrees` 45, `flipud` 0, `close_mosaic` 0, seed 0 |
| hardware | 3× RTX A6000, `--cache ram`, ~64 s/epoch, 1 h 35 m wall |
| run | `runs/projects/auto_battlebots_2026-07-29_02-36-15_yolo26n` |
| ladder | ep{25, 50, 75, last} |

```bash
venv/bin/python training/yolo/train.py \
  training/data/nhrl_robots_bbox_2class/data.yml yolo26n -e 100 --save-period 25 --cache ram
```

Val peaked at **ep69** (mAP50-95 0.6668) and was flat to ep100 (0.6644) — the same ep63–90 plateau
the prior experiment saw, so the 100-epoch budget was neither short nor badly over-annealed.

**Val mAP is not comparable to any earlier run.** The re-split changed the val set, so the 0.667
reported here and the numbers in `data_scaling_2026-07-27.md` are computed over different scenes.
Only the external eval below is comparable across experiments.

## Results — agnostic level

| model | precision | recall | F1 | mAP50 | mAP50-95 |
|---|---|---|---|---|---|
| deployed baseline | **0.962** | 0.742 | 0.838 | 0.737 | 0.504 |
| **base100_ep75** | 0.923 | **0.882** | **0.902** | **0.871** | **0.570** |
| base100_eplast | 0.942 | 0.865 | 0.902 | 0.852 | 0.561 |
| merged_ep25 | 0.894 | 0.812 | 0.851 | 0.792 | 0.511 |
| merged_ep50 | 0.875 | 0.874 | 0.875 | 0.850 | 0.526 |
| merged_ep75 | 0.881 | 0.869 | 0.875 | 0.844 | 0.540 |
| merged_eplast | 0.889 | 0.841 | 0.864 | 0.827 | 0.530 |

`base100_ep75` and `base100_eplast` reproduce `data_scaling_2026-07-27.md` to three decimals, which
is the harness sanity check: the two experiments are on the same footing.

### Paired bootstrap vs `base100_ep75`

| arm | recall Δ [95 % CI] | precision Δ [95 % CI] | F1 Δ |
|---|---|---|---|
| merged_ep25 | −0.070 [−0.092, −0.050] worse | −0.030 [−0.050, −0.008] worse | −0.051 worse |
| merged_ep50 | −0.008 [−0.029, +0.012] **ns** | −0.048 [−0.067, −0.032] **worse** | −0.028 worse |
| merged_ep75 | −0.013 [−0.029, +0.003] **ns** | −0.042 [−0.061, −0.023] **worse** | −0.027 worse |
| merged_eplast | −0.041 [−0.060, −0.023] worse | −0.035 [−0.054, −0.017] worse | −0.038 worse |
| base100_eplast | −0.017 worse | +0.018 better | −0.000 ns |

## Results — archetype level

| model | precision | recall | F1 | mAP50-95 | wrong-class | AP50-95 `opponent` | AP50-95 `house_bot` |
|---|---|---|---|---|---|---|---|
| base100_ep75 | 0.917 | 0.876 | 0.896 | 0.634 | 0.007 | **0.511** | **0.758** |
| base100_eplast | 0.941 | 0.865 | 0.901 | 0.628 | 0.001 | 0.501 | 0.755 |
| merged_ep50 | 0.859 | 0.858 | 0.858 | 0.562 | 0.019 | 0.486 | 0.639 |
| merged_ep75 | 0.869 | 0.857 | 0.863 | 0.585 | 0.015 | 0.492 | 0.678 |
| merged_eplast | 0.882 | 0.834 | 0.857 | 0.577 | 0.008 | 0.482 | 0.672 |

**`house_bot` takes the larger hit** — AP50-95 0.758 → 0.678, versus 0.511 → 0.492 for `opponent`.
The merged frames contain **zero `house_bot` boxes** (all 2,027 are class `robot`), so they dilute the
`house_bot` share of the corpus without adding a single positive example of it. That is a plausible
mechanism, but it is not isolated from the confound below either.

## The confound

The merge and the re-split were applied together, and the re-split moved substantially more data than
the merge added:

| | base100 train | merged train |
|---|---|---|
| frames | 26,733 | 25,914 (**−819**) |
| scenes | 47, all NHRL | 45 = **39 NHRL + 6 recap** |
| NHRL scenes retained from base100 train | — | 33 of 47 |
| NHRL scenes pushed out to val | — | **14** |
| previously-held-out val scenes pulled in | — | 6 |

Reaching an 80/20 split required growing val from 4,732 to 6,573 frames, and whole scenes are the
only unit that can move. The result is that the merged model trained on **fewer frames and fewer NHRL
scenes** than `base100` did, despite the corpus as a whole being larger.

This matters because `data_scaling_2026-07-27.md` measured what happens when NHRL scene count falls:
its `scene50` arm (24 scenes) failed the parity gate specifically on **precision** (−0.061, −0.047,
−0.086 across checkpoints) while recall held. That is the same signature seen here — recall `ns`,
precision significantly down — at a milder scene reduction (47 → 39) and a correspondingly milder
precision loss (−0.042 to −0.048).

**So the three candidate explanations are entangled and this run separates none of them:**

1. the 1,022 added recap frames are off-distribution and hurt (plausible — plywood arena, oblique
   handheld camera, nothing like the ZED overhead view; and they are 6.4 % of the new val),
2. losing 14 NHRL training scenes hurt (has direct precedent in the prior experiment),
3. the added frames contain no `house_bot`, diluting that class.

The honest reading is **(2) has the strongest prior support**, and attributing the regression to the
new footage would be unjustified on this evidence.

## The experiment that would settle it

Train one arm on **base100's exact 47 NHRL train scenes plus the 15 recap scenes**, validated on the
original 9 val scenes. That holds NHRL training scenes fixed and varies only the recap data, making
the delta against `base100_ep75` directly interpretable. Roughly 1 h 40 m on 3 GPUs.

If that arm matches `base100_ep75`, the recap data is neutral and the 80/20 re-split is what cost the
precision — in which case the re-split should be reconsidered, not the merge. If it still loses
precision, the recap footage is genuinely harmful and should be dropped or down-weighted.

## Risks / caveats

- **One seed.** The prior series measured ~0.048 recall spread between nominally identical runs. The
  significant precision deltas here (−0.042 to −0.048) are near that magnitude; they are consistent
  across all four checkpoints, which argues against pure noise, but a second seed would harden it.
- **The confound above is the dominant caveat** and is not a minor one — it is the reason this
  report recommends no corpus change.
- **`--labels` is easy to get wrong.** Scoring a 2-class model under `taxonomy_merged.yaml` requires
  `--labels "opponent,house_bot"`, not `"robot,house_bot"` — the taxonomy maps our robots to
  `opponent` in GT. The wrong mapping silently produces `wrong_class_rate` 0.82 and archetype recall
  ~0.15 while leaving agnostic metrics untouched, so it looks like a class-confusion result rather
  than a configuration error. Check `wrong_class_rate` before reading any archetype table.
- **Domain gap unchanged.** Training is overhead cage footage; the eval is the robot's own ZED. The
  merge made this *worse*, not better — the added scenes are a third viewpoint that matches neither.
- **`--cache ram` is non-deterministic** (ultralytics warns on it). Used because the disk cache would
  write ~200 GB of full-res `.npy` against 286 GB free. Exact reproduction of this run is not
  guaranteed.

## Recommendation

- **Do not deploy the merged model.** `base100_ep75` remains the best available detector on this
  eval; nothing here displaces the `data_scaling_2026-07-27.md` recommendation.
- **Do not conclude the merged data is bad.** Run the isolating arm above first — the evidence
  currently points at the re-split, not the footage.
- **Reconsider the 80/20 split for this corpus.** 80/20 is a reasonable default in the abstract, but
  on a 71-scene corpus where scene diversity drives accuracy, spending 14 NHRL scenes on val to hit a
  round ratio is a poor trade. The prior 47/9 carve reserved less and scored better.
- **If the recap footage is kept, label its `house_bot` instances or accept the dilution.** Adding
  frames that are 100 % one class shifts the class balance whether or not that is intended.

## Artifacts

- Report assets: `assets/2026-07-29_merged_corpus/{headline.png, train_curves.png}`
- Scores: `training/data/nhrl_keypoints_eval_test/scores_merged_2026-07-29/{summary.csv,
  significance.csv, headline.png, confusion_*.png}`
- Run: `runs/projects/auto_battlebots_2026-07-29_02-36-15_yolo26n/` (weights `epoch{25,50,75}.pt`,
  `last.pt`, `best.pt`)
- Engines: `merged_{ep25,ep50,ep75,eplast}_x86_64_sm86.engine` (scratch; rebuild from the run weights)
- `base100` weights + engines preserved at
  `/media/storage/auto-battlebots-archive/experiments_2026-07/datascale_weights/`
- Corpus split of record: `training/data/nhrl_robots_bbox_2class/split_manifest.json`; pre-merge file
  placement in `.pre_resplit_2026-07-29.txt`
