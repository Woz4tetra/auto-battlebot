# Mask centroid vs box center — 2026-08-03

Question for the blog post: when the YOLO-seg blob model emits a mask and a box for the same
robot, does the mask centroid tell the pipeline anything the box center does not? It does not.
The centroid sits 2.75 px from its own box center at the median, 2.5 % of the robot's own
on-screen size, and when both are graded against hand-labeled ground truth the **box center is
the better position estimate**, by 1.38 px (95 % CI [1.08, 1.71]).

The second half of the run scores the seg baseline against the deployed-track 2-class mixed bbox
model on the same eval set. The bbox model wins on every headline metric, so nothing is lost by
throwing the mask head away.

## Headline

1. **The centroid lands on the box center.** Median offset 2.75 px (0.025 of the box's longer
   side). 87.6 % of detections fall within 5 % of that side, 99.3 % within 10 %. The worst
   detection in 817 is 31.5 px off, 17.7 % of its box.
2. **The centroid is not more accurate, it is slightly less.** Against the GT box center, the
   mask centroid errs 6.66 px mean / 4.70 px median; the seg box center errs 5.28 / 3.59. Paired
   over the 741 robots the seg model found, the centroid is **+1.376 px worse** [+1.077, +1.710].
3. **A bbox-only model matches the seg box center.** On the 698 robots both models found, the seg
   box center and the 2-class mixed model's box center differ by +0.315 px [−0.030, +0.682],
   **not significant**. All three estimates put the robot in the same place.
4. **The bbox model is the better detector anyway.** Agnostic F1 0.893 vs 0.819, recall 0.847 vs
   0.756, precision 0.944 vs 0.894, mAP50-95 0.560 vs 0.517.

![centroid offset and position error](assets/2026-08-03_mask_centroid/mask_centroid_vs_box.png)

## Setup

| | |
|---|---|
| eval set | `training/data/nhrl_keypoints_eval_test`, 372 reviewed frames, 1026 scored GT robot boxes |
| seg baseline | `yolo26n-seg_nhrl_robots_2026-04-27`, 5 classes `[object, robot, house_bot, mr_stabs_mk2, mrs_buff_mk3]` |
| bbox comparison | `yolo26n_nhrl_robots_bbox_2class_mixed_2026-07-31`, 2 classes `[robot, house_bot]` |
| taxonomy | `taxonomy_merged.yaml` for both (our robots fold into `opponent`; `object` excluded) |
| thresholds | conf 0.5, NMS IoU 0.45, match IoU 0.5, imgsz 640 |
| hardware | dev box, RTX A6000 (sm86), `_x86_64_sm86` engines |

Offsets are normalized by the box's longer side, the same convention `score.py` uses for keypoint
PCK. A typical robot box on this eval set is 122 px on its longer side (p10 59, p90 242), so 0.025
is about 3 px.

## Result 1: the centroid sits on the box center

817 seg detections at conf 0.5, each carrying a mask.

| quantity | mean | median | p90 | p95 | max |
|---|---|---|---|---|---|
| offset (px) | 4.27 | 2.75 | 9.59 | 13.14 | 31.48 |
| offset (fraction of longer side) | 0.029 | 0.025 | 0.053 | 0.066 | 0.177 |
| mask fill of its own box | 0.752 | 0.758 | 0.890 | 0.916 | 0.969 |

Fraction of detections whose centroid falls within a given fraction of the box's longer side:

| threshold | share |
|---|---|
| 0.02 | 0.378 |
| 0.05 | 0.876 |
| 0.10 | 0.993 |

The mask fills 76 % of its box at the median, which is why the two agree. A robot seen from the
arena camera is a roughly convex blob that fills most of its bounding rectangle, so the pixel-mass
centroid and the rectangle center coincide.

Per class, and by box size:

| class | n | median offset | p90 | median mask fill |
|---|---|---|---|---|
| house_bot | 210 | 0.021 | 0.058 | 0.822 |
| mrs_buff_mk3 | 290 | 0.022 | 0.043 | 0.739 |
| opponent | 282 | 0.029 | 0.059 | 0.713 |
| mr_stabs_mk2 | 35 | 0.035 | 0.051 | 0.808 |

| box longer side (px) | n | median offset (px) | median offset (norm) |
|---|---|---|---|
| 29–80 | 205 | 1.57 | 0.024 |
| 80–122 | 204 | 2.10 | 0.021 |
| 122–173 | 204 | 3.20 | 0.022 |
| 173–644 | 204 | 7.12 | 0.031 |

The normalized offset is flat across box sizes. The pixel offset grows with the box because it is
a fixed fraction of it, not because big detections are worse.

## Result 2: which position estimate is closest to ground truth

Distance from each estimate to the GT box center, over class-blind IoU≥0.5 matches.

| estimate | n matched | mean (px) | median (px) | mean (norm) | median (norm) |
|---|---|---|---|---|---|
| seg mask centroid | 741 | 6.66 | 4.70 | 0.050 | 0.039 |
| seg box center | 741 | 5.28 | 3.59 | 0.042 | 0.031 |
| bbox-only model box center | 865 | 4.89 | 3.62 | 0.041 | 0.031 |

Paired bootstrap (1000 resamples, 95 % CI) over the GT boxes both estimators matched:

| comparison | n | delta (px) | 95 % CI | verdict |
|---|---|---|---|---|
| mask centroid − seg box center | 741 | +1.376 | [+1.077, +1.710] | **seg box center better** |
| mask centroid − bbox-only model | 698 | +1.710 | [+1.324, +2.132] | **bbox-only model better** |
| seg box center − bbox-only model | 698 | +0.315 | [−0.030, +0.682] | ns |

The centroid loses because a robot silhouette is not symmetric. A spinning weapon, a wedge, or a
partly occluded chassis pulls the pixel mass off center while the box stays anchored to the
extremes. Part of the gap is the metric: the labeler drew tight rectangles, so the box center is
the quantity ground truth encodes, and an estimator that reports a box center starts with an
advantage. The pipeline consumes a box center too, which is why this is the scale that matters.

## Result 3: seg baseline vs 2-class mixed bbox model

Two separate `score.py` runs. `--labels` is one shared mapping for all candidates, so a 5-class and
a 2-class engine cannot share a run, which also means no paired bootstrap between them.

Agnostic level, all robots collapsed to one blob:

| model | precision | recall | F1 | mAP50 | mAP50-95 | matched GT |
|---|---|---|---|---|---|---|
| seg baseline | 0.894 | 0.756 | 0.819 | 0.736 | 0.517 | 741 / 1026 |
| **bbox 2class mixed** | **0.944** | **0.847** | **0.893** | **0.833** | **0.560** | 865 / 1026 |

Archetype level (`opponent`, `house_bot`), the vocabulary both models can emit:

| model | precision | recall | F1 | mAP50-95 | opponent AP50-95 | house_bot AP50-95 | wrong-class rate |
|---|---|---|---|---|---|---|---|
| seg baseline | 0.876 | 0.741 | 0.803 | 0.594 | 0.439 | **0.749** | 0.021 |
| **bbox 2class mixed** | **0.937** | **0.841** | **0.886** | **0.610** | **0.513** | 0.707 | **0.007** |

The bbox model finds 124 more robots out of 1026, fires wrong-class a third as often, and gives up
0.042 of house_bot AP. The seg baseline's agnostic numbers reproduce `seg_vs_bbox_2026-07-18.md` to
three decimals and the bbox model's reproduce `synthetic_arms_2026-07-31.md`, which is the harness
sanity check that these runs sit on the same footing as the earlier reports.

Instance level is not comparable and is omitted. The 2-class model has no `mr_stabs_mk2` or
`mrs_buff_mk3` head, so every correct detection of our own robot scores as a wrong class (0.367
wrong-class rate against the seg model's 0.153).

## Caveats

- **The centroid analysis runs through ultralytics on the `.pt` weights, not the TensorRT engine.**
  `TrtYoloModel` discards the mask coefficients, so there is no engine path to a mask. Detection
  counts from this path differ slightly from the engine runs in Result 3. The offsets are still
  self-consistent because centroid and box come from the same forward pass.
- **Offsets are measured only where the seg model fired.** The 285 GT robots it missed contribute
  nothing here, so this says nothing about whether a mask would help on hard detections.
- **35 detections are labeled `mr_stabs_mk2`, which does not appear in these fights.** They are
  misclassifications of other robots. Their mask-vs-box geometry is still valid, so they stay in
  the offset tables and are called out in the per-class row.
- **No paired significance between the two models** for the reason above. The accuracy comparison
  in Result 3 is two independent runs on identical frames, not a paired bootstrap.
- **Ground truth is axis-aligned boxes.** A mask-based GT would grade the centroid differently.
  Boxes are what the labeling tool produces and what target selection consumes, so this is the
  relevant scale.

## Reproduce

```bash
source scripts/activate_python.sh
OUT=training/data/nhrl_keypoints_eval_test/scores_mask_centroid_2026-08-03

python training/model_eval/mask_centroid_vs_box.py training/data/nhrl_keypoints_eval_test \
  --seg-weights data/models_v2/yolo26n-seg_nhrl_robots_2026-04-27.pt \
  --seg-labels "opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3" \
  --bbox-weights data/models/yolo26n_nhrl_robots_bbox_2class_mixed_2026-07-31.pt \
  --bbox-labels "opponent,house_bot" \
  --taxonomy training/model_eval/taxonomy_merged.yaml --conf 0.5 --output $OUT/centroid

python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate seg_baseline=data/models_v2/yolo26n-seg_nhrl_robots_2026-04-27_x86_64_sm86.engine \
  --labels "opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3" \
  --taxonomy training/model_eval/taxonomy_merged.yaml --conf 0.5 --output $OUT/seg

python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate bbox_2class_mixed=data/models/yolo26n_nhrl_robots_bbox_2class_mixed_2026-07-31_x86_64_sm86.engine \
  --labels "opponent,house_bot" \
  --taxonomy training/model_eval/taxonomy_merged.yaml --conf 0.5 --output $OUT/bbox
```

## Artifacts

- Tool: `training/model_eval/mask_centroid_vs_box.py` (new)
- Scores: `training/data/nhrl_keypoints_eval_test/scores_mask_centroid_2026-08-03/{centroid,seg,bbox}/`
- Report assets: `assets/2026-08-03_mask_centroid/{mask_centroid_vs_box.png, centroid_summary.csv, summary_seg_baseline.csv, summary_bbox_2class_mixed.csv, headline_*.png}`
- Per-detection offsets: `.../centroid/centroid_offsets.csv` (817 rows)
- Per-GT-box position errors: `.../centroid/position_errors.csv` (1026 rows)

## Related

- `seg_vs_bbox_2026-07-18.md` established that dropping the seg head costs no box quality and runs
  ~18 % faster. This report closes the remaining gap in that argument: the mask output the head
  was computing carried no position information the box did not already have.
- `synthetic_arms_2026-07-31.md` is where the 2-class mixed model comes from.
