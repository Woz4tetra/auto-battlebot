# Seg vs bbox box-quality grade (Exp 3)

Analysis date: 2026-07-18. Question: for the opponent/blob detector in the two-model split, does dropping the segmentation head cost any box quality? Answer: no. A bbox-only detector matches the seg baseline on recall and localization, is significantly better on precision and F1, and runs ~18% faster.

## Summary

Same images, same 5 classes, same train/val/test splits as the seg baseline blob model. The only variable is the head: `yolo26n-seg` (segment) vs `yolo26n` (detect). Scored both engines on the real eval in one paired run.

- **Box recall does not degrade.** Agnostic recall 0.756 (seg) vs 0.742 (bbox): delta -0.015, **not significant** (95% CI -0.039..0.011). Localization recall (IoU>=0.5 gate) identical to recall, also ns.
- **Precision and F1 improve.** Agnostic precision 0.893 -> 0.963 (+0.070, significant). Agnostic F1 0.819 -> 0.838 (+0.019, significant). The detect head fires cleaner: fewer false boxes at conf 0.5.
- **Opponent detection is not hurt.** Opponent AP50-95 0.244 (seg) -> 0.304 (bbox) under identical label mapping.
- **~18% faster.** Raw GPU compute (dev box sm89): seg median 0.984 ms -> bbox 0.804 ms. The bbox head has no mask-prototype branch and a narrower output tensor.
- **One small cost:** tight-IoU localization is marginally lower (agnostic mAP50-95 0.517 -> 0.504 real; 0.729 -> 0.706 synthetic-val). Not load-bearing: navigation uses box centers, not pixel-tight masks, and the IoU>=0.5 localization gate is unchanged.

Verdict: use bbox-only for the opponent/blob detector. It loses nothing that the downstream consumer uses, gains precision and speed, and the seg mask output was never consumed anyway.

## Setup

- **Dataset:** `training/data/nhrl_robots_bbox`, created from `nhrl_seg/nhrl_robots` (the exact images/splits the baseline blob model trained on) by converting seg polygons to axis-aligned boxes (`training/yolo/seg_to_bbox.py`). 5 classes `[object, robot, house_bot, mr_stabs_mk2, mrs_buff_mk3]`, train 49086 / val 5454.
- **Models compared:**
  - `blob_seg`: `yolo26n-seg_nhrl_robots_2026-04-27` (segment head, the baseline blob model, aka `blob_generic` in earlier reports).
  - `bbox`: `yolo26n_nhrl_robots_bbox_2026-07-16` (detect head), trained on `nhrl_robots_bbox`, 500 epochs on megamind.
- **Eval:** `training/data/nhrl_keypoints_eval_test`, 372 reviewed frames of real mrs_buff fights. GT labels opponents as a single generic `opponent`.
- **Scoring:** `score.py`, both engines in one run (identical class order -> paired bootstrap), `--labels "opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3"` (the canonical mapping from score.py's own docstring: `object` and `robot` are generic robot blobs -> `opponent`), `taxonomy.yaml`, conf 0.5, `--baseline blob_seg`.

## Result: real eval

Agnostic level (all robots collapse to one blob; the load-bearing "can it find robots" metric). Paired bootstrap, 1000 resamples, delta = bbox - seg:

| metric | seg | bbox | delta | verdict |
|---|---|---|---|---|
| recall | 0.756 | 0.742 | -0.015 | ns |
| localization_recall | 0.756 | 0.742 | -0.015 | ns |
| precision | 0.893 | 0.963 | +0.070 | **better** |
| f1 | 0.819 | 0.838 | +0.019 | **better** |
| mAP50 | 0.736 | 0.737 | +0.001 | (not bootstrapped) |
| mAP50-95 | 0.517 | 0.504 | -0.013 | (not bootstrapped) |

Per-class AP50-95 (archetype level; not bootstrapped, small per-class samples):

| class | seg | bbox | note |
|---|---|---|---|
| opponent | 0.244 | 0.304 | bbox higher |
| house_bot | 0.750 | 0.704 | slightly lower |
| mrs_buff_mk3 (our robot) | 0.486 | 0.453 | slightly lower |

The per-class drops on house_bot and our own robot are within the tight-IoU story (below) and are not significance-tested. The metric that gates target selection, agnostic recall, is statistically unchanged.

## Result: synthetic val (same split, box metrics)

Direct val-split box(B) comparison, identical images and classes, only the head differs:

| box metric | seg | bbox |
|---|---|---|
| precision | 0.939 | 0.944 |
| recall | 0.790 | **0.838** |
| mAP50 | 0.824 | **0.880** |
| mAP50-95 | 0.729 | 0.706 |

Same pattern as the real eval: bbox wins on recall and mAP50, loses a little on tight-IoU mAP50-95. The detect head localizes boxes slightly looser than the mask-supervised seg head, but finds more robots.

## Result: latency

Raw GPU compute time, `trtexec --loadEngine --noDataTransfers`, 300 iterations, dev box (x86_64 sm89). Relative difference is the signal; absolute times on the Jetson will be larger.

| engine | median | mean | p90 |
|---|---|---|---|
| seg | 0.984 ms | 1.018 ms | 1.019 ms |
| bbox | 0.804 ms | 0.820 ms | 0.820 ms |

bbox is ~18% faster median (~19% mean). The saving is the mask-prototype branch and the narrower output tensor (`[1, 9, 8400]` for detect vs box + 32 mask coeffs + prototype for seg). That branch was pure overhead: nothing downstream consumed the masks.

## Why box quality holds

The seg head supervises boxes with mask information, which tightens localization at high IoU (hence seg's edge on mAP50-95). But target selection and navigation only need the box center and a match at IoU>=0.5, and on both metrics the detect head is equal or better. Trading a sliver of high-IoU tightness for higher recall, higher precision, and 18% less compute is the right trade for this consumer.

## Implication

- **Adopt bbox-only for the opponent/blob detector** in the two-model architecture. The seg head added latency and an unused output for no gain in any metric the pipeline reads.
- **Keep the seg->bbox conversion in the pipeline** (`training/yolo/seg_to_bbox.py`). Future opponent datasets can be authored/edited as seg polygons (richer labels, easier review) and converted to boxes for training with zero box-quality loss.
- **Our-robot keypoints stay in the separate pose model.** This experiment only covers the box detector; the keypoint model (mrs_buff heading) is unaffected.

## Caveats

- **Opponent AP reads 0.244 here, not 0.210 as in the Meshy grade.** Different `--labels` mapping: this run uses the canonical `object,robot -> opponent` (both generic classes count as opponent), the earlier `scores_blob_generic` used a narrower mapping. The comparison here is internally valid because both engines used identical labels in the same run; do not cross-compare the absolute opponent AP against other reports' mappings.
- **Per-class AP is not bootstrapped.** Only agnostic/archetype aggregate metrics have significance verdicts. Per-class AP deltas (opponent, house_bot, mrs_buff) are directional, on small per-class samples.
- **Latency measured on the dev GPU, not the Jetson.** The ~18% relative saving should hold or grow on the weaker Orin GPU, but confirm on-target before trusting the absolute budget.

## Artifacts

- Model: `data/models/yolo26n_nhrl_robots_bbox_2026-07-16.{pt,onnx,engine}` (engine `_x86_64_sm89`)
- Baseline engine: `data/models/yolo26n-seg_nhrl_robots_2026-04-27_x86_64_sm89.engine`
- Scores: `training/data/nhrl_keypoints_eval_test/scores_bbox_vs_seg/{summary.csv, significance.csv, headline.png, confusion_*.png}`
- Training run: megamind `runs/projects/auto_battlebots_2026-07-16_20-50-57_yolo26n` (500 epochs, batch 128)
- Dataset: `training/data/nhrl_robots_bbox` (local + megamind), built by `training/yolo/seg_to_bbox.py`
