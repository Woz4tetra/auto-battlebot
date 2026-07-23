# Synthetic + real bbox opponent detector, 2026-07-22

Analysis date: 2026-07-22. Question: does adding the available synthetic opponent diversity to the real
bbox training set improve opponent detection, without regressing our robots? Answer on the real
held-out val split: **yes, modestly and where it matters** — the main opponent class gains ~2 pp recall
and a little AP, our robots hold or improve slightly, at the cost of a small tight-IoU dip on house_bot.
The independent-eval grade (unseen fights) remains a dev-box follow-up.

Plan: `synthetic_plus_bbox_plan.md`. This is the results writeup.

## Summary

- **Opponent recall improves.** The dominant opponent class `robot` gains **recall 0.877 → 0.896
  (+0.0195)** on 9,191 val instances (~180 more opponents found), with mAP50 +0.002. The smaller generic
  `object` blob class gains mAP50 +0.015 and mAP50-95 +0.010. Synthetic diversity helped exactly the
  class the experiment targeted.
- **Our robots are not hurt** (target selection safe). `mrs_buff_mk3` mAP50-95 0.789 → 0.794, precision
  0.965 → 0.982; `mr_stabs_mk2` recall +0.009, mAP50-95 +0.012.
- **One small cost:** `house_bot` mAP50-95 0.913 → 0.892 (−0.021) — a tight-IoU localization dip (its
  mAP50 is unchanged at ~0.988), not a detection loss. house_bot is not target-selection-critical.
- **Aggregate is net positive but small:** all-class recall 0.831 → 0.838, mAP50 0.873 → 0.879,
  mAP50-95 0.7085 → 0.7092.
- **Trade shape:** synthetic buys opponent recall (+2 pp) and a hair of opponent AP for a −0.011 dip in
  `robot` precision (a few more false boxes from the domain gap) and the house_bot tight-IoU dip. For an
  opponent detector whose weak spot is *finding* opponents, that is the right direction.

Verdict: at a **0.36× synthetic dose** the available synthetic diversity gives a real, if modest, lift to
opponent detection on real held-out frames with no cost to our robots. Encouraging enough to justify the
independent-eval grade and, if that holds, an oversampled higher-dose arm. Not yet a deploy decision.

## Setup

- **Head/regime held constant.** Both models are `yolo26n` detect, 500 epochs, batch 128, imgsz 640, DDP
  across 3 A6000s, identical Ultralytics 8.4.9 settings (`close_mosaic=10`, linear LR, `lrf=0.01`). The
  only variable is the training mix.
- **`real_bbox` (baseline):** the existing `nhrl_robots_bbox` 500-epoch model, **reused not retrained**
  (`runs/projects/auto_battlebots_2026-07-16_20-50-57_yolo26n/weights/best.pt`, staged as
  `data/models/yolo26n_nhrl_robots_bbox_2026-07-16.pt`). 5 classes `[object, robot, house_bot,
  mr_stabs_mk2, mrs_buff_mk3]`, train 49,086.
- **`mix_all`:** real bbox train + **all** converted synthetic, train 67,533 (49,086 real + 18,447
  synth), **val kept real-only** (the real `nhrl_robots_bbox` val, unchanged) so the val curve is
  apples-to-apples with the reused baseline. Trained on `training/data/mix_all`, 500 epochs, 19.9 h.
  best.pt staged as `data/models/yolo26n_mix_all_2026-07-22.pt`.
- **Synthetic source (no rendering).** The existing synthetic keypoint set
  `training/data/all_robot_keypoints` (BlenderProc `synthgen`, ~97% synthetic) converted to bbox by
  `training/yolo/pose_to_bbox.py`: keypoint columns dropped, classes remapped into the bbox vocabulary
  (`nhrl_robot` → generic `robot`; `mr_stabs_mk2`→3, `mrs_buff_mk3`→4), images hardlinked, `.npy` cache
  skipped. Output `training/data/synth_bbox_from_keypoints`, 20,497 frames, **0 integrity errors**.
- **Dose.** Real train already holds **99,655** opponent boxes (`object` 15,568 + `robot` 84,087); the
  synthetic set adds **36,108** opponent (`robot`) boxes = a **0.36× dose**. The plan's 1×/3× arms are not
  reachable from this data without duplicating frames, so this is the single achievable `mix_all` arm.

### Why this is a val-split comparison, not `score.py`

The independent hand-labeled eval `nhrl_keypoints_eval_test` (where the reference points 0.084 / 0.21 /
0.675 were measured) is **not on megamind**, and `score.py` is purpose-built for the MCAP-derived eval
sets (it reads `data.yaml` and does `int(label_path.stem)` to align GT to a playback by SVO stamp) so it
cannot score a training dataset whose frames are named `..._frame_004090`. The achievable on-megamind
real comparison is therefore **Ultralytics `model.val()` on the real held-out val split** (5,454 real
frames, identical for both models, neither trained on it, `mix_all` kept val real-only). It is
same-corpus-as-training (different frames of the same fights), so it answers *"does synthetic help on
real held-out frames"* — real and valid, weaker than generalization to unseen fights.

## Result: real val split (5,454 frames)

`model.val()`, `conf=0.001 iou=0.6 imgsz=640`, identical for both. Δ = mix_all − real_bbox. Val instance
counts in parentheses.

| class (val instances) | metric | real_bbox | mix_all | Δ |
|---|---|---|---|---|
| **robot** (9,191) — main opponent | recall | 0.8769 | **0.8964** | **+0.0195** |
| | precision | 0.9300 | 0.9186 | −0.0114 |
| | mAP50 | 0.9293 | 0.9316 | +0.0023 |
| | mAP50-95 | 0.7127 | 0.7105 | −0.0022 |
| **object** (1,714) — generic blob | mAP50 | 0.5486 | 0.5639 | +0.0153 |
| | mAP50-95 | 0.3931 | 0.4027 | +0.0096 |
| | precision | 0.8556 | 0.8789 | +0.0233 |
| | recall | 0.4568 | 0.4597 | +0.0029 |
| **mrs_buff_mk3** (1,983) — our robot | mAP50-95 | 0.7892 | 0.7943 | +0.0051 |
| | precision | 0.9646 | 0.9823 | +0.0177 |
| | recall | 0.9239 | 0.9231 | −0.0008 |
| **mr_stabs_mk2** (1,451) — our robot | recall | 0.9276 | 0.9366 | +0.0090 |
| | mAP50-95 | 0.7343 | 0.7463 | +0.0120 |
| **house_bot** (2,804) | mAP50 | 0.9869 | 0.9884 | +0.0015 |
| | mAP50-95 | 0.9130 | 0.8921 | −0.0209 |
| **ALL** | recall | 0.8311 | 0.8377 | +0.0066 |
| | mAP50 | 0.8732 | 0.8785 | +0.0053 |
| | mAP50-95 | 0.7085 | 0.7092 | +0.0007 |

## Interpretation

- **The lift lands on opponents.** `robot` is the class the synthetic `nhrl_robot` boxes fed, and it is
  the class that moved most: +2 pp recall on 9,191 instances (~180 more opponents localized) for −1 pp
  precision — the model finds more opponents and fires a few more false boxes, netting a small mAP50 gain.
  This is the diversity effect the experiment predicted: the real set has many opponent boxes but few
  distinct opponents; synthetic adds pose/shape/lighting variety that generalizes to more of the val
  opponents.
- **No cost to our robots.** Both our-robot classes hold or improve. That matters more than the aggregate:
  a mix that bought opponents by degrading `mrs_buff` would be non-deployable (aim assist needs our
  robot), and it did not.
- **house_bot tight-IoU dip is the one regression**, and a benign one: mAP50 is flat (~0.988), only the
  high-IoU mAP50-95 fell (−0.021). Navigation uses box centers, not pixel-tight boxes, and house_bot is
  not a target-selection driver.
- **Same tail behavior as every long run.** `mix_all`'s val box-loss bottomed at **e432** then flattened
  while train loss kept dropping (overfitting onset), and `close_mosaic` at e491 traded a sliver of recall
  for precision in the last 10 epochs — the effect documented separately. best.pt is fitness-selected, so
  the tail dip does not bias this comparison; both models take their own best.pt under identical settings.

## Caveats

- **0.36× dose, one arm.** This tests whether the *available* synthetic helps at all, not the dose curve.
  A higher-dose arm needs either more rendered synthetic or frame oversampling (pure reweight). The
  positive result here is what justifies spending that compute next.
- **Same-corpus val, not independent fights.** The val frames are different frames of the same recordings
  the training data is drawn from. The gains are real held-out gains but do not prove generalization to
  *unseen* opponents/fights. That is the independent-eval question.
- **Point estimates, not bootstrapped.** `model.val()` gives no significance test. The `robot` recall
  gain is large-sample (9,191 instances) and likely real; the sub-0.01 aggregate deltas are directional.
  The paired bootstrap (via `score.py`) only comes with the independent eval.
- **No agnostic-recall / opponent-AP-vs-reference-points number here.** Those are `score.py` metrics under
  the `object,robot → opponent` collapse; this writeup reports Ultralytics per-class box metrics instead.
  Do not cross-compare these absolute numbers against the 0.084 / 0.21 / 0.675 reference points — different
  scorer, different eval.

## Follow-up: independent-eval grade (dev box)

The definitive test — agnostic recall, opponent AP, paired bootstrap against `nhrl_keypoints_eval_test`
with the plan's reference points — runs on the dev box (where that eval set lives):

1. Copy `data/models/yolo26n_{nhrl_robots_bbox_2026-07-16,mix_all_2026-07-22}.pt` to the dev box.
2. Build `_x86_64_sm89` engines there (`convert_to_onnx.py` → `convert_to_tensorrt.py`).
3. Run the paired `score.py` from the plan (§5, `--labels "object,robot,house_bot,mr_stabs_mk2,
   mrs_buff_mk3"`, `--baseline real_bbox`) against `nhrl_keypoints_eval_test`.

If agnostic recall rises with precision/F1 held, the val result generalizes to unseen fights and an
oversampled higher-dose arm is worth training.

## Artifacts

- Converter: `training/yolo/pose_to_bbox.py`
- Synthetic bbox set: `training/data/synth_bbox_from_keypoints` (from `all_robot_keypoints`, unmodified)
- Pooled dataset: `training/data/mix_all` (real train + all synthetic; val real-only)
- Models: `data/models/yolo26n_mix_all_2026-07-22.pt`,
  `data/models/yolo26n_nhrl_robots_bbox_2026-07-16.pt` (baseline, reused)
- Training run: `runs/projects/auto_battlebots_2026-07-22_02-40-51_yolo26n` (500 epochs, 19.9 h, batch 128)
- Val comparison: `runs/valcmp_real_bbox`, `runs/valcmp_mix_all`
