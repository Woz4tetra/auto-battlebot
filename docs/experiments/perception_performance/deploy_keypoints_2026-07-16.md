# Deploy-recipe model (keypoints + box-only opponents) vs baselines

Analysis date: 2026-07-16. Model: `yolo26n-pose_deploy_keypoints_2026-07-15` (Exp 2).

## Summary

The Exp 2 deployment recipe works for opponent detection and fails for keypoints.

- **Opponent detection reached the real-trained ceiling.** Opponent box AP50-95 is 0.218, matching the real seg blob (0.210) and 2.6x above the generic-synthetic model (0.084). Mixing real opponent boxes into the keypoint dataset does what we hoped for detection.
- **Keypoint quality collapsed.** Heading error on our robot jumped from 9.0 deg (deployed baseline) to 38.5 deg. Only 50% of keypoints land within 10 deg vs 80% for the baseline. This is a data/capacity ceiling, not undertraining: the pose head plateaued at epoch 16 of 150.
- **mrs_buff box AP regressed** from 0.468 to 0.308 once opponents became a competing class (wrong-class rate 0.20).

For an aim-assist system that depends on heading, a single nano network cannot serve both opponent detection and precise keypoints at once. Keep them split, or grow the backbone.

## Setup

- **Model:** `yolo26n-pose`, 3 classes `[mr_stabs_mk2, mrs_buff_mk3, opponent]`, kpt_shape [2,3]. Our robots carry real keypoints; opponent is a single box-only class (vis-0 masked keypoints, from real+synthetic boxes with our robots filtered out).
- **Training:** `training/data/deploy_keypoints` on megamind, 150/300 epochs (canceled at diminishing returns). Val at cancel: box mAP50 0.940 / mAP50-95 0.653, pose mAP50 0.650.
- **Eval:** `training/data/nhrl_keypoints_eval_test`, 372 reviewed frames. GT instances: opponent 491, mrs_buff_mk3 389, house_bot 236, object 8, **no mr_stabs** (so keypoint numbers are effectively mrs_buff heading).
- **Scoring:** `training/model_eval/score.py`, conf 0.5, IoU 0.5, 1000-resample paired bootstrap. Box uses `taxonomy.yaml`; keypoints use `taxonomy_keypoint.yaml` (opponents excluded).
- **Engines:** all x86_64_sm89, FP16, built locally. The May-01 `our_robots` baseline engine was rebuilt from its ONNX for this run.

## Box / opponent detection

Agnostic (all robots collapse to one blob) and archetype per-class AP50-95:

| model | opponent source | agnostic recall | agnostic mAP50-95 | opponent AP | mrs_buff AP | house_bot AP |
|---|---|---|---|---|---|---|
| blob_generic (real seg, 1 class) | real | **0.675** | **0.479** | 0.210 | 0.461 | 0.744 |
| blob_indiv (real seg, per-class) | real | 0.410 | 0.309 | 0.069 | 0.226 | 0.735 |
| all_robots (pose, generic-syn) | CAD distractors | 0.321 | 0.205 | 0.084 | 0.468 | 0.0 |
| **deploy (pose, real+syn)** | real + synthetic | 0.496 | 0.284 | **0.218** | 0.308 | 0.0 (no class) |

- **deploy vs all_robots is a significant win on opponent detection.** Agnostic recall +0.175 (CI 0.145-0.207), opponent AP 0.218 vs 0.084. Real opponent boxes are what close the gap; the generic CAD distractors never taught the model what real opponents look like.
- **deploy matches the real blob on opponent AP** (0.218 vs 0.210). Exp 2's core hypothesis holds: box-only supervision with masked keypoints is as good as full seg supervision for detecting opponents.
- **deploy trails blob_generic on the agnostic metric** (recall 0.496 vs 0.675). blob_generic carries a dedicated house_bot class and no keypoint burden, so it sweeps the 236 house_bot boxes deploy has no class for.
- **mrs_buff box AP dropped** (0.308 vs all_robots 0.468). Archetype precision fell 0.14 and wrong-class rate rose to 0.20: adding an opponent class pulls some mrs_buff detections into "opponent."
- Box mAP was **still climbing at cancel** (0.57 -> 0.65), so opponent detection would likely improve with a full 300-epoch run. Keypoints would not (see below).

## Keypoints (our robots, taxonomy_keypoint.yaml)

| model | kp_err_px | pck@0.1 | heading_err_deg | heading_acc@10deg |
|---|---|---|---|---|
| **our_robots (deployed baseline)** | 9.8 | 0.715 | **9.0** | **0.797** |
| all_robots (all-synthetic kp) | 15.2 | 0.568 | 15.2 | 0.769 |
| **deploy (deployment recipe)** | 19.6 | 0.360 | **38.5** | 0.504 |

- **Keypoints regressed 4x vs the deployed model** (heading 9.0 -> 38.5 deg; pck 0.715 -> 0.360). At 38.5 deg mean heading error, aim assist cannot rely on this model's orientation output.
- **This is a capacity/data ceiling, not early stopping.** Pose mAP50-95 hit 0.636 at epoch 16 and moved to 0.648 by epoch 150. The pose head stopped learning almost immediately while the box head kept improving. A 2.7M-param nano model split across 74k opponent box frames and 47k keypointed frames starves the keypoint head.
- **deploy detects our robots more often but locates them worse.** On our-robot GT its recall is 0.605 vs the baseline's 0.379, yet its keypoints are far less accurate. Better box recall, worse pose.

## Deployment implication

The single coarse-network recipe buys opponent detection at the cost of the keypoint precision the system exists to provide. Three ways forward:

1. **Two heads / two models (recommended).** Keep the dedicated our-robot keypoint model (heading 9.0 deg) and pair it with an opponent detector trained on real+synthetic boxes. The opponent detector alone already matches the real blob.
2. **Bigger backbone.** Retrain the combined recipe on `yolo26s`/`yolo26m-pose` and re-measure whether the pose plateau lifts once the head has capacity. Check the latency budget on the Jetson first.
3. **Rebalance the combined dataset.** Cap opponent box frames, or raise the keypoint loss weight, so the pose head is not drowned. Cheaper to try than (2) but unlikely to fully recover 9 deg.

## Caveats

- **Possible eval leakage.** deploy's real opponent boxes come from `nhrl_seg/nhrl_robots` match recordings. If any eval frames share those recordings, opponent AP is optimistic. Worth confirming the eval recordings are disjoint from the box training set. blob_generic trained on the same source and lands at the same AP, which is reassuring but not proof.
- **No mr_stabs in the eval.** Keypoint and mrs_buff numbers are mrs_buff-only. mr_stabs keypoint quality under this recipe is unmeasured here.
- **Early stop.** deploy ran 150/300 epochs. Box would gain from finishing; pose would not.
- **score.py class-count gotcha.** `score.py` infers `num_classes` from the `--labels` count and derives `num_keypoints` from the tensor width. Scoring a 2-class engine with a 3-label string misparses the output to `num_keypoints=0` and returns ~0 recall. Score each engine with a `--labels` list whose length equals its class count. This bit the first our_robots run.

## Artifacts

- Engine: `data/models/yolo26n-pose_deploy_keypoints_2026-07-15_x86_64_sm89.engine`
- Scores: `training/data/nhrl_keypoints_eval_test/scores_deploy`, `scores_box_cmp` (deploy vs all_robots), `scores_kp_cmp` + `scores_ourrobots` (keypoints), `scores_blob_generic`, `scores_blob_indiv`.
