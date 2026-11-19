# Box detection: all_robot_keypoints pose model vs baseline seg blob, 2026-07-14

For finding robot boxes anywhere in the frame, the deployed **seg blob baseline wins by a wide margin**:
agnostic (class-blind) recall ceiling 0.919 vs the all_robot_keypoints pose model's 0.658, and 0.756 vs
0.321 at a matched confidence of 0.5. But the entire gap is **house_bot and opponents** — classes the
blob was trained on with real match footage and the pose model either lacks or barely learned from
synthetic data. On our own robot (mrs_buff_mk3) the two are **tied** (0.73 vs 0.72 correct box recall).

So this is not "the pose model is a worse detector of our robot." It is "the seg blob is a general
all-robot detector trained on real data, and the pose model is a synthetic-trained, class-limited model
whose job is keypoints on our robots, not general detection." For the detection job, keep the blob.

## Models compared

- **pose_all** — `yolo26n-pose_all_robots_2026-07-14_x86_64_sm89.engine`, 3 classes (mr_stabs_mk2,
  mrs_buff_mk3, nhrl_robot). Trained on `all_robot_keypoints` (~97% synthetic). Box head of the pose model.
- **blob_generic** — `yolo26n-seg_nhrl_robots_2026-04-27_x86_64_sm89.engine`, 5 classes (object, robot,
  house_bot, mr_stabs_mk2, mrs_buff_mk3), the deployed seg blob baseline, trained on real match footage.
- **pose_our** — `yolo26n-pose_our_robots_2026-05-01_x86_64_sm89.engine`, 2 classes (our robots only),
  the deployed keypoint model, included as a reference.

This is a cross-task comparison (seg produces box+mask, pose produces box+keypoint), but both emit boxes,
so box detection is directly comparable. It is not an architecture A/B: the models differ in task,
classes, and training data at once. Read it as "which deployed model detects robot boxes best," not "is
seg better than pose."

## Ground truth

`training/data/nhrl_keypoints_eval_test`, 372 reviewed frames, scored with `taxonomy.yaml` (excludes only
`object` debris; opponents, house_bot, and our robots all count). Box instances: 359 mrs_buff_mk3, 461
opponent, 206 house_bot; no mr_stabs_mk2 in this GT. `score.py --conf` gates predictions before mAP, so
the low-conf rows are the fair, threshold-independent comparison.

## Agnostic box detection (localization, class-blind) — the fair headline

How well each model puts a box around *any* robot, regardless of label:

| conf | blob_generic (mAP50 / recall) | pose_all (mAP50 / recall) | pose_our (mAP50 / recall) |
|---|---|---|---|
| 0.05 | **0.862 / 0.919** | 0.544 / 0.658 | 0.352 / 0.365 |
| 0.30 | **0.821 / 0.856** | 0.423 / 0.457 | 0.314 / 0.314 |
| 0.50 | **0.736 / 0.756** | 0.317 / 0.321 | 0.305 / 0.304 |
| 0.60 | **0.664 / 0.675** | 0.207 / 0.208 | 0.286 / 0.283 |

The blob leads at every threshold. Two secondary points: pose_all's recall collapses much faster with
confidence (0.658 -> 0.208 from conf 0.05 -> 0.6) than the blob's (0.919 -> 0.675), the same
synthetic-training under-confidence seen in the keypoint report; and pose_our is capped low because with
only two classes it structurally cannot box opponents or house_bot (461 + 206 of the 1,026 GT boxes).

## Instance box detection (localization + correct label)

| conf | blob_generic (mAP50 / recall / wrong) | pose_all (mAP50 / recall / wrong) | pose_our |
|---|---|---|---|
| 0.05 | **0.748 / 0.741 / 0.194** | 0.406 / 0.590 / 0.104 | 0.294 / 0.315 / 0.139 |
| 0.50 | **0.659 / 0.639 / 0.155** | 0.281 / 0.318 / 0.009 | 0.279 / 0.297 / 0.022 |

The blob's lower wrong-class rate advantage is partly structural: it has more classes to confuse (opponent
vs house_bot vs our robots), while pose_all has fewer, so pose_all's low wrong-class rate (0.009 at 0.5)
is not a quality win.

## Where the gap actually is (per-class, conf 0.5)

Correct box recall by GT class (from the confusion matrices):

| GT class | blob_generic | pose_all | note |
|---|---|---|---|
| house_bot (206) | **0.908** | 0.000 | pose_all has no house_bot class; it misses all 206 |
| mrs_buff_mk3 — our robot (359) | 0.727 | **0.721** | tied |
| opponent (461) | **0.451** | 0.145 | blob ~3x; opponents are real-trained for the blob, synthetic-only for pose_all |

Per-class box AP50-95 tells the same story: mrs_buff (blob 0.486 vs pose_all 0.468) is a wash, while
house_bot (0.750 vs 0.000) and opponent (0.244 vs 0.084) are where the blob pulls away.

## Why the blob wins box detection

- **It was trained on real, comprehensive match footage** covering opponents and house_bot, so it detects
  the full scene. The pose models are ~97% synthetic and (for pose_all) learned opponents only from
  synthetic renders, which transfer weakly (see the keypoint report's synthetic-generator findings).
- **Class coverage.** pose_all has no house_bot class at all and pose_our has neither house_bot nor an
  opponent class. Those missing classes are ~65% of the GT boxes, so the pose models cannot compete on
  whole-scene recall no matter how good their detector is.
- **On our own robot, they are equal.** mrs_buff box recall is 0.72–0.73 for both the blob and pose_all,
  so the pose model is a competent detector of the thing it was built for.

## Caveats

- Cross-task, cross-data comparison; not an architecture benchmark. It answers "which deployed model
  boxes robots best," which is the operational question.
- x86 sm89 engines, not the Jetson aarch64 engines.
- GT has no mr_stabs_mk2 instances; our-robot box numbers are mrs_buff_mk3 only. `object` is excluded.
- conf 0.5 is a shared reference point; the blob's deployed threshold is 0.6, where it still leads.

## Takeaway

1. For general robot detection (find every robot in frame, including opponents and house_bot), keep the
   **seg blob**. The pose models are not substitutes — not because they detect our robot worse, but
   because they lack the classes and the real training data for the rest of the scene.
2. The pose model's role is **keypoints on our robots**, where its box detection is already on par with the
   blob. Do not judge it on whole-scene box mAP.
3. If a single model must do both (all-robot boxes + our-robot keypoints), it needs real opponent/house_bot
   training data and those classes added, plus the synthetic-recipe fixes from the keypoint report.

Scores and plots: `assets/2026-07-14_box_pose_vs_blob/`. Reproduce with (per model, sweeping `--conf`):

```bash
python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate pose_all=data/models/yolo26n-pose_all_robots_2026-07-14_x86_64_sm89.engine \
  --labels "mr_stabs_mk2,mrs_buff_mk3,opponent" --conf 0.5 --taxonomy training/model_eval/taxonomy.yaml

python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate blob_generic=data/models/yolo26n-seg_nhrl_robots_2026-04-27_x86_64_sm89.engine \
  --labels "opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3" --conf 0.5 --taxonomy training/model_eval/taxonomy.yaml
```
