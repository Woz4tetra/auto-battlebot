# Does model size matter for the keypoint model - or does the corpus? - 2026-09-07

`yolo26{n,s,x}-pose` trained 200 epochs on `our_robot_keypoints` (31,912 train / 3,530 val,
2 classes) as arms D, E and F, against the same three sizes trained on `all_robot_keypoints`
as arms A, B and C, carried over from `pose_model_size_2026-09-05.md`. Batch 96, imgsz 640,
seed 0, `--save-period 50`, every arm cold-started from `yolo26<size>-pose.pt`. Scored on
`nhrl_keypoints_eval_test` (688 frames, 8 recordings) with `score.py`, four confidences,
paired bootstrap 1000x.

Supersedes the corpus question left open by `pose_model_size_2026-09-05.md`, whose reference
model `yolo26n-pose_our_robots_2026-05-01` was trained on a different corpus than every arm
it was compared against.

## Headline

1. **The corpus barely matters, and the effect that looked large was an artifact.** Scored
   the way the plan registered, arm D beat arm A on heading by 4.45 deg at the operating
   point. Scored on our own robots only, the gap is 1.47 deg, `ns` at conf 0.05 and 0.3, and
   points the *other* way at conf 0.05.
2. **The artifact is in the taxonomy, and it invalidates part of the previous report.**
   `taxonomy_keypoint.yaml` excludes `house_bot` and `object` but keeps `opponent`, and 763
   of the eval set's 1,433 scoreable boxes are `opponent`. A 2-class model cannot emit one.
   Every arm in the previous sweep was graded on a set where 53% of the targets were
   opponents, which is not the keypoint model's job.
3. **Three of `pose_model_size_2026-09-05.md`'s conclusions do not survive the fix.**
   `yolo26s-pose` is not worse than `n`; `yolo26x-pose` passes criterion (a) at all four
   confidences rather than failing at the operating point; and the deployed model does not
   beat the all-robots arms, it loses to plain arm A by 2.34 deg.
4. **Arm D is a better model than the one deployed**, by 3.81 deg of heading at conf 0.5 on
   the same corpus. It is not a controlled comparison: the deployed model ran 500 epochs,
   fine-tuned from a lost parent, annealed to `lrf 0.01`, and trained with `flipud 0.5`.
5. **Criterion (c) fails under both scorings**, so the registered rule says do not switch the
   corpus on this evidence.

## Setup

| | |
|---|---|
| Corpora | `our_robot_keypoints` 31,912 train / 3,530 val, 2 classes, 98.7% synthetic; `all_robot_keypoints` 18,447 / 2,049, 3 classes, 97.8% synthetic |
| Shared | All 497 real frames appear in both, and are the only real frames in either |
| Arms | A/B/C = `n`/`s`/`x` on `all_robot_keypoints`; D/E/F = the same three on `our_robot_keypoints` |
| Schedule | 200 epochs, batch 96, imgsz 640, `--save-period 50`, seed 0, `lrf 0.1`, `flipud 0.0`, `close_mosaic 0` |
| Endpoint | epoch 200 (`last.pt`) for every arm; `best.pt` is unused, val does not measure generalization here |
| Eval | `nhrl_keypoints_eval_test`, 688 frames / 8 recordings, paired bootstrap 1000x |
| Hardware | megamind, 3x RTX A6000 sm86, via `training/gpu_queue.py` |
| Run dirs | `runs/projects/auto_battlebots_2026-09-07_01-01-14_yolo26n-pose` (D) |

D and A are matched on every training argument. Both read
`model: yolo26n-pose.pt, epochs 200, batch 96, imgsz 640, lr0 0.01, lrf 0.1, seed 0,
flipud 0.0, close_mosaic 0, weight_decay 0.0005, fraction 1.0` out of their checkpoints, and
differ only in `data:`. That is the control the previous report lacked.

```bash
Q="venv/bin/python training/gpu_queue.py"
$Q submit --name D_n_our --by claude-pose-corpus -- \
  venv/bin/python training/yolo/train.py training/data/our_robot_keypoints yolo26n-pose \
    -d 0 1 2 -b 96 -e 200 --save-period 50
```

Arm D took 5.94 h.

## The scoring bug, which turned out to matter more than the question

The plan called for one invocation over a grid mixing 2-class and 3-class engines, which
`score.py` could not do: it passed one global `--labels` to every candidate, and
`len(class_labels)` is what sets `num_classes` and so where the keypoint values start in the
raw tensor. A 2-class engine read with three labels parses to `num_keypoints=0`.
`--candidate-labels NAME=a,b` fixes that, mirroring the existing per-candidate `--stretch`.
Every engine now prints its own layout line, and the grid runs in one pass with a paired
bootstrap across corpora.

Fixing that exposed a second problem the plan did not anticipate. `taxonomy_keypoint.yaml`
excludes `house_bot` and `object`. It does not exclude `opponent`. The eval set's scoreable
ground truth is:

| label | boxes | boxes with a visible keypoint |
|---|---:|---:|
| `mrs_buff_mk3` | 670 | 670 |
| `opponent` | 763 | 763 |
| `mr_stabs_mk2` | 0 | 0 |

Every opponent box carries keypoints, and keypoint matching in `score.py` is class-blind, so
a 3-class arm is scored on opponents and a 2-class arm is not. That makes the matched-box
count, recall, and every keypoint mean incomparable across the two rows of the grid. It also
means the previous report's caveat, "with `house_bot` and `object` excluded, the keypoint
metrics rest on the `mr_stabs_mk2` and `mrs_buff_mk3` boxes only", was wrong: they rested on
a set that was 53% opponent, and there is not one `mr_stabs_mk2` box in the eval set at all.

`taxonomy_keypoint_ours.yaml` adds `opponent` to the exclusion list. That scores every arm on
our own robots, which is the deployment task: the keypoint model estimates our robot's
heading so the aim assist knows which way it is facing, and the opponent belongs to the other
branch. Both scorings are reported below. The ours-only one was added after seeing the
matched-box counts, so it is a post-hoc analysis choice; the reason for it is structural
rather than empirical, and it does not favour arm D. It favours arm C.

Precision is not comparable in the ours-only scoring. A 3-class arm's opponent detections
become false positives once opponent ground truth is excluded, so A/B/C read artificially
low. Recall, matched-box count and the keypoint metrics are comparable; precision is not.

## The primary question - arm D against arm A

Both scorings, conf 0.5, the deployed operating point.

| scoring | A heading | D heading | delta | 95% CI | matched boxes |
|---|---:|---:|---:|---|---|
| as registered | 10.447 | 5.999 | -4.448 | [-6.590, -2.450] | 492 -> 405 |
| our robots only | 7.194 | 5.726 | **-1.468** | [-2.646, -0.500] | 410 -> 403 |

Two thirds of the apparent gain was arm A being charged for opponents it places badly. What
survives is 1.47 deg.

### Our robots only, all four confidences

| conf | metric | A | D | delta | 95% CI | verdict |
|---|---|---:|---:|---:|---|---|
| 0.05 | kp_heading_err_deg | 10.000 | 11.122 | +1.122 | [-1.145, +3.569] | ns |
| 0.05 | kp_pck@0.1 | 0.662 | 0.644 | -0.019 | [-0.051, +0.014] | ns |
| 0.30 | kp_heading_err_deg | 8.334 | 7.783 | -0.551 | [-2.428, +1.387] | ns |
| 0.30 | kp_pck@0.1 | 0.700 | 0.683 | -0.017 | [-0.051, +0.018] | ns |
| 0.50 | kp_heading_err_deg | 7.194 | 5.726 | -1.468 | [-2.646, -0.500] | **better** |
| 0.50 | kp_pck@0.1 | 0.727 | 0.716 | -0.011 | [-0.043, +0.023] | ns |
| 0.60 | kp_heading_err_deg | 6.045 | 4.679 | -1.365 | [-2.630, -0.396] | **better** |
| 0.60 | kp_pck@0.1 | 0.754 | 0.744 | -0.009 | [-0.046, +0.030] | ns |

**The ranking flips between confidences.** D is 1.12 deg worse at conf 0.05 and 1.47 deg
better at conf 0.5. Keypoint placement, which heading is derived from, is `ns` everywhere and
slightly negative at every threshold. Whatever the corpus buys, it is not better keypoints;
it is a confidence ordering that puts D's good detections above the gate.

### Per recording, conf 0.5, our robots only

| recording | A heading | D heading | delta | A boxes | D boxes |
|---|---:|---:|---:|---:|---:|
| `main_2026-05-01_17-42-20` | 6.34 | 6.83 | +0.49 | 79 | 75 |
| `main_2026-05-02_10-06-02` | - | 4.59 | - | 0 | 3 |
| `main_2026-05-02_11-45-05` | 6.47 | 6.06 | -0.41 | 79 | 74 |
| `main_2026-05-02_14-12-25` | 6.86 | 5.23 | -1.64 | 63 | 68 |
| `main_2026-05-02_15-35-00` | 5.52 | 5.13 | -0.39 | 75 | 76 |
| `main_2026-05-02_16-18-05` | 3.31 | 2.19 | -1.11 | 12 | 18 |
| `main_2026-05-02_17-26-12` | 14.02 | 7.39 | **-6.63** | 53 | 44 |
| `mrs_buff_mk3_massd_ns_jetson_2026-08-29` | 6.30 | 4.96 | -1.34 | 49 | 45 |

D wins 6 of the 7 recordings where both models find robots. The size of the win is one
recording: drop `17-26-12` and the average delta is -0.73 deg. `17-26-12` is also where arm A
is worst by a wide margin, 14.02 deg against 3.3 to 6.9 everywhere else, so the corpus is
mostly recovering one bad recording rather than lifting the set.

Note that `main_2026-05-01_17-42-20` scores here but returns "no scoreable labels" when
pointed at directly. `reviewed_stems` prefers a subdataset's older `.edit_state.json` over
the root `validation_state.json`, which is the trap its own docstring warns about. Per-
recording numbers must be produced with the root validation state copied alongside each
recording, or they will not reconcile with the aggregate.

### Decision rule, as registered

- **(a) heading improves at conf 0.5 with a 95% CI excluding 0.** **Passes** under both
  scorings, -4.448 [-6.590, -2.450] as registered and -1.468 [-2.646, -0.500] on our robots.
- **(b) `kp_pck@0.1` at conf 0.5 does not get worse by a CI excluding 0.** **Passes** under
  both, +0.024 `ns` as registered and -0.011 `ns` on our robots.
- **(c) the matched-box count at conf 0.5 does not fall.** **Fails** under both. 492 -> 405
  as registered, which is structural. 410 -> 403 on our robots, a fall of 7 boxes, which is
  not structural and is what the criterion was written to catch.

All three were required. **The registered verdict is: do not switch the keypoint model to
`our_robot_keypoints` on this evidence.** The heading gain is real at the operating point and
small, it disappears at lower thresholds, keypoint placement does not improve, and the
matched-box count moves the wrong way.

## What the fix does to the previous report

Same engines, same eval set, same bootstrap. Only the taxonomy changed. Heading against
arm A at conf 0.5:

| candidate | as registered | our robots only |
|---|---|---|
| B (`s`, all_robots) | +2.475 [+0.400, +4.683] **worse** | -0.466 [-1.808, +0.789] ns |
| C (`x`, all_robots) | -1.359 [-3.631, +0.729] ns | -2.118 [-3.569, -0.931] **better** |
| D (`n`, our_robots) | -4.448 [-6.590, -2.450] better | -1.468 [-2.646, -0.500] better |
| deployed | -0.016 [-2.955, +2.501] ns | +2.342 [+0.471, +4.342] **worse** |

Three conclusions in `pose_model_size_2026-09-05.md` rest on the left column:

- **"Do not deploy `yolo26s-pose` under any circumstances. It is dominated on both axes."**
  On our robots `s` is `ns` on heading at every confidence and `better` than `n` on pixel
  error at conf 0.5 and 0.6 and on PCK at 0.6. It is still not worth 1.29x the latency, but
  it is not dominated.
- **"`x` is `ns` on heading at the operating point, so criterion (a) is not met."** On our
  robots `x` passes (a) at all four confidences: -3.808, -2.887, -2.118, -1.176, every CI
  excluding 0. The accuracy case for `x` is stronger than the report concluded. Only latency
  stops it, which is the same verdict for a different reason.
- **"`x` is the first all-robots pose model to beat the deployed `our_robots` model."** Plain
  arm A already beats it, 7.194 against 9.535 deg, CI excluding 0. That also puts
  `all_robots_pose_2026-07-14.md`'s negative result in question, since it graded the same way.

## Arm D against the deployed model

Same corpus, different schedule and lineage. Conf 0.5, our robots only:

| | boxes | kp_err_px | PCK@0.1 | heading deg | recall |
|---|---:|---:|---:|---:|---:|
| D | 403 | 9.196 | 0.716 | 5.726 | 0.601 |
| deployed | 494 | 9.677 | 0.713 | 9.535 | 0.737 |

D places keypoints no better, finds 0.136 less of our robots, and reads heading 3.81 deg
better. The two differ in four ways at once, from the deployed checkpoint's own `train_args`:

| | D | deployed |
|---|---|---|
| epochs | 200 | 500 |
| parent | `yolo26n-pose.pt` | `yolo26n-pose_our_robots_2026-04-24.pt`, not on the repo or the archive |
| `lrf` | 0.1 | 0.01, fully annealed |
| `flipud` | 0.0 | 0.5 |

`flipud 0.5` is the one that should be suspected first for a heading-specific gap. A vertical
flip is exactly the transform that corrupts a front-to-back vector, and the deployment ZED
has a fixed up-vector and never sees the arena inverted. This is a hypothesis the experiment
did not test, not a finding.

## Where the plateau is on this corpus

Arm D's ep100 against its ep200, our robots only. `boxes` is the matched count the keypoint
metrics average over.

| conf | ckpt | boxes | kp_err_px | PCK@0.1 | heading deg | recall | precision |
|---|---|---:|---:|---:|---:|---:|---:|
| 0.05 | ep100 | 585 | 10.845 | 0.649 | 14.342 | 0.873 | 0.569 |
| 0.05 | ep200 | 546 | 11.010 | 0.644 | 11.122 | 0.815 | 0.871 |
| 0.50 | ep100 | 509 | 9.441 | 0.704 | 7.669 | 0.760 | 0.919 |
| 0.50 | ep200 | 403 | 9.196 | 0.716 | 5.726 | 0.601 | 0.988 |

The previous report's plateau claim reproduces: pixel error and PCK are flat from epoch 100
to epoch 200 at the low confidence floor, 10.845 -> 11.010 px and 0.649 -> 0.644. Heading
improves 3.2 deg even at conf 0.05, more than the previous sweep saw, but the matched-box
count falls with it.

The new finding is what the second hundred epochs costs. **ep100 finds far more of our
robots**: recall 0.873 against 0.815 at conf 0.05, and 0.760 against 0.601 at conf 0.5. For
an aim assist that has to track a robot every frame, 0.16 of recall is a large price for
1.9 deg of heading. If arm D's corpus is ever shipped, ep100 is the checkpoint to look at
first, and choosing between them is a deployment question rather than a metric question.

## Caveats

- **The ours-only taxonomy is a post-hoc choice.** It was written after seeing that a 2-class
  arm cannot match an opponent box. The justification is structural, and it does not favour
  the arm this experiment was built to test, but it was not registered in advance.
- **The corpus contrast still varies two things**, class vocabulary and 1.73x the training
  frames. Arm G, `--fraction 0.578`, is the control that separates them and has not been run.
  Given that the corpus effect is 1.5 deg at one threshold and absent at two others, there
  may not be enough effect left to attribute.
- **Single seed per arm.** The plan registered that a D-against-A heading gap under a couple
  of degrees is `ns` in practice on one seed each. The measured gap is 1.47 deg.
- **The win rests on one recording.** Six of seven recordings favour D by 0.4 to 1.6 deg; the
  seventh accounts for the rest.
- **Neither val split is scene-disjoint.** Both are random frame-level carves of
  synthetic-dominated corpora, now recorded in both datasets' `README.md`. Arm D reached
  0.992 box mAP50 and 0.982 pose mAP50-95 on val while scoring 0.716 PCK on the eval set.
- **The eval set has no `mr_stabs_mk2` at all**, so every keypoint number in this report is
  `mrs_buff_mk3`, 670 boxes before thresholding. The class-balance argument for
  `our_robot_keypoints` in the plan, that its minority class carries more boxes, cannot be
  tested here.
- **The deployed model is still not a controlled comparison**, and arm D controls its corpus
  but not its schedule, its parent or its augmentation.
- **Engines are sm86**, built and scored on megamind.
