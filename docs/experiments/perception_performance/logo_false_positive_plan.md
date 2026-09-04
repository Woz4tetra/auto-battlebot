# Killing the floor-logo false positive with background subtraction and depth (plan)

**Amended 2026-09-04, after the first pass.** Three things changed and the report reflects the
amended version:

1. **Eval set is all 688 frames, not 429.** `validation_state.json` marks every frame `pass`,
   MassD included. The 429 came from the stale `.edit_state.json`. `score.py` now prefers
   `validation_state.json`, so the reviewed-frame workaround below is obsolete.
2. **Engine under test is the deployed `mixed_2026-07-31`**, not the named `2026-07-29`.
3. **A fifth and sixth arm were added**: `yolo_and_height` and `yolo_and_rgb_and_height`, gating
   on height above the fitted field plane. The first pass tested only occlusion prominence and
   wrongly concluded from it that depth cannot work at all. Height above the plane is the
   quantity `render_depth_birdseye.py` uses, and it does work.

The deployed opponent detector fires on the arena floor logo. Four arms, one detector, one
variable: what evidence outside the RGB crop is allowed to veto a box.

## Question

The detector is a 2-class box model (`robot`, `house_bot`). On the MassD 2026-08-29 arena it
puts a confident box on the "MASSACHUSETTS RESURGENCE" banner lying flat on the plywood floor,
in nearly every frame. Both extra signals available at runtime should reject it:

- The banner never moves, so a floor background built from the recording contains it.
- The banner is flat, so it stands 0 cm proud of the floor around it. A robot occludes floor
  that would have been much further away.

Does either gate remove the logo box, and what does it cost in recall?

## Reproduction

Baseline engine `yolo26n_nhrl_robots_bbox_2class_2026-07-29_x86_64_sm89.engine`, conf 0.6,
`--labels opponent,house_bot`, `taxonomy.yaml`. Agnostic level:

| eval set | frames | precision | recall | mAP50 | mAP50-95 |
|---|---|---|---|---|---|
| May NHRL fights (reviewed) | 429 | 0.923 | 0.839 | 0.821 | 0.530 |
| MassD 2026-08-29 | 98 | **0.452** | 0.589 | 0.466 | 0.304 |

Precision halves on the new arena. Recall drops too, but the precision collapse is the
headline: more than half of what the detector reports on MassD is not a robot.

A contact sheet of 12 MassD frames with GT in green and detections in red shows where those
false positives are. In 11 of 12 frames there is a box on the banner. Several frames also
carry a box on the red corner mat and on the dark mat at the far wall. The false positives are
flat printed graphics on the floor, not clutter, not crowd, not glare.

This matches the failure mode recorded in `interpret_context_vs_appearance.py`'s cut-paste
probe: the detector leans on context, and a rectangular high-contrast graphic sitting on the
arena floor is a robot-shaped context.

## Arms

One detector, run once per frame. Every arm reads the same raw boxes; the arms differ only in
which boxes survive.

| arm | gate | what it tests |
|---|---|---|
| 1. `yolo_only` | none | baseline, the model as it ships |
| 2. `yolo_and_rgb` | box must overlap the background-subtraction foreground | does "it was not there before" reject the logo |
| 3. `yolo_and_depth` | box must contain prominent pixels | does "it stands proud of the floor" reject the logo |
| 4. `yolo_and_rgb_and_depth` | both | does combining help, or just compound the recall cost |

Gate mechanics, both expressed as a fraction of the box:

- **RGB:** fraction of box pixels in the difference-image foreground mask must be
  >= `--box-rgb-fraction`.
- **Depth:** fraction of *measured* box pixels whose depth prominence exceeds
  `--prominence` metres must be >= `--box-fraction`. Measured pixels are the denominator, so a
  box the stereo could not see into is judged on what was actually observed. This is the gate
  `depth_gated_subtraction.py` already implements.

Gates only ever remove boxes. No arm can gain recall over the baseline, so recall loss is the
entire cost side of the ledger and precision gain is the entire benefit side.

## Eval sets

`training/data/nhrl_keypoints_eval_test`, scored two ways, both reported:

- **MassD 2026-08-29** (98 frames, the whole subdataset). This is where the logo is and where
  the arms are decided.
- **May NHRL fights** (429 reviewed frames, the canonical set). Regression check. A gate that
  fixes MassD by throwing away robots in the cage is not a fix.

**Reviewed-frame gotcha to handle before scoring.** `score.py` gates on the root
`.edit_state.json`, whose `reviewed` array holds 429 frames and does not include a single MassD
frame. Scoring the root today silently drops the entire recording the experiment is about. The
MassD subdataset does have all 98 labels, hand-drawn with keypoints. The workaround is to score
the subdataset directory directly, which has no `.edit_state.json` of its own, so every label
file counts. **Spot-check those 98 labels against the images before trusting any number from
them** — they were never marked reviewed and nothing has verified them.

Score all four arms in one `score.py` run per eval set so the paired bootstrap runs
(`--baseline yolo_only`, 1000 resamples). All four arms share the class order of the engine, so
`--labels opponent,house_bot` is valid for all of them.

## Measuring the logo specifically

Precision is the decision metric, but it does not name the logo. A secondary measurement does:
project every false-positive box centre into the field frame through
`camera_transforms/<stamp>.json`, and cluster. Robots move, so a real detection wanders. The
banner does not, so its false positives pile into one tight static cluster.

Report per arm: the number of static FP clusters, the frame count of the largest, and an image
crop of it. If arm 1's largest cluster is the banner and arms 2-4 shrink it toward zero, the
experiment has answered the question that was actually asked, not just moved a precision number.

## Infrastructure

Most of this exists. Reused as-is:

- `auto_battlebot/floor_background.py` — warps recording frames to a metric top-down raster and
  takes the per-pixel median. Robots move, so the median is an empty floor with the banner
  still in it.
- `auto_battlebot/background_subtraction.py` — difference image, illumination compensation,
  blob extraction.
- `auto_battlebot/camera_geometry.py` — per-frame `tf_field_from_camera`.
- `playground/cache_gt_depth.py` — replays the SVO through pyzed at `NEURAL_PLUS` and caches
  float16 depth per GT frame, joined on SVO frame index.
- `playground/depth_gated_subtraction.py` — depth prominence, the box prominence gate, and the
  `yolo_only` / `yolo_and_depth` arms.
- `training/model_eval/score.py` — `PrecomputedDetector` already scores a predictions JSON
  through the identical metric path as an engine.

Changes needed:

1. `depth_gated_subtraction.py` emits `"class_id": 0` for every detector box, collapsing
   `house_bot` into `robot`. Preserve the real class id so `--labels opponent,house_bot` maps
   correctly and the house-bot column is not fabricated.
2. Add arms `yolo_and_rgb` and `yolo_and_rgb_and_depth`. The RGB foreground mask is already
   computed in the same loop for the `rgb_only` arm; the box gate is the same shape as the
   existing prominence gate.
3. New `playground/false_positive_gates.py` for the clustering measurement above, plus the
   separability and threshold-sweep diagnostics that say whether either gate has signal at all.

## Risks

**Depth prominence has already failed once here, and the prior is bad.** An earlier pass over
this same MassD recording found the per-box prominence signal *anti-correlated* with detector
correctness, AUC 0.236 — worse than a coin flip, and in the direction that says prominent boxes
are the wrong ones. The physical story for that: these robots are 3 to 5 cm tall, at the noise
floor of stereo at 2 to 3 m, while the genuinely prominent things in frame are the arena wall,
the floor hole, and the crowd through the glass. If robots are as flat as the logo to the
stereo, arm 3 cannot work.

The occlusion-based prominence in `depth_gated_subtraction.py` is the response to that: it
measures the floor a robot hides rather than the robot's own height, which is tens of
centimetres even for a low wedge. Whether that recovers the signal is exactly what arm 3
measures. **Report the AUC again either way.** If it comes back below 0.5 a second time, that
is the result, and arm 3 should be written up as a dead end rather than retried a fourth time.

**SVO seeking may corrupt the depth cache.** `cache_gt_depth.py` calls `set_svo_position` per
frame. Seeking an SVO is known in this project to smear the decode. Gate on this before
trusting arm 3: retrieve the left image alongside depth at each seek and compare it to the GT
PNG for that stamp. If they diverge, the depth belongs to a different frame than the labels and
arm 3 is measuring noise. Fix by decoding forward with `enable_depth=False` skips rather than
seeking.

**Background subtraction needs a pose per frame and a static camera-to-field chain.** Frames
whose transform method is `ambiguous` or `unavailable` have no usable pose and drop out. Report
how many MassD frames survive; if the count is small, the arm-2 numbers are on a different
frame set than arm 1 and cannot be compared directly.

**Neither gate is free at runtime, and neither is in the 60 ms budget yet.** This experiment
grades offline accuracy only. Cost lands in a follow-up.

**The engine under test is not the deployed one.** `config/_desktop.toml` and
`config/_jetson.toml` ship `yolo26n_nhrl_robots_bbox_2class_mixed_2026-07-31`, not the
`2026-07-29` engine named here. Score the deployed engine as a fifth baseline row so the report
says whether the logo failure is specific to the older weights.

## Decision rules, fixed before running

- **Adopt a gate** if it raises MassD agnostic precision with a 95% CI excluding 0, and costs
  less than 0.05 agnostic recall on the May reviewed set.
- **Reject a gate** that cannot clear the logo cluster, whatever it does to aggregate precision.
  Removing scattered false positives while leaving the banner is not the fix being asked for.
- **Prefer the simpler arm on a tie.** If arm 2 and arm 4 are within noise of each other, arm 2
  wins: background subtraction needs no stereo, no depth mode, and no second engine pass.
- A recall cost above 0.05 on May is disqualifying even with a large precision gain. The
  pipeline coasts through detection dropouts already, and the opponent track is live in under
  half of frames.

## Deliverables

1. This plan.
2. `depth_gated_subtraction.py` extended with the two RGB-gated arms and correct class ids.
3. `playground/false_positive_gates.py`.
4. Depth cache for the MassD subdataset, with the seek-integrity check reported.
5. `score.py` output for all four arms on both eval sets, with paired-bootstrap tables.
6. Annotated contact sheets, one per arm, on the same 12 MassD frames.
7. A report, `logo_false_positive_<date>.md`, opening with the reproduction table above.
