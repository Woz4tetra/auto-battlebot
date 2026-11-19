# Perception Improvement Plan

Split out from the control plan. Sensing improvements come mainly from better model training, and they
progress independently of the control law. The control plan (`control_improvement_plan.md`) treats
perception accuracy and dropout rate as fixed constraints; this plan improves those constraints.

Priority: lower than control for now. The control work can proceed against today's perception, aiming
within the current accuracy. This plan raises the ceiling.

## Why this matters

Perception accuracy sets the position tolerance the controller can meaningfully aim for, and the dropout
rate sets how long the controller must coast on prediction. Both are quantified in
`perception_reliability_may_fights.md` (six May competition fights):

- A live opponent is present in only ~48% of frames (per-fight 16-82%).
- Dropout gaps are median 56 ms but heavy-tailed: p90 338 ms, p99 1.4 s, max 5.8 s.
- Our-robot tracking is ~58% live pooled and collapses to 1.1% in one fight.

Every point of validity we recover shortens the horizon the predictor has to cover and tightens the
tolerance the controller can hit.

## Directions

1. **Better detection / keypoint model training.** The main lever. Train on more and better-labelled data,
   including the May competition fights, which are already replayable and have label tooling
   (`training/model_eval/export_labels.py`, `edit_labels.py`, `score.py`). Target the failure modes behind
   the dropouts: opponent lost under motion blur, occlusion, and unusual poses.
2. **Track-ID stability.** The `FrameIdAssigner` greedy nearest-slot assignment drives ID switches. Measure
   the true switch rate first (see instrumentation below), then tune `max_jump_distance` /
   `max_consecutive_jump_rejects` or replace the assignment.
3. **Our-robot self-tracking.** Investigate the fight where our-robot tracking collapsed to 1.1% while
   opponent tracking held at 82%. Our front/back keypoints were essentially never detected; find out why
   (bot appearance, orientation, occlusion) and fix the model or the keypoint scheme.
4. **Height / off-plane projection.** Keypoint projection assumes robots are flat on the field plane, so a
   pitched or lifted robot projects to the wrong ground position. Height compensation is a known planned
   improvement to sensing accuracy.

## Instrumentation to add

- **True track-ID-switch rate.** The reliability tool currently uses `jump_reject` count as a proxy. Log
  the live THEIRS `FrameId` set per tick in the perception diagnostics block (`src/runner.cpp:426`), then
  `training/model_eval/perception_reliability.py` can compute the real switch rate.
- Re-run `perception_reliability.py` after any model change to measure the delta against the May baseline.

## Punted from the old control plan

- Stage 0 perception-reliability measurement: done (`perception_reliability_may_fights.md`).
- "Robustify against bad sensing" split: the sensing-accuracy half lives here; the controller-side half
  (coast on last prediction with a bounded max-coast timeout, confidence gate, widen aim tolerance when
  uncertain) stays in the control plan, since operating within perception limits is a control behavior.

## Next steps

1. Add the per-slot `FrameId` logging and get a true track-ID-switch rate.
2. Re-train the detection/keypoint model on the May fights and re-measure reliability against the baseline.
3. Diagnose the self-tracking collapse fight.
