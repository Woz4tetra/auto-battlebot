# Does rembg find robots from a fixed overhead cage camera?

Status: **planned** (2026-09-04). Nothing built, nothing run.

Follow-up to `rembg_field_2026-09-04.md`, which answered the same question on the moving ZED
footage with a no: `isnet-general-use` found 3% of robots at IoU 0.5. This plan reruns the one
pose-free arm of that experiment on NHRL's fixed overhead cage cameras, with the stationary camera
letting the baseline be built properly this time.

## Why rerun a null result

The first run's own per-recording table is the reason. Its one recording shot high over the wall
with a clean floor and no crowd in frame (`main_2026-05-01_17-42-20`) scored recall 0.12 at IoU
0.5 against 0.02 for the seven low, cluttered ones. One recording cannot separate viewpoint from
resolution or luck, and the report said so. The NHRL `Cage-*-High` cameras are that viewpoint
everywhere, so they are the direct test of whether the clean-overhead number was a view effect.

Three of the first run's failure modes are also structurally different from up here:

| failure in run 1 | why cage-high changes it |
|---|---|
| the crowd is the salient object (rows 2, 5, 6 of `sheet_raw_mattes.jpg`) | overhead cameras look down at the floor; the crowd is behind the polycarbonate and mostly out of frame |
| robots reach the network at 69 px | median GT box is 191 px here, **102 px at the network**, 1.5x larger |
| the baseline was contaminated (MassD median had the robot baked in) | a fixed camera gives a clean per-scene median over hundreds of frames with no pose and no warp |

The fourth failure mode, **the floor itself is salient**, is the one this view may make worse. The
floor fills more of the frame from overhead, and a bigger uniform region is a stronger saliency
candidate. That is the risk the plan is built to measure rather than dodge.

## What is dropped, and what replaces it

**`rembg_warped` and `rembg_geom` are gone.** Both need a camera pose per frame, and no poses exist
for NHRL broadcast video. A bird's-eye arm could be rebuilt without a pose by fitting a homography
from the DeepLab hull's four corners to the nominal square, and on a fixed camera that fit would be
computed once per scene. It is deliberately not done: `rembg_warped` was the worst arm in run 1
because the raster is a bright square on black and rembg returns the square. A pose-free version
would have the same input and the same answer.

**`rembg_deeplab` is the arm.** Untouched frame in, `isnet-general-use` matte out, intersected after
inference with the convex hull of the largest DeepLab floor component. Same code, same masking
order, same reason for the order (pre-masking hands the network a trapezoid to return).

**The baseline changes from weak to strong.** Run 1 had to build its floor median from ~100 GT
frames through a pose-based warp. Here the camera does not move, so the baseline is the plain thing:
per-scene per-pixel median over frames spread across the fight, identity warp, `subtract`,
`find_blobs`. It needs no pose either, which makes it the fair pose-free comparison this time. Its
field mask is the DeepLab hull of the *median image*, which has no robots on it and therefore no
robot-shaped holes for the hull to repair: the cleanest hull this experiment can produce.

## Dataset: `background_subtraction_eval_test`

Built from `training/data/nhrl_robots_bbox_2class`, scenes whose name matches `Cage-.*High`.
This is the dataset the very first version of this plan proposed and then dropped when the
experiment moved to the ZED set; it comes back because it is the only labelled cage-high data.

### The 22 scenes

All 22 are **1920x1080**. Twenty of them carry `_720p_` in the name and are not 720p; the name
records the source stream label, not the exported frame size. Frames on disk are every 10th source
frame (every 20th for two scenes), so consecutive dataset frames are 0.33 s apart at 30 fps.

| # | scene | frames | split | stride | p95 step px | drift px | median robot px |
|---|---|---|---|---|---|---|---|
| 1 | clyde-colossus Cage-2-OH 2026-02-07 | 1,333 | train | 10 | 0.2 | 3.5 | 247 |
| 2 | chinchilla-sphinx Cage-5-OH 2026-03-07 | 1,286 | train | 10 | 0.5 | 0.7 | 155 |
| 3 | BZ apr25 Cage-6-OH (57d6) | 1,281 | train | 10 | 0.3 | **36.9** | 174 |
| 4 | BZ oct24 Cage-6-OH (22a9) | 1,230 | train | 10 | 0.8 | 0.5 | 179 |
| 5 | BZ may25 Cage-6-OH (a50a) | 1,174 | train | 10 | 0.5 | 0.8 | 223 |
| 6 | Cage-6-OH 2025-10-04 ThePowerOfFriendship | 1,121 | train | 10 | 0.5 | 0.4 | 175 |
| 7 | Cage-2-OH 2025-11-01 ChainsawKitty | 1,046 | train | 10 | 0.5 | 3.0 | 209 |
| 8 | BZ feb25 Cage-2-OH (e1c9) | 1,001 | train | 10 | 1.4 | 0.3 | 223 |
| 9 | Cage-2-OH 2025-12-06 Jellybaby | 999 | train | 10 | 0.3 | 0.5 | 213 |
| 10 | Cage-2-Blue-High 2025-10-04 | 917 | train | 10 | 0.4 | **23.0** | 177 |
| 11 | Cage-2-Red-High 2025-10-04 MiniTouro | 871 | train | 10 | 1.2 | **18.2** | 193 |
| 12 | snowdrift-badluck Cage-6-OH 2025-05-03 | 649 | train | 20 | 0.3 | 0.4 | 150 |
| 13 | BZ sep24 Cage-6-Red-High (c40f) | 605 | train | 10 | 0.6 | **8.8** | 259 |
| 14 | Cage-6-OH 2025-11-01 PrettyFly | 579 | train | 10 | 0.4 | 0.5 | 193 |
| 15 | Cage-5-OH 2025-11-01 SupremeRuler | 564 | **val** | 10 | 0.4 | 0.2 | 247 |
| 16 | BZ W-13 may25 Cage-5-Red-High | 536 | train | 10 | 0.7 | 2.3 | 155 |
| 17 | BZ jun24 Cage-6-Red-High (857f) | 519 | train | 10 | 0.5 | **23.0** | 169 |
| 18 | Cage-5-OH 2025-10-04 LittleMan | 465 | **val** | 10 | 0.7 | 0.4 | 273 |
| 19 | clyde-verticalizer Cage-2-Red-High 2025-11-01 | 446 | train | 20 | 0.4 | 2.6 | 249 |
| 20 | Cage-6-OH 2025-04-05 MellonBaller | 373 | **val** | 10 | 0.4 | 0.2 | 157 |
| 21 | BZ dec24 Cage-1-NE-High (b75b) | 319 | **val** | 10 | 0.2 | 0.2 | 93 |
| 22 | BZ feb26 cole-anubis W-53 Cage-2-OH | 300 | train | 10 | 0.3 | 0.4 | 223 |

Totals: **17,614 frames, 57,525 boxes** (40,084 `robot`, 17,441 `house_bot`), zero empty-label
frames, three boxes per frame in 75% of them (two robots and the house bot). Every frame on disk
already passed the source dataset's manual validation; the 2,018 cage-high frames that failed it
were removed from the source before this, so no further filtering is needed.

"p95 step" is the 95th percentile phase-correlation displacement between consecutive dataset
frames at full resolution; "drift" is first frame against last. Three `Cage-*` scenes without
`High` in the name (`Cage-6-Red`, `Cage-6-Blue`, `Cage-6-Red june25`) are excluded by the name
rule; `Cage-6-Blue` also drifts 215 px and is not a fixed camera.

### Stability: static frame to frame, five one-time repositions

Every scene holds under 1.7 px between consecutive frames, so all 22 are fixed cameras in the
sense that matters per frame. Five scenes (3, 10, 11, 13, 17) show a first-to-last drift of 9 to
37 px with tiny step sizes, which is a single bump or re-aim somewhere in the fight, not a pan.
Scene 11 has one 16 px step, which is where its bump is.

**rembg does not care.** It has no temporal state; a moved camera is a different frame and nothing
more. So the drift does not select scenes for the arm, and all 22 are in.

**The median baseline does care**, because a bumped camera puts the pre-bump floor 20 px off the
post-bump frame and the whole field lights up as difference. So the baseline aligns each frame to
the scene's reference frame by phase correlation (translation only, which is what a bump is)
before subtracting. That is one `cv2.phaseCorrelate` and one `warpAffine` per frame. The five
drifting scenes get reported separately in the baseline's own table so a residual alignment
failure is visible as a per-scene outlier instead of a pooled loss.

### Sampling

**1,000 scored frames plus 220 tuning frames.**

- **Tuning:** the first 10 frames of each scene by frame index, 220 in all, matching
  `TUNING_FRAMES_PER_RECORDING = 10` from run 1. Held out of scoring. The first frames of a fight
  are the countdown, robots parked in their squares; that is fine for tuning a per-frame method
  and it is exactly the frame the baseline median must not be built from.
- **Scored:** 1,000 frames drawn from the remainder, proportional to scene length with a floor of
  20 per scene, seeded. Proportional allocation runs from about 20 (scenes 21, 22) to about 76
  (scene 1), so no scene dominates and none is a single-digit sample. Realised counts are written
  into the dataset README.

### Layout, and two constraints inherited from the code

```
training/data/background_subtraction_eval_test/
    README.md                     scene table above, realised counts, seed, stability CSV
    <scene_short_name>/
        data.yaml                 names: [robot, house_bot], identical in every scene
        images/<int>.jpg          hardlinked from the source (os.link, per the runbook)
        labels/<int>.txt          copied YOLO rows
```

**Integer stems are mandatory.** `score.py:191` does `int(label_path.stem)` and `rembg_field.py`
sorts stems with `key=int` and exposes `int(stem)` as the frame key, so a stem like
`clyde-colossus__frame_004990` breaks both. Stems become `scene_number * 1_000_000 + frame_index`
written without leading zeros (scene 7, source frame 3410 is `7003410`). Scene numbers 1 to 22 and
frame indices under 17,500 keep the keys unique across scenes and monotone within one, which is
what the tuning-frame selection relies on.

**Class names must match the GT.** Run 1 stamped `opponent` on every rembg detection because that
was a GT class there. Here the GT classes are `robot` and `house_bot`, so `LABEL` becomes `robot`
(or a parameter). An unknown label passes `score.py`'s check with a warning and then scores every
detection as a false positive, which would look like a method failure and is not one.

The source dataset is not modified. The new dataset carries no `validation_state.json` of its own,
because every frame in it passed validation by construction; the scoring root that `predict`
writes pins `score.py` to the scored frames as before.

## The YOLO reference is contaminated on 18 of 22 scenes

`yolo26n_nhrl_robots_bbox_2class_2026-07-29` was trained on the `train` split of this very
dataset. Eighteen cage-high scenes are in that split. Scoring the engine on them reports training
fit, not detection, and every other 2class engine in `data/eval_models/` shares the corpus.

So the YOLO row is computed **on the four `val` scenes only** (15, 18, 20, 21: 1,721 source
frames, roughly 100 of the 1,000 scored) and labelled as such. It is a reference on a subset, not
a bootstrap comparison, the same status it had in run 1. rembg and the median baseline have no
training set and are scored on everything.

## Code changes

Everything reuses `rembg_field.py`; the changes are what a pose-free, jpg, non-ZED dataset needs.

1. `enumerate_frames`: glob `*.jpg` as well as `*.png`. `load_frame_geometry` already returns
   `None` when `camera_info.json` is absent, so frames come back unposed without a code path.
2. `scored_frames` and `tuning_frames`: drop the `posed` requirement when the run has no posed
   arm. `ARMS` for this run is `("rembg_deeplab",)`; `cache` skips the raster pass.
3. `write_scoring_root`: use the frame's real extension instead of the hardcoded `.png`.
4. `LABEL`: `robot` for this dataset (see above).
5. New `training/model_eval/make_cage_high_eval_dataset.py`: scene regex, stability
   measurement to CSV, tuning/scored split, sampling, integer stems, hardlinks, per-scene
   `data.yaml`, README.
6. New `training/model_eval/stationary_bgsub_predict.py`: per scene, 60 frames evenly spaced
   across the non-tuning frames, phase-correlation alignment to the scene's reference frame,
   `build_median_background` with identity warps, DeepLab hull of the median as `valid_mask`,
   `subtract` and `find_blobs` with the same `SubtractionParams` defaults as
   `background_subtraction_predict.py`, confidence by the same `SCORE_REFERENCE_DIFF` convention,
   predictions JSON out.
7. `rembg_field_render.py`: `ArmRenderer` for the one arm, title cards per scene, and a fourth
   panel showing the baseline's difference image on the same frame so the two methods are
   compared on the same pixels.

## Tuning

Same procedure as run 1: cache mattes for the 220 tuning frames once, sweep post-processing on the
cached arrays, pick by F1 at IoU 0.5, freeze, keep the whole surface.

Two grid changes, both from run 1's surface:

- **Threshold: 96 to 248 step 8** (20 values). Run 1 swept 64 to 240 step 16, found the surface
  flat below 128, and chose 208 to 224 near the top of the range. The new grid drops the dead
  bottom and resolves the top at twice the density.
- **`min_area`: 100 to 25,600, doubling** (9 values). Robots here are 1.5x wider than on the ZED
  set and cover 2 to 4x the pixel area, and run 1's grid stopped at 12,800.

Overlap fraction stays 0.1 to 0.9 step 0.1. Run 1 found it nearly inert (F1 0.074 to 0.076 across
the range) because components were wholly in or wholly out; it stays in the sweep to confirm that
holds from overhead, where the near wall is a much smaller part of the frame.

The baseline's `SubtractionParams` (threshold 35, `min_area` 400, open 5, close 15) are the
tuned NHRL values from `auto_battlebot/background_subtraction.py` and are **not** swept. Sweeping
one method and not the other would tilt the comparison; the baseline is scored with the numbers it
shipped with, and the write-up says so.

## Scoring: IoU 0.3 and 0.5

Two `score.py` runs into separate output dirs, agnostic level, paired bootstrap 1,000 resamples,
`--conf 0.0` for the saliency and subtraction candidates (their operating point is the tuned
threshold), YOLO in its own run at `--conf 0.5` on the val-scene scoring root.

```bash
for iou in 0.3 0.5; do
  venv/bin/python training/model_eval/score.py training/data/eval_results/rembg_cage_high/scoring_root \
    --candidate bgsub_stationary=.../bgsub_stationary.json \
    --candidate rembg_deeplab=.../rembg_deeplab.json \
    --labels robot,house_bot --iou "$iou" --conf 0.0 --baseline bgsub_stationary \
    --output training/data/eval_results/rembg_cage_high/iou${iou}
done
```

### Decision rule, registered before running

Run 1 gives the numbers to beat, so the outcome is decided in advance:

- **Viewpoint hypothesis.** `rembg_deeplab` recall at IoU 0.5 on cage-high must clear **0.12**,
  the clean-overhead recording's number from run 1, to say the view helped at all. Below that, the
  1080p recording was luck and the view was never the problem.
- **Usefulness.** `rembg_deeplab` must not be significantly worse than `bgsub_stationary` on recall
  at IoU 0.5 (paired bootstrap CI on the delta includes or exceeds zero). A pose-free method that a
  pose-free median beats has no reason to exist in this stack.

Passing the first and failing the second is the likely result and is still worth having, because
it would pin the failure to the floor-saliency mode rather than the view. Failing the first closes
rembg as a direction for good. Passing both would be the first positive result for saliency here
and would earn a Jetson timing run, which run 1 estimated at several times the 60 ms budget.

The IoU 0.3 column separates detection from localization as before. Run 1 saw recall barely move
between the two for rembg (0.05 to 0.03), meaning the loss was detection; if that gap opens up
here, robots are being found and boxed loosely, which is a different and more fixable problem.

## Video clips

**Flip-books, one per method.** `rembg_deeplab` and `bgsub_stationary`, each running through all
22 scenes in frame order with a title card per scene, 4 fps, four panels: frame with GT and
predictions, raw matte (or raw difference image), matte after the hull with the hull outline, and
the other method's output on the same frame. As in run 1, the raw panel is the one that tells "the
mask deleted it" from "the network never found it".

**Stability from the dataset itself.** The source videos are not on this machine or in the
archive, so run 1's SVO-window script cannot be used. But the dataset frames are every 10th source
frame, which is 3 fps of real contiguous footage, coarse but consecutive. For two scenes (one
Overhead-High, one Red-High), every frame in the scene runs through both methods in order, and the
clip and the carried-detection count (IoU 0.5 against the previous frame) are reported at that
rate. Run 1 measured 43% and 72% carried at 30 fps; at 3 fps the robots move further between
frames, so the number is not directly comparable and is labelled as a 3 fps figure.

Output to `docs/experiments/perception_performance/assets/<date>_rembg_cage_high/`, H.264, 960x540,
a few MB each, as run 1's were.

## Risks

**The floor is a bigger, cleaner salient object from overhead.** Run 1's dominant failure was the
whole floor polygon coming back at alpha 128+. From above, the floor is a larger fraction of the
frame with straighter edges, which is a stronger candidate for "the object". The threshold grid is
dense at the top for this reason, and the raw-matte sheet is the first thing to look at. If the
matte is the floor on most frames again, the result is the result.

**DeepLab may or may not have seen these frames.** `training/deeplab/field_labels.py` parses cage
numbers out of exactly these BrettZone and `Cage-N-...` filename patterns, so the field corpus is
drawn from the same archive and Cages 2, 5 and 6 are well represented in it. Whether these
specific 22 scenes were in the training set cannot be checked here: the corpus manifest
(`field_index.json`) is not on this machine. The hull is a mask, not the thing being graded, so
in-sample is acceptable, but the write-up has to state the uncertainty rather than claim
generalisation.

**Cage-1 is unseen by DeepLab.** `deeplab_field_data_plan.md` records Cage-1 as one training frame
and calls it dead as a field. Scene 21 (`Cage-1-NE-High`) is the one cage-high scene where the
hull may be wrong, and it also has the smallest robots (93 px median). Its hull gets eyeballed
first, and if DeepLab fails there the scene is scored with the hull it produces and flagged, not
dropped, because a fixed camera in a cage DeepLab has never seen is a realistic deployment case.

**The baseline is now strong enough to be the whole story.** With a clean median, no pose and no
warp, `bgsub_stationary` on a fixed camera is close to the best-case background subtraction. If it
scores near YOLO on the val scenes, the more interesting finding of this experiment is about the
baseline, and the write-up should say that plainly rather than bury it under the rembg result.

**Alignment can fail on the five drifted scenes.** Phase correlation on a frame where a robot
spins in the middle can lock onto the robot instead of the cage. The baseline's per-scene table
exposes that as an outlier on scenes 3, 10, 11, 13 or 17; if it happens, the fallback is a median
per segment split at the bump, not a global loosening of the threshold.

## Steps

1. `make_cage_high_eval_dataset.py`: build the dataset, write the stability CSV and README, check
   integer stems round-trip through `score.py`'s `load_gt` on the result.
2. Code changes 1 to 4 in `rembg_field.py` and `rembg_field_predict.py`. Confirm `enumerate_frames`
   returns 1,220 frames, 220 tuning, all unposed.
3. Run DeepLab over the set, hull, cache. Contact sheet of raw mask against hull for three frames
   per scene, scene 21 first. Also hull the 22 median images for the baseline and put them beside
   the per-frame hulls on the same sheet; disagreement between the two is a DeepLab error made
   visible.
4. `cache`: one rembg pass over all 1,220 frames (about 105 ms each on the A6000 at 1080p, a few
   minutes).
5. `tune` on the 220 tuning frames with the new grids. Freeze parameters.
6. `predict`: `rembg_deeplab` predictions and the scoring root over the 1,000 scored frames.
7. `stationary_bgsub_predict.py`: medians, alignment, predictions on the same 1,000 frames.
8. Score at IoU 0.3 and 0.5. YOLO on the val-scene subset in its own run.
9. Flip-books, the two 3 fps stability clips, contact sheets (raw mattes, missed robots, hull
   disagreements, baseline outliers on the drifted scenes).
10. Write `docs/experiments/perception_performance/rembg_cage_high_<date>.md`: the decision rule
    and its outcome first, then the table, the run-1 comparison, per-scene and per-cage breakdowns,
    the sweep surface, timing, the clips.

## Open decisions

- Report per-cage (Cage-2, 5, 6, 1) as well as per-scene? Cage identity is the unit DeepLab's data
  plan reasons in, and it costs nothing, so the plan assumes yes.
- The baseline's median uses 60 frames per scene. More is cheap on a fixed camera; 60 is enough
  for a robot to be absent from any given pixel in most samples, which is all the median needs.
- Whether to keep the flip-book title cards per scene at 22 scenes (about 90 s of cards per clip at
  4 fps) or collapse to a one-line caption. The plan assumes cards, matching run 1.
