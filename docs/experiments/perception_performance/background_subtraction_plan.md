# Does rembg find robots on the arena floor?

Status: **done** (2026-09-04). Results in `rembg_field_2026-09-04.md`.

Scope pivoted twice. The first draft tested four subtraction methods on a new stationary-camera
dataset built from `nhrl_robots_bbox_2class`. The second tested `rembg` on the moving-camera eval
set with the field masked into the image before inference. This version keeps the dataset and drops
the pre-masking: **the field mask is applied after rembg runs, not before.** Neither earlier draft
was committed, so neither is recoverable from git. No dataset was built and nothing was run.

## Question

Every background subtraction method in the repo needs the camera to hold still, or needs a pose to
warp away the motion. `rembg` needs neither. It is a salient-object matting network: one image in,
one foreground matte out, no temporal state and no camera model.

**So: how much of a robot detector do you get for free from a saliency network that has never seen a
combat robot, and does giving it a bird's-eye view of the floor help or hurt?**

A positive result transfers to the live Jetson stack unchanged. Nothing else tested in this repo
has that property.

## The three arms

One rembg model, `isnet-general-use`, across three input and masking treatments.

| arm | rembg input | field mask | mask source | needs pose |
|---|---|---|---|---|
| `rembg_warped` | 975x975 floor raster | before, intrinsic to the warp | geometric | yes |
| `rembg_geom` | raw frame | **after** inference | geometric polygon | yes |
| `rembg_deeplab` | raw frame, same matte as above | **after** inference | DeepLab, convex-hulled | no |

**Masking after inference is the design decision that shapes the rest.** Filling outside the field
with black would hand the network a bright trapezoid on a dark ground, which is a closed,
high-contrast, centrally-placed region and exactly what a saliency net is built to return. Running
on the untouched frame keeps the input in the network's training distribution and discards
out-of-field responses afterward, which removes the fill colour as a variable entirely.

Two consequences worth stating up front, because they are free wins that fall out of that choice:

- **`rembg_geom` and `rembg_deeplab` share one rembg pass per frame.** The matte is computed once
  and cached; the two arms differ only in which mask intersects it. So the comparison isolates the
  mask exactly, with identical mattes, and the second arm costs almost nothing to add.
- **`rembg_deeplab` needs no camera pose at all.** It is the only arm with no geometric dependency,
  which makes it the one that reflects what the Jetson would actually run.

## Dataset: `nhrl_keypoints_eval_test`, unchanged

No new dataset. The original ask named `background_subtraction_eval_test` sampled from
`nhrl_robots_bbox_2class`; the pivot to the moving-camera set makes that unnecessary, because this
set already carries hand-corrected ground truth **and** the exported camera geometry the warp needs.

| | |
|---|---|
| recordings | 8 |
| labeled frames | 688 (`validation_state.json`: 688 pass, 68 fail) |
| resolution | **mixed**: 588 frames 1280x720, 100 frames 1920x1080 |
| GT boxes | 1,843 |
| by class | `opponent` 763, `mrs_buff_mk3` 670, `house_bot` 402, `object` 8 |
| camera poses | 541 `exact`, 147 `interpolated`, 18 `unavailable`, 14 `ambiguous` |

The 32 frames with `unavailable` or `ambiguous` poses cannot be warped, so all three arms score on
the same ~656 frames for comparability. `rembg_deeplab` could run on all 688, and that extension is
reported as a footnote rather than mixed into the main table.

**The first 10 frames of each recording are the tuning set and are held out of scoring** (see
Tuning). That is up to 80 frames, leaving roughly 576 scored. Holding them out is a judgement call
rather than something asked for, and it costs 12% of the set, but every threshold in this experiment
is picked on those frames and scoring them again would report the fit rather than the method.

`mr_stabs_mk2` has zero boxes in this set, and `object` has 8. Neither supports a per-class claim.

**Frame size is not uniform.** Resolution is constant within a recording and varies between them:
`main_2026-05-01_17-42-20` is 1920x1080 (100 frames) and the other seven recordings are all
1280x720 (588 frames). Nothing may hardcode a frame size or cache one set of intrinsics across the
set. `camera_info.json` already lives per recording, which is the right granularity, and `score.py`
reads each image's size per frame, so scoring is unaffected. The code written here has to match that
discipline.

## Why `isnet-general-use`, and only it

`u2net` is dropped. Two reasons, and the first is decisive.

**Input resolution.** rembg resizes internally before inference: `u2net` to 320x320,
`isnet-general-use` to 1024x1024 (confirm against the installed package before running). Measured
over the 1,843 GT boxes, the median box's longer side is 86 px at 720p and 144 px at 1080p. After
the network's internal resize:

| arm / model | median robot at inference | 10th percentile |
|---|---|---|
| raw frame, 720p, `u2net` @320 | ~21 px | ~14 px |
| raw frame, 720p, `isnet` @1024 | 69 px | 43 px |
| raw frame, 1080p, `isnet` @1024 | 77 px | 54 px |
| 975 raster, `isnet` @1024 | ~105 px | n/a |

At 21 px a robot is close to undetectable and a null result would say nothing about saliency. At
~70 px the test is fair. This also removes the argument for cropping the frame before inference,
which an earlier draft floated to buy back resolution.

**Multi-object handling.** U2Net is built around a *single* salient object. An NHRL frame holds two
robots and a house bot, so U2Net would tend to merge them or pick one, and merged blobs cost recall
through IoU failure even when the pixels are right. ISNet handles multiple distinct objects better.

Run per frame with no video context, as asked: one `remove(..., only_mask=True)` call per image,
returning the matte directly instead of an RGBA cutout.

## Field masks

**Geometric polygon** (`rembg_warped`, `rembg_geom`): project the nominal 8 ft square
(`NOMINAL_FIELD_SIZE_M = 2.4384`) through each frame's `tf_field_from_camera`, using
`camera_geometry.load_frame_geometry` and `FloorRaster.image_from_raster`.
`make_field_projection_check.py` already draws exactly this and is the tool for eyeballing it.
`FLOOR_MARGIN_M = 0.06` trims it inward, because the arena wall is not on the floor plane and sits
within a few pixels of the floor edge.

**DeepLab mask** (`rembg_deeplab`): `data/models/field_deeplabv3p_r50_2026-07-29.pth` at 256x256,
upscaled to frame size, largest connected component, **then the convex hull of that contour**. This
model was trained on ZED footage, which is what this dataset is, so it is on home ground here in a
way it would not be on broadcast frames.

The hull is the fix for the arm's central problem, and it is not a cosmetic one. A robot standing on
the floor *occludes* the floor, so DeepLab returns a hole exactly where the robot is. Masking a
detection against the raw mask would therefore drop the very detections the experiment is trying to
count, because each robot sits in its own hole. The hull fills those holes. It also repairs the case
`convex_hull_masks.py` documents from the field corpus, where a robot lying across the frame splits
the floor into uneven blobs and the smaller genuine half looks like a speck next to the larger.

A convex hull is the right shape here rather than a convenient one: the arena floor is a square, and
a square seen in perspective is a convex quadrilateral. The true field region is convex, so hulling
can only recover floor that the mask lost. It cannot invent a concavity that should have been there.

`training/deeplab/convex_hull_masks.py` is prior art for the per-blob version. This use is simpler,
one hull over the largest contour, so it wants a small inline function rather than that script.

A detection survives masking when the overlap between its matte component and the field mask exceeds
a fixed fraction of the component's area. A blob half on the floor and half up the wall should not
count as a floor detection, and a hard "centroid inside" test would let it. The fraction is one
number, set once, reported in the write-up.

## Pipeline

Shared, per frame:

1. Load frame and `FrameGeometry`. Skip `unavailable` and `ambiguous`.
2. Build the floor homography from intrinsics and pose via `FloorRaster.image_from_raster`.
3. Run rembg once on the raw frame. Cache the matte for both post-hoc arms.

`rembg_warped`:

4. `warp_inverse(frame, homography, raster.size)` into a 975 x 975 raster (`RASTER_PX_PER_M = 400`
   over 2.4384 m). Zero everything outside the floor square and the in-front mask
   (`FloorRaster.in_front_mask` removes the region behind the camera that `warpPerspective`
   otherwise paints into the sky). The raster is the same size for every frame regardless of source
   resolution, which normalizes the input on a mixed-resolution set.
5. rembg on the raster. This is a second inference pass, unavoidable since the input differs.
6. Threshold the matte, connected components, box per component above `min_area`.
7. **Project each raster box back to image pixels** through the homography and take the bounding box
   of the resulting quadrilateral. A raster-axis-aligned box is not axis-aligned in the image, so
   this inflates boxes slightly and that cost belongs to this arm.

`rembg_geom` and `rembg_deeplab`:

4. Threshold the cached matte, connected components.
5. Intersect each component with the arm's field mask; drop components below the overlap fraction.
6. Box per surviving component above `min_area`. No back-projection, no warp.

All three write the predictions JSON `score.py` already consumes (`engine_path.suffix == ".json"`
selects `PrecomputedDetector`):

```json
{"labels": ["opponent"],
 "frames": {"<stamp_ns>": [{"xyxy": [x1,y1,x2,y2], "score": 0.0, "class_id": 0}]}}
```

`opponent` rather than an invented `robot`, matching `background_subtraction_predict.py`'s
`--label` default, so `score.py`'s unknown-label check stays quiet.

Confidence is the mean alpha inside the component. That is a real confidence, unlike the difference
intensity `background_subtraction_predict.py` has to invent, so the mAP column means more here than
it does there.

Timing gets reported per input size the network actually sees: 1280x720 and 1920x1080 for the raw
pass, 975x975 for the raster pass. The two post-hoc arms share the raw pass, so their marginal cost
is the mask plus the intersection, reported separately from the inference.

## Baselines

Both already exist, so the table costs one extra run each:

- **Warped median subtraction**, `background_subtraction_predict.py` on the same frames. The tuned
  geometric method, and the honest thing to beat.
- **`yolo26n_nhrl_robots_bbox_2class`**, `data/eval_models/..._eplast_x86_64_sm86.engine`. The
  trained detector, as the ceiling.

## Tuning: sweep on the first 10 frames of each recording

Three numbers need values and none has an obvious default: the matte threshold (alpha cutoff),
`min_area`, and the overlap fraction for post-hoc masking. All three are chosen by sweep on the
**first 10 frames of each of the 8 recordings**, up to 80 frames, which are then held out of
scoring.

Ten frames per recording rather than 80 from one: the recordings differ in resolution, lighting and
arena, so a tuning set drawn from a single fight would fit that fight. Taking the head of each
recording covers all 8 at even weight.

The sweep is cheap because **the mattes for the tuning frames are computed once and cached**, and
only post-processing varies across the grid. One rembg pass over 80 frames, then the grid runs on
cached arrays.

| parameter | range | applies to |
|---|---|---|
| matte threshold | 64 to 208, step 16 (10 values) | all three arms |
| `min_area` | 8 values, log-spaced | all three arms, swept per arm |
| overlap fraction | 0.1 to 0.9, step 0.1 (9 values) | `rembg_geom`, `rembg_deeplab` only |

`min_area` is swept separately per arm because the units are not comparable. At `RASTER_PX_PER_M =
400` a 0.25 m robot covers roughly 10,000 px² in the raster; the median 720p GT box is about
7,400 px². Close enough to look interchangeable, far enough apart to matter at the tail.

Selection criterion is **F1 at IoU 0.5**, one value frozen per arm before any scoring run. The full
sweep surface goes in the write-up, not just the argmax: a flat optimum and a sharp one mean
different things about how much the result depends on the tuning, and a sharp one is itself a
finding.

## Scoring: IoU 0.3 and 0.5

`score.py` takes a single `--iou`, so this is two runs into separate output dirs:

```bash
for iou in 0.3 0.5; do
  venv/bin/python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
    --candidate bgsub_median=.../bgsub.json \
    --candidate rembg_warped=.../rembg_warped.json \
    --candidate rembg_geom=.../rembg_geom.json \
    --candidate rembg_deeplab=.../rembg_deeplab.json \
    --candidate yolo2class=data/eval_models/yolo26n_nhrl_robots_bbox_2class_2026-07-29_eplast_x86_64_sm86.engine \
    --labels opponent,house_bot --iou "$iou" --baseline bgsub_median \
    --output training/data/eval_results/rembg_iou${iou}
done
```

Both thresholds get reported side by side for every arm, because **the gap between them is itself a
measurement**. A method that finds robots but boxes them loosely scores well at 0.3 and badly at
0.5; a method that misses them scores badly at both. That separates detection from localization,
which matters most for `rembg_warped`, where height parallax is expected to inflate boxes (see
Risks). Reporting only 0.5 would fold the two failures into one number.

**Only the `agnostic` level says anything about rembg.** It has no class head; every detection is
emitted as one class. `archetype` and `instance` are reported for the YOLO reference only. Recall,
precision and F1 carry the result.

## Video clips

Every arm gets rendered as video, because a table of recall and precision does not show *how* a
method fails, and the failure modes expected here (merged blobs, whole-floor responses, the parallax
smear) are obvious on sight and invisible in a number.

**Eval-frame clips, one per arm.** Each clip runs through all 8 recordings in stamp order with a
title card between them, so one file per arm rather than 24. The eval frames are sampled seconds
apart, so this is a flip-book, not motion: 4 fps, with a caption per frame carrying the recording,
stamp, detection count, GT count, and the TP/FP/FN split. Same 2x2 panel for every arm:

| | |
|---|---|
| frame, GT boxes and predicted boxes | raw rembg matte |
| matte after the field mask, mask outline drawn | arm-specific: the 975 raster for `rembg_warped`, the field mask overlay for the post-hoc arms |

The raw-matte panel is the one that earns the format. It shows what rembg saw *before* any masking,
which is the only way to tell "the mask deleted it" from "rembg never found it", and those two are
indistinguishable in the score.

**Contiguous clips, for temporal stability.** The eval frames cannot show flicker, and flicker
matters: rembg runs per frame with no temporal state, and anything feeding the filter needs to be
stable frame to frame. So one 10 s window per arm from the source MCAP at native frame rate, running
the arm on every frame, no GT overlay. Two recordings only, one 720p and one 1080p, to bound the
cost. If detections strobe on and off between adjacent frames, that shows up here and nowhere else
in this experiment.

Output to `docs/experiments/perception_performance/assets/<date>_rembg/`, H.264, capped at 1280x720
and a few MB each. `docs/media/` already carries committed demo GIFs, so video in the docs tree has
precedent, but these should stay small enough not to bloat the repo.

## Risks

**Height parallax will smear robots in `rembg_warped`.** The floor homography is exact only for
points on the floor plane. A point at height `h`, seen from a camera at height `H`, lands in the
raster displaced radially outward from the nadir by `d * h / (H - h)`, where `d` is its ground
distance from the nadir. A robot is 0.1 to 0.2 m tall and the fight happens metres from the nadir,
so each robot warps into a wedge pointing away from the camera. This helps detection (the wedge is
high contrast against bare floor) and hurts localization (the box is too long). The IoU 0.3 and 0.5
pair is what separates the two. The actual displacement gets computed per frame from
`geometry.camera_height_m` and reported, so the confound is quantified rather than guessed at.

**Saliency is competitive, so post-hoc masking has its own failure mode.** Running on the untouched
frame means a person leaning on the cage, a bright light, or the arena structure can win the
saliency contest and suppress the robots. Those responses get discarded by the mask, but the
suppression happened before the mask ever applied, so the robots may simply be absent from the
matte. This is the cost of not pre-masking and it trades one failure for another rather than
removing failure. The diagnostic is a contact sheet of the raw mattes before masking: if the network
is confidently segmenting the crowd, that is visible immediately.

**`rembg_deeplab` inherits DeepLab's errors, and the hull only fixes the inward ones.** Hulling
recovers floor the mask lost to occlusion and splitting. It does nothing about a mask that bleeds
*outward* past the wall, and it actively makes that worse, since the hull of an over-large mask is
at least as large. So the expected residual failure flips direction: not deleted detections, but
wall and floor-adjacent clutter surviving the mask and costing precision. The two post-hoc arms
share a matte, so any gap between them is attributable to the mask alone, which is the cleanest
possible read on it. The per-arm difference is the measurement, not a nuisance.

**rembg has never seen a combat robot, an arena floor, or an overhead view.** A null result is a
real and likely outcome. The write-up reports it plainly rather than tuning thresholds until the
table looks better.

## Steps

1. `pip install rembg` into `venv/`, pin it in `pyproject.toml`, fetch the `isnet-general-use`
   weights. Confirm the model's internal input size.
2. Verify the geometric polygon on a contact sheet with `make_field_projection_check.py`. Drop any
   frame whose projection is visibly wrong and record the count.
3. Build the shared frame loader: geometry, homography, polygon, pose filtering, matte cache. Reuse
   `FloorRaster`, `warp_inverse` and `camera_geometry` rather than reimplementing them.
4. Run DeepLab over the frame set, take the largest contour's convex hull, cache the masks. Eyeball a
   contact sheet of raw mask against hull against geometric polygon. The hull's effect on robot-sized
   holes should be visible before anything depends on it.
5. Implement the raw-frame pass and both post-hoc arms off the cached matte.
6. Implement `rembg_warped`, including the raster-box back-projection.
7. **Tune.** Cache mattes for the 80 tuning frames, sweep threshold, `min_area` and overlap fraction,
   pick each arm's values by F1 at IoU 0.5, freeze them, and keep the full sweep surface for the
   write-up.
8. Write three predictions JSONs over the ~576 scored frames. Generate the two baselines on the same
   frames.
9. Score at IoU 0.3 and 0.5, agnostic level, paired bootstrap, into separate output dirs.
10. Render the video clips: one eval-frame flip-book per arm, plus the two contiguous stability
    windows.
11. Build failure contact sheets: raw mattes before masking, merged blobs, missed robots, the
    parallax smear, and every detection the two masks disagree about.
12. Write `docs/experiments/perception_performance/rembg_field_<date>.md` with the IoU 0.3 / 0.5
    table, the sweep surfaces, the parallax measurement, timing per input size, the contact sheets
    and links to the clips. Break recall and precision out by source resolution as well as pooled:
    720p is 85% of the set, so a pooled number is essentially the 720p number and any 1080p effect
    would be invisible in it.

## Open decisions

- Holding the 80 tuning frames out of scoring is my call, not something asked for. It costs 12% of
  the set. The alternative is scoring all ~656 and reporting the tuning frames separately, which
  keeps more data at the price of a number that partly reports its own fit.
- Committing the clips to the repo, or keeping them local and linking from the write-up? `docs/media/`
  has committed GIFs already, so either is consistent with what is there.
