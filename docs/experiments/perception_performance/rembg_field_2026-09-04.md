# Does rembg find robots on the arena floor? No.

Status: **done** (2026-09-04). Plan: `background_subtraction_plan.md`.

`isnet-general-use` through `rembg`, run per frame on the 589 scored eval frames, finds 3% of
the robots at IoU 0.5 and 5% at IoU 0.3 in its best arm. It is not a robot detector at any
threshold in the sweep. What it returns instead is the arena floor: on 30% of scored frames more
than half the floor polygon reads as salient at alpha 128 or above, and the robots are small
bright specks on top of that response that only separate from it near the top of the alpha scale.
The frame-median background subtraction, even in the weakened form run here, finds 27% of robots
at IoU 0.5 and the trained YOLO finds 79%.

## What ran

Three arms, one `rembg` model, tuned on the first 10 frames of each recording (71 posed frames
held out) and scored on the remaining posed frames.

| arm | rembg input | field mask | applied |
|---|---|---|---|
| `rembg_warped` | 975x975 floor raster | floor square, intrinsic to the warp | before |
| `rembg_geom` | raw frame | nominal 8 ft square through the pose, inset 0.06 m | after |
| `rembg_deeplab` | raw frame, same matte | DeepLab floor, largest blob, convex hull | after |

Code: `training/model_eval/rembg_field.py` (shared pieces), `rembg_field_predict.py`
(`cache`, `tune`, `predict`), `rembg_field_render.py` (flip-books and contact sheets),
`rembg_field_stability.py` (contiguous SVO windows). Results and cached mattes are under
`training/data/eval_results/rembg/`; clips and sheets under `assets/2026-09-04_rembg/`.

Confirmed before running: rembg 2.0.83 resizes to 1024x1024 for `isnet-general-use`
(`DisSession.predict`), so the median 720p robot reaches the network at about 69 px, as the plan
estimated. The CUDA provider only loads if `torch` is imported first, because onnxruntime dlopens
`libcudnn.so.9` by soname and the venv has it only inside torch's bundled wheels; the code does
that and reports the active providers.

### Frame accounting

688 validated frames, 660 with a usable pose. 71 of the posed frames (not 80: nine of the first ten frames
across three recordings have no pose) are the tuning set. 589 frames are scored: 513 at
1280x720 and 76 at 1920x1080, 1,564 GT boxes. `rembg_deeplab` also ran on the 19 unposed
non-tuning frames (608 total) as the footnote the plan asked for.

`score.py` reads `validation_state.json` and passes exactly these frames through a scoring root
of symlinks with a filtered state file (`scoring_root/`), so no arm is charged for a frame it
did not run on. Every projection in the field check contact sheet (`field_check.jpg`, 16 frames
across all 8 recordings) sits on the cage walls; no frame was dropped for a bad pose.

### Two departures from the plan

**The baseline is weaker than planned.** The eight source MCAPs are not on this machine or on
pathfinder, so `background_subtraction_predict.py` gained `--background-from-gt-frames`: the floor
median is built from the sub dataset's own ~100 posed frames instead of 160 frames spread over
the recording. Seven of the eight medians are clean empty floors (`bgsub_backgrounds/`). The
MassD recording is not: the robot barely moves in that fight and is baked into the median as
three coloured blobs, and that recording scores F1 0.06 against 0.21-0.37 for the others. So
`bgsub_gtframes` is a floor on what the tuned method does, not the method, and every "worse"
verdict below is against that floor.

**The threshold grid was extended.** The plan swept alpha 64 to 208. All three arms picked 208,
the top of the range, so the grid was extended to 240. The post-hoc arms peak at 208 and fall
off at 224; `rembg_warped` moved to 224. The optimum sitting that high is itself the result: the
network paints the floor at moderate alpha and only the robots reach the top of the scale.

## Result

Agnostic level, `--conf 0.0` for the saliency and subtraction candidates because their operating
point is set by the tuned threshold and a confidence cut would silently drop detections whose
mean alpha is under 0.5. YOLO is scored in its own run at its normal `--conf 0.5` and is a
reference row, not a bootstrap comparison.

| candidate | IoU | precision | recall | F1 | vs `bgsub_gtframes` recall | F1 |
|---|---|---|---|---|---|---|
| `bgsub_gtframes` | 0.3 | 0.211 | 0.416 | 0.280 | baseline | |
| `rembg_deeplab` | 0.3 | 0.200 | 0.054 | 0.085 | -0.361 [-0.387, -0.338] worse | worse |
| `rembg_geom` | 0.3 | 0.182 | 0.053 | 0.082 | -0.363 [-0.387, -0.339] worse | worse |
| `rembg_warped` | 0.3 | 0.098 | 0.049 | 0.066 | -0.366 [-0.391, -0.342] worse | worse |
| `yolo2class` | 0.3 | 0.853 | 0.832 | 0.842 | reference | |
| `bgsub_gtframes` | 0.5 | 0.138 | 0.272 | 0.183 | baseline | |
| `rembg_deeplab` | 0.5 | 0.122 | 0.033 | 0.052 | -0.238 [-0.261, -0.216] worse | worse |
| `rembg_geom` | 0.5 | 0.110 | 0.032 | 0.050 | -0.240 [-0.263, -0.217] worse | worse |
| `rembg_warped` | 0.5 | 0.054 | 0.027 | 0.036 | -0.245 [-0.267, -0.222] worse | worse |
| `yolo2class` | 0.5 | 0.815 | 0.795 | 0.805 | reference | |

Paired bootstrap, 1000 resamples, 95% CI on the delta. Precision deltas for the two post-hoc arms
are not significant at either IoU (they sit at 0.12-0.20, the same band as the baseline);
recall and F1 are worse everywhere with intervals nowhere near zero. mAP@.5 is 0.008 for
`rembg_deeplab` against 0.042 for the baseline and 0.761 for YOLO.

The IoU 0.3 to 0.5 gap separates detection from localization. For `bgsub_gtframes` recall
falls from 0.42 to 0.27, so a third of what it finds is loosely boxed. For the rembg arms recall
is 0.05 at 0.3 and 0.03 at 0.5: the loss is mostly detection, and loosening the match does not
recover it. `rembg_warped` loses proportionally the most between the two (0.049 to 0.027),
which is the parallax cost, but it starts from so little that it hardly matters.

### The two post-hoc arms are the same arm

`rembg_geom` and `rembg_deeplab` share one matte, chose identical parameters (threshold 208,
`min_area` 1600, overlap 0.9), and differ by 2 true positives over 589 frames. At the chosen
threshold the matte has 574 components above `min_area` across the scored set; the geometric
polygon keeps 455 and the DeepLab hull keeps 426. The extra components the polygon lets through
are wall, rail and floor-edge clutter (`sheet_mask_disagreements.jpg`: every "kept by geom only"
tile is on the near rail or the wall base, where the nominal square runs below the visible
floor). The hull is the slightly better mask and it needs no pose. Neither mask changes the
answer, because the components they are filtering are mostly not robots to begin with.

The overlap fraction is nearly inert. Across 0.1 to 0.9, F1 on the tuning set moves from 0.074
to 0.076. The components that survive are either wholly inside the field or wholly outside it;
there are few straddlers for the fraction to adjudicate.

### By resolution

| candidate | IoU | 720p (513 frames, 1,341 GT) P / R / F1 | 1080p (76 frames, 223 GT) P / R / F1 |
|---|---|---|---|
| `bgsub_gtframes` | 0.5 | 0.147 / 0.264 / 0.188 | 0.107 / 0.318 / 0.160 |
| `rembg_deeplab` | 0.5 | 0.089 / 0.019 / 0.031 | 0.185 / 0.121 / 0.146 |
| `rembg_geom` | 0.5 | 0.074 / 0.017 / 0.028 | 0.185 / 0.121 / 0.146 |
| `rembg_warped` | 0.5 | 0.047 / 0.023 / 0.031 | 0.085 / 0.049 / 0.062 |

The plan's warning that the pooled number is the 720p number holds, and the 1080p recording is
where rembg does least badly: recall 0.12 at IoU 0.5 against 0.02 on 720p. Robots reach the
network at 77 px instead of 69 px there, but that is a small difference and the bigger one is
the view. The 1080p recording (`main_2026-05-01_17-42-20`) is shot high over the wall with a
clean floor and no crowd in frame, which is the frame rembg handles best (see the first row of
`sheet_raw_mattes.jpg`). One recording cannot separate resolution from viewpoint, so read this as
"the 1080p recording", not "1080p". Full table in `by_resolution.csv`; per-recording in
`by_recording_iou0.5.csv`.

## Why it fails

`sheet_raw_mattes.jpg` shows the raw matte for three frames of every recording. There are three
modes and all three are visible on that sheet:

1. **The floor is the salient object.** Roughly a third of frames come back with the whole floor
   polygon at alpha 128 or above. The robots are inside that blob, brighter than the floor
   around them, and thresholding at 208 recovers a few of them as specks. The rest merge into
   the floor component, which then fails `min_area` from above or the overlap test from below.
2. **The crowd is the salient object.** When people stand at the cage, rembg segments them
   cleanly and confidently and returns almost nothing on the floor (rows 2, 5 and 6 of the
   sheet). This is the competitive-saliency failure the plan described. The field mask removes
   the people, but the robots were never in the matte.
3. **Nothing is salient.** Frames with a dim, wide view return a near-black matte with faint
   floor texture. These contribute the median case: the median frame has 3% of its floor
   polygon at alpha 128 or above and 53% of frames have under 5%.

`sheet_missed_robots.jpg` puts the matte crop beside each missed robot. Most misses are mode 1
or 3: the robot is visible as a smudge at alpha 100-180, below the 208 the sweep chose, and
lowering the threshold lets the floor in first. That is why the F1 surface (`sweep_surface.png`)
is so flat below 128 and only lifts above 176: below that the floor component swallows
everything.

`rembg_warped` is worse for the reason the plan predicted plus one it did not. The predicted
one: the raster is a bright square on black and rembg returns the square (`flipbook_rembg_warped.mp4`,
top right panel, nearly every frame). The unpredicted one is the size of the parallax smear.
Camera height over the scored set has median 0.63 m (range 0.41 to 0.87 m), and the median GT
box sits 1.6 m from the nadir (p90 2.4 m). A point 0.1 m up lands 0.30 m past its footprint in
the raster, 123 raster px at the median and 191 at p90; at 0.2 m it is 301 px median, 477 px
p90. A 0.25 m robot becomes a 0.5 to 1 m wedge, longer than it is wide by several times, so
even the components that do isolate a robot box it at IoU well under 0.5
(`sheet_parallax_smear.jpg`). The plan expected the wedge to help detection; it does not,
because the whole floor is already salient and the wedge is just more of it.

## Temporal stability

`rembg_field_stability.py` decodes a 10 s window from a source SVO at 30 fps and runs the shared
raw pass and the `rembg_deeplab` arm on every frame. Only that arm can run here: the other two
need a pose per frame and poses exist only for the exported eval frames. Both windows are 720p,
because the three SVOs on this machine are all 720p; no 1080p window was available.

| window | frames | det/frame (mean, max) | frames where the count changed | detections carried from the previous frame |
|---|---|---|---|---|
| `2026-05-02T11-45-08` at 475 s | 299 | 0.76, 6 | 30% | 43% |
| `2026-05-02T15-35-04` at 116 s | 299 | 0.75, 4 | 21% | 72% |

"Carried" means a detection with IoU at least 0.5 against some detection in the previous frame.
In the first window fewer than half of the detections existed one frame earlier; the count
changes between one frame in three and the next. The clips (`stability_*.mp4`, three panels:
frame with detections, raw matte, matte after the hull) show the matte itself breathing: the
floor response fades in and out over a few frames as the camera pans, and detections appear and
vanish with it. Nothing here is stable enough to feed the filter without heavy temporal
smoothing, and there is not enough signal to smooth.

## Timing

RTX A6000, CUDA execution provider, per `remove(..., only_mask=True)` call including the PIL
round trip and the LANCZOS resize back to frame size:

| input | median | p90 | n |
|---|---|---|---|
| 1280x720 raw frame | 88 ms | 92 ms | 588 |
| 1920x1080 raw frame | 105 ms | 112 ms | 100 |
| 975x975 floor raster | 90 ms | 95 ms | 660 |

The network sees 1024x1024 in every case, so the spread is resize cost. The two post-hoc arms
share the raw pass; their marginal cost is the mask and the intersection at 16 ms per frame
(DeepLab at 13 ms of that, geometric warp and hull the rest) plus 5 ms of thresholding and
connected components. On the Jetson the raw pass alone would take several times the whole 60 ms
budget, which would have mattered if the result had been positive.

## Tuning

Selection by F1 at IoU 0.5 on the 71 tuning frames; the surface is in `sweep_surface.csv` and
plotted in `sweep_surface.png`.

| arm | threshold | `min_area` | overlap | tuning F1 | scored F1 (IoU 0.5) |
|---|---|---|---|---|---|
| `rembg_warped` | 224 | 1600 raster px | | 0.032 | 0.036 |
| `rembg_geom` | 208 | 1600 px | 0.9 | 0.075 | 0.050 |
| `rembg_deeplab` | 208 | 1600 px | 0.9 | 0.076 | 0.052 |

The surface is flat and low. F1 sits at 0.03 for every threshold from 64 to 112, steps to 0.06
at 128, dips at 160, and peaks at 0.076 at 208 before dropping to 0.03 at 240. `min_area` is
flat from 100 to 1600 and collapses above 3200, where the few robot-sized components go with
the clutter. There is no sharp optimum to overfit; the tuning frames and the scored frames
agree to within 0.03 F1, and that 0.03 is in the direction of the scored set being harder.

## What this says

- A general saliency network with no arena training does not find robots on the NHRL floor.
  The floor, the crowd and the cage all out-compete a 70 px robot for saliency, and the result
  holds across every threshold, both field masks, and both the raw frame and the top-down warp.
- Masking after inference did what it was meant to do, which is remove the fill-colour
  variable: `rembg_warped`, the one arm that pre-masks, is the worst of the three. But the
  post-hoc arms still fail, so the pre-masking concern was real and not the main problem.
- The DeepLab hull is a good field mask. It matched the geometric polygon on every robot
  detection and passed 29 fewer components, all wall and rail clutter, with no pose. That part is reusable
  independently of rembg.
- Nothing from this experiment should go toward the Jetson stack. Frame-median subtraction,
  which needs a pose and a background, beats it by 8x on recall from a contaminated baseline.

## Open decisions, resolved

- Tuning frames held out (71 frames, 11% of the posed set). The tuned and scored F1 agree to
  0.03, so the holdout cost nothing that matters.
- Clips are in `assets/2026-09-04_rembg/`: three flip-books at 4.3 to 5.5 MB (960x540, CRF 33,
  all 8 recordings with title cards) and two stability windows at about 1 MB each. Whether to
  commit them is still a call to make at commit time; they are sized to be committable.
