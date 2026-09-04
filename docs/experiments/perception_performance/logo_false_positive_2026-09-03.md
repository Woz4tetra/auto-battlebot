# Killing the floor-logo false positive with background subtraction and depth

Analysis date: 2026-09-03, revised 2026-09-04. Question: the opponent detector puts confident
boxes on the MassD arena's floor banner. Can background subtraction or depth veto them?

Both can. Neither survives as an unconditional gate, because the recall cost on the arena the
detector already handles is far too high at any threshold that helps the arena it does not.

Plan: `logo_false_positive_plan.md`.

## What changed in the revision

The first pass got three things wrong. They are corrected throughout and called out here because
two of them changed a conclusion.

1. **The eval set was 429 frames; it should have been 688.** `validation_state.json` marks all 688
   frames `pass`, MassD included. `score.py` was reading the stale `.edit_state.json`, which lists
   429 and names no MassD frame at all. Fixed: `score.py` now prefers `validation_state.json` and
   falls back to `.edit_state.json` only when it is absent.
2. **The engine under test was the wrong one.** Now the deployed
   `yolo26n_nhrl_robots_bbox_2class_mixed_2026-07-31`.
3. **"Depth cannot work" was wrong.** The first pass tested only occlusion prominence, found
   AUC 0.430, and generalized that to depth. Height above the fitted field plane, the quantity
   `render_depth_birdseye.py` paints, scores AUC 0.715 to 0.790 on the same boxes and lifts MassD
   precision by 0.248 with a significant CI. The retraction is in "Why prominence failed and
   height did not" below.

## Summary

- **The false positive is the banner, and it is nearly all of the problem.** 114 of 210 detections
  on MassD are false. 104 of those 114 (91%) fall into six static clusters on the arena floor. The
  largest fires in 40 frames at one fixed field position, at confidences up to 0.84, higher than
  many true detections.
- **Background subtraction separates best**, AUC 0.824. On MassD it lifts precision 0.457 ->
  **0.743** (+0.285, CI 0.205..0.367) for 0.114 recall.
- **Height above the field plane also separates**, AUC 0.715 at the renderer's band and 0.790 at a
  wider one. On MassD precision 0.457 -> **0.705** (+0.248, CI 0.174..0.337) for 0.222 recall.
  Median height inside a box is **0.030 m for a real robot and -0.002 m for a false positive**,
  which is exactly the flat-graphic-versus-short-robot distinction.
- **Occlusion prominence is backwards**, AUC 0.437, precision +0.021 (ns). It is the one depth
  formulation that does not work, and the reason is now understood.
- **Together they are the strongest precision gate and the worst recall cost**: 0.800 precision
  (+0.343) at 0.238 recall.
- **No gate survives the pre-registered recall rule.** On the full 688 frames every gate costs far
  more than the 0.05 recall budget: `yolo_and_rgb` takes recall 0.720 -> 0.102, `yolo_and_height`
  0.720 -> 0.318. The May arena has almost nothing to win (baseline precision 0.896) and everything
  to lose.

## Reproduction

Deployed engine, conf 0.6, `--labels opponent,house_bot`, `taxonomy.yaml`, agnostic level:

| eval set | frames | precision | recall | f1 | mAP50 |
|---|---|---|---|---|---|
| all recordings | 688 | 0.896 | 0.720 | 0.798 | 0.700 |
| MassD 2026-08-29 only | 98 | **0.457** | 0.519 | 0.486 | 0.461 |

The older `2026-07-29` engine scores MassD precision 0.452. Both fail identically, so this is a
training-coverage gap, not a bad checkpoint.

`false_positive_crops.jpg` shows the top false positives by confidence: overwhelmingly the
"MASSACHUSETTS RESURGENCE" banner lying flat on the plywood floor, plus the red corner mat and the
dark mat at the far wall.

## Arms

One detector pass per frame. Every arm reads identical raw boxes and differs only in which
survive, so recall loss is the entire cost and precision gain the entire benefit.

| arm | gate |
|---|---|
| `yolo_only` | none, the model as it ships |
| `yolo_and_rgb` | >=45% of the box differs from the floor background |
| `yolo_and_depth` | >=15% of measured box pixels stand proud of their surroundings |
| `yolo_and_height` | >=30% of measured box pixels sit in the robot-height band above the plane |
| `yolo_and_rgb_and_height` | the two gates that carry signal, together |

## Result: MassD, the arena with the logo

98 frames, paired bootstrap 1000 resamples, baseline `yolo_only`:

| arm | precision | recall | f1 | mAP50 |
|---|---|---|---|---|
| `yolo_only` | 0.457 | 0.519 | 0.486 | 0.461 |
| `yolo_and_rgb` | **0.743** | 0.405 | 0.524 | 0.380 |
| `yolo_and_depth` | 0.479 | 0.362 | 0.412 | 0.330 |
| `yolo_and_height` | 0.705 | 0.297 | 0.418 | 0.283 |
| `yolo_and_rgb_and_height` | **0.800** | 0.238 | 0.367 | 0.228 |

| arm | metric | delta | CI | verdict |
|---|---|---|---|---|
| `yolo_and_rgb` | precision | +0.285 | 0.205..0.367 | **better** |
| `yolo_and_rgb` | recall | -0.114 | -0.160..-0.069 | worse |
| `yolo_and_rgb` | f1 | +0.038 | -0.016..0.093 | ns |
| `yolo_and_depth` | precision | +0.021 | -0.016..0.064 | ns |
| `yolo_and_depth` | f1 | -0.074 | -0.116..-0.037 | worse |
| `yolo_and_height` | precision | +0.248 | 0.174..0.337 | **better** |
| `yolo_and_height` | recall | -0.222 | -0.284..-0.168 | worse |
| `yolo_and_rgb_and_height` | precision | +0.343 | 0.246..0.445 | **better** |
| `yolo_and_rgb_and_height` | recall | -0.281 | -0.342..-0.220 | worse |

## Result: all 688 frames

| arm | precision | recall | f1 |
|---|---|---|---|
| `yolo_only` | 0.896 | 0.720 | 0.798 |
| `yolo_and_rgb` | 0.874 | **0.102** | 0.183 |
| `yolo_and_depth` | 0.911 | 0.595 | 0.720 |
| `yolo_and_height` | 0.936 | 0.318 | 0.475 |
| `yolo_and_rgb_and_height` | 0.904 | 0.056 | 0.106 |

The May recordings dominate this set (590 of 688 frames) and every gate is a disaster on them. The
RGB gate keeps a tenth of the true detections. This is the finding that blocks shipping any of it.

## Why prominence failed and height did not

Per-box separability on MassD, 210 boxes, 96 true and 114 false:

| signal | mean on true | mean on false | AUC |
|---|---|---|---|
| `fg_fraction` (background subtraction) | 0.577 | 0.311 | **0.824** |
| `height_fraction` (band above plane) | 0.348 | 0.153 | **0.715** |
| `prom_fraction` (occlusion prominence) | 0.512 | **0.611** | 0.437 |

Prominence is the local max-filter depth minus the pixel's own depth: one-sided, relative, and
plane-free. Its independence from the plane fit is what makes it fail. The banner sits near the
arena wall, so the max filter reaches over the wall to the room beyond and a flat painted graphic
reads as maximally proud of its surroundings. Banner crops carry `prom_fraction` of 1.00.
Prominence measures nearness to a depth edge, not height above the floor.

Height above the fitted plane is two-sided and absolute: the banner falls below the band and the
wall, glass and crowd fall above it. It costs a dependency on the plane fit and the pose chain,
which is precisely why the earlier version of `depth_gated_subtraction.py` dropped it, arguing that
3 to 5 cm robots sit below stereo noise at 2 to 3 m. That argument was wrong. Measured per box, the
median height inside a true box is 0.030 m and inside a false one -0.002 m.

Band width matters more than the renderer's default suggests, swept on MassD:

| band (m) | AUC | mean on true | mean on false |
|---|---|---|---|
| 0.01 - 0.05 (renderer default) | 0.712 | 0.402 | 0.215 |
| 0.005 - 0.08 | 0.722 | 0.625 | 0.305 |
| 0.01 - 0.12 | 0.761 | 0.636 | 0.265 |
| 0.02 - 0.10 | **0.790** | 0.520 | 0.169 |

A 4 cm window is right for a picture, where the eye forgives a robot rendered in patches, and too
tight for a per-box gate, where it is narrower than the plane fit's own error. The script default
is now 0.02 to 0.10.

## Why the video tools look better than these scores

`field_background_subtraction.py` and `render_depth_birdseye.py` both show the robots clearly. Both
scored below baseline here. That is not a contradiction, and only part of it is a real defect.

**1. The video and the gate ask different questions.** This is the biggest factor. A video answers
"is there visible evidence somewhere in this frame". The gate answers "is at least 45% of *this
particular box* foreground". Across the May recordings:

| threshold on `fg_fraction` | share of detector boxes passing (May) | (MassD) |
|---|---|---|
| > 0 (visible at all) | **0.94** | 1.00 |
| >= 0.10 | 0.66 | 0.91 |
| >= 0.25 | 0.39 | 0.71 |
| >= 0.45 (the gate) | **0.09** | 0.48 |

94% of May boxes have some foreground in them, which is what you are seeing in the video. 9% clear
the bar the gate set. The eye also integrates a partial, speckled blob into "that is a robot"; a
box-fraction test cannot.

**2. The video shows its best three blobs, the gate must judge every box.**
`field_background_subtraction.py` boxes `--max-blobs 3` per frame and overlays the difference at
`--overlay-alpha 0.9`, which amplifies faint signal for a viewer. Nothing in that pipeline has to
decide about a marginal box, and nothing counts a false positive against it.

**3. My compare mask is genuinely worse, and this is a real defect.**
`depth_gated_subtraction.py` subtracts only inside a nominal 2.4384 m floor square, inset 6 cm,
routed through a 400 px/m metric raster and back, then eroded 11 px.
`field_background_subtraction.py` uses the pipeline's own `/field_mask` in image space, eroded
12 px, with no raster round trip. Measured coverage of true detector boxes:

| recording | mean compare-mask coverage of true boxes |
|---|---|
| MassD | 0.892 |
| May 15-35-00 | 0.719 |
| May 17-26-12 | 0.679 |

Roughly 30% of every true box on May is outside the region my arm even looks at, because the real
fitted field is smaller and offset from the nominal square. Using the segmented field mask instead
would recover it.

**4. The subtraction parameters suppress small robots.** Both scripts share these defaults, so this
is not what separates them, but it does depress my numbers. On May, foreground fraction inside the
covered part of true boxes:

| illumination | edge tolerance | fg fraction on true boxes |
|---|---|---|
| local | 0.5 (defaults) | 0.273 |
| local | 0.0 | 0.357 |
| global | 0.5 | 0.362 |
| none | 0.5 | 0.365 |
| global | 0.0 | **0.445** |

`illumination=local` builds a gain field with a 40 px sigma, comparable to a robot's size in these
wide May frames, so a small robot is partly absorbed into its own illumination estimate.
`edge_tolerance=0.5` raises the threshold wherever the background has gradients, which is exactly
where a robot's edges are. Background sample count is not a factor (0.273 at 160 samples, 0.265 at
48, 0.235 at 16).

Re-running every arm with `--illumination global --edge-tolerance 0.0` confirms this is a
re-scaling rather than a fix: May recall improves (RGB arm 0.102 -> 0.231 on the full set) but
MassD precision falls at the same fixed threshold (0.743 -> 0.586), because raising every box's
foreground fraction makes a fixed 0.45 cut more permissive. The threshold, not the parameters, is
the thing that does not transfer.

## Result: the false positives are static, and that is still the useful finding

Projecting each false positive's box foot onto the arena floor through its own frame pose and
clustering at 0.20 m, on MassD with the deployed engine:

| cluster | field position (m) | false positives | frames |
|---|---|---|---|
| A | (-0.107, -0.376) | 40 | 40 |
| B | (-0.295, 0.416) | 25 | 25 |
| C | (-0.098, 0.449) | 15 | 15 |
| D | (-0.315, -0.367) | 14 | 14 |
| E | (-0.430, 0.373) | 10 | 10 |

One false positive per frame per cluster, which is what a fixed object looks like. Six clusters
clear the static threshold and together hold 104 of 114 false positives. A robot moves, so its
detections scatter; scenery does not.

This discriminator needs no stereo, no floor median and no photometric threshold, so it inherits
neither the arena-lighting dependence that broke the RGB gate nor the plane-fit dependence of the
height gate. By construction it cannot suppress a moving robot, which is the failure mode that
disqualified both.

### Reconciling this with the embedding probe's negative result

`opponent_embedding/embedding_prototype_probe_report.md` ran the same idea and found nothing: "no
proposal held the same 8 px-quantized box for 100+ consecutive frames in any recording. Logo false
positives at conf 0.05 flicker and jitter rather than persisting, so the 'static box = fixture'
heuristic finds nothing to pool at this threshold."

That is not a contradiction of the table above; the two measure different things. Three
differences, and each one matters for how a suppressor should be built:

- **Frame of reference.** That pass quantized boxes in *image space*. The camera moves, so a
  world-static object's image box moves with it and never holds a quantized cell. This experiment
  projects each box foot into the *field frame* through that frame's own pose, which removes
  camera motion before asking whether anything is static.
- **Confidence.** That pass ran conf 0.05 to feed an embedding gallery. This one runs the deployed
  0.6. The banner's box is confident (0.76 to 0.84) and stable; the cloud of 0.05 proposals around
  it is neither.
- **Persistence test.** That pass required 100+ *consecutive* frames. This one counts occurrences
  across ground-truth frames sampled roughly 2 s apart, so 40 hits is 40 separate visits to the
  same spot over the recording, not a 40-frame run.

So the suppressor to build is more specific than "detections that do not move": it must cluster in
the **field frame**, at **deployed confidence**, and accumulate evidence **across occurrences
rather than requiring consecutive frames**. An image-space, low-confidence, consecutive-run version
has already been measured and does not work.

## Implications

- **Do not ship any of these gates unconditionally.** Every one costs more than the recall budget
  on the arena the detector already handles.
- **Occlusion prominence is retired; height above the plane is not.** Use the field-frame height
  band for any future depth work, with a band around 0.02 to 0.10 m, and do not repeat the
  "robots are below stereo noise" reasoning. It was wrong.
- **Build the static-detection suppressor next.** 91% of MassD false positives are static in the
  field frame.
- **Fix the compare mask** in `depth_gated_subtraction.py` to use the pipeline's `/field_mask`
  rather than a nominal floor square. It is throwing away 30% of every true box on May.
- **Get MassD-arena frames into the training corpus.** Both engines fail identically.

## Caveats

- **Thresholds were swept on the test set.** The 0.45 foreground fraction and the 0.02 to 0.10 band
  were both chosen with MassD in view, so MassD's gains are optimistic bounds.
- **MassD stereo is the worst of the eight recordings**, 0.42 mean valid-pixel fraction against
  0.56 to 0.69 on the May recordings. The height gate is working with the least depth available.
- **Arms cover fewer frames than the eval set.** Frames need both a pose and cached depth. All
  arms share the frame set, so deltas hold, but absolute recall is depressed relative to running
  the engine directly.
- **Static-cluster positions drift between field re-inits**, since `camera_world` is redefined at
  each initialisation. Cluster counts survive that; the coordinates are not arena survey points.
- **No latency measurement.** Neither gate has been costed against the 60 ms budget.

## Depth cache integrity

`cache_gt_depth.py` reaches each frame with `set_svo_position`, and a smeared seek is silent.
`check_depth_cache_alignment.py` re-seeks each index, pulls the left image instead of depth, and
sweeps neighbouring indices, comparing inside the GT boxes where the only moving thing is. An
absolute difference cannot settle this (rectification differences put even a correct match in the
teens) but the shape of the curve can. All eight recordings give a clean V centred on the claimed
index, MassD sharpest at 11.46 gray levels deep. The ±1 minima on some May recordings are the known
half-frame offset between SVO and pipeline stamps.

## Clips

`clips_logo_fp/` holds one 300-frame clip per recording, four panels each: the frame, the
background-subtraction foreground, height above the plane, and prominence, with the same detector
boxes coloured green or red by each gate. `clips_logo_fp/README.md` explains how to read one and
carries the per-recording keep rates.

The keep-rate column is worth reading on its own. At one fixed RGB threshold of 0.45, the gate
keeps 70% of detector boxes on MassD and 0% to 19% on five of the six May recordings, and it also
splits *within* the NHRL cage: 43% on 14-12-25 against 0% on 15-35-00. Per-arena calibration would
not have rescued it; per-fight calibration would have been needed. Prominence, meanwhile, keeps
100% of boxes on five of seven recordings and never drops below 76%, which is what a gate that
does not fire looks like.

## Artifacts

- Plan: `docs/experiments/perception_performance/logo_false_positive_plan.md`
- Clips: `docs/experiments/perception_performance/clips_logo_fp/` (8 clips, gitignored as `*.mp4`;
  regenerate with `playground/make_gate_clips.py`)
- Arms, deployed engine, all recordings: `training/data/eval_results/logo_fp_mixed/`
  (`_tuned` suffix for the `global`/`0.0` subtraction parameters), plus `gates_massd.json` and
  `false_positive_crops.jpg`
- Arms, `2026-07-29` engine: `training/data/eval_results/logo_fp_massd/`, `logo_fp_all/`
- Scores: `scores_mixed_full/`, `scores_mixed_massd/`, `scores_tuned_full/`,
  `scores_tuned_massd/`, `scores_engine_cmp_massd/` under `training/data/eval_results/`
- Depth cache: `training/data/eval_results/depth_cache_neural_plus/` (NEURAL_PLUS, 8 recordings)
- Cache alignment: `training/data/eval_results/depth_cache_alignment.json`
- Code: `playground/depth_gated_subtraction.py` (RGB and height arms added, class ids preserved),
  `playground/false_positive_gates.py` (new), `playground/check_depth_cache_alignment.py` (new),
  `playground/make_gate_clips.py` (new), `training/model_eval/score.py`
  (`validation_state.json` precedence)
