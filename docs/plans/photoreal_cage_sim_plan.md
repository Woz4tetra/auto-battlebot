# Photoreal NHRL cage simulator plan

Goal: render frames that look enough like the NHRL cage, from a camera mounted
high and near the glass, that a detector trained or graded on them predicts how
the real detector will behave at that mount. Written for the camera move in
`docs/experiments/rgb_homography/rgb_camera_migration_2026-09-09.md`.

Status: options, not a decision. Nothing here is built.

## What I measured before writing this

I pulled the BrettZone API and the footage already on disk rather than assuming.

**Each fight is filmed by 11 to 13 cameras, not one.** For `nhrl_may26_3lb` W-52:
`Program-Feed`, `Cage-2-Overhead-High`, `Cage-2-Blue-High`, `Cage-2-NE-High`,
`Cage-2-Red-High`, `Cage-2-SW-High`, `MobileCam-1/3/4`, `PTZ-Bowl-1`, `SkyCam-B`.
Cage 1 adds `Cage-1-Ceiling-High` and `SkyCam-A`. Cage 5 published only three.
Every one carries a 4K60 `s3path` plus 720p, 360p and 72p proxies, and the fixed
feeds start within 5 ms of each other (`...54.778.mp4` against `...54.783.mp4`).

`playground/bgsub_cage/download_cage_video.py:43` filters all of that down to
`Cage-N-Overhead-High` because it is the angle that does not move. The other
four fixed angles have never been downloaded.

**The four side cameras sit inside the glass at roughly the height this migration
is aiming for.** `Cage-2-Red-High` in particular is close to the target
viewpoint: low, near the panel, looking across the mat, robot large in frame.

**Camera motion, measured by ORB plus RANSAC homography on frame corners:**

| feed | baseline | corner shift |
| --- | --- | --- |
| `Cage-2-Overhead-High` (1080p) | 35 s to 109 s | 1.36 to 2.51 px |
| `Cage-2-Blue-High` (4K) | 2 s | 0.47 px |
| `Cage-2-Blue-High` (4K) | 60 s | 38.84 px |

The overhead camera holds still. The side cameras hold still over a second and
drift over a minute, so a one-shot rig calibration will not survive a fight.
Zero parallax within a clip also means no single feed can be reconstructed on
its own. Multi-camera or a moving camera is the only way to get 3D.

**What the frames show.** The overhead view is a near top-down of a scuffed grey
mat with painted NHRL logos and teal and pink corner triangles, which is the
surface behind the arena-logo false positives in
`logo_false_positive_2026-09-03.md`. The side views show the hard part: dozens of
linear LED tubes overhead, strong specular glare and bloom, purple and cyan
ambient wash, a dark crowd bowl behind the panels, and polycarbonate that is
scratched, dusty and throwing double reflections.

Geometry here is close to a textured box. Lighting and glass are the difficulty.

## What exists to build on

| Layer | State | Where |
| --- | --- | --- |
| Renderer runtime | BlenderProc on Blender 4.2.1, Cycles, GPU Docker, resumable | `training/synthetic/`, `Dockerfile` |
| Annotation and gating | Pure, unit-tested, drop-reason taxonomy, YOLO pose and seg writers | `synthgen/annotations.py`, `gating.py`, `tests/test_purity.py` |
| Robot assets | 2 OnShape CAD robots with part-color to PBR mapping, 148 Meshy `nhrl_*.glb` with 146 keypoint sidecars | `training/data/models/`, `distractor_models/robots/` |
| Sim to C++ seam | TCP: `w*h*3` uint8 RGB, `w*h` float32 depth, `fx fy cx cy`, ground-truth poses, filling `CameraData` | `src/simulation/sim_connection.cpp:151-215` |
| Prior 3D sim | Genesis rendered images fed the full NN stack, demoted to make the kinematic sim primary | deleted `config/simulation/genesis.toml`, commit `3a0cc47d` |
| Real intrinsics | Measured, 95.2 deg HFOV at 1920 | `config/cameras/brettzone_cage_high.toml` |
| Cage footage | 241 mp4 local, 78 Overhead-High clips, 28 GB | `data/downloads/` |
| 360 capture | Own RICOH THETA V captures dated 2024-09-01 | `simulation/assets/panoramas/drive_test_box_*.JPG` |

The seam is the part worth noticing. Any Python process that produces an RGB
frame and intrinsics can drive the real C++ perception stack in lockstep today.
`sim_rgbd_camera.cpp` still compiles and is still registered.

**What does not exist.** There is no cage. `synthgen/environment.py` is 114 lines:
one ground plane, a random ambientCG texture from a list of about 500 arbitrary
surfaces, a random PolyHaven HDRI, and 1 to 3 white point lights at random
positions with energy 10 to 2000 W. No walls, no panels, no ceiling, no fixtures.
`simulation/assets/cage/floor.obj` is a 4-vertex quad and nothing in
`training/synthetic/` reads it.

There are also no camera intrinsics anywhere in the render path. Every synthetic
frame to date was rendered at Blender's default 50 mm on a 36 mm sensor, about
39.6 deg horizontal, as an undistorted pinhole. The deployed camera is 104.6 deg
spec with an uncalibrated wide M12 lens behind polycarbonate. Two config keys
that look like they cover part of this are dead: `light_color_temp_range` is
never parsed (`configuration.py:4-8`) and `hue_jitter_degrees` is explicitly
deleted (`materials.py:367`).

## The prior that should set expectations

This repo has already measured synthetic-to-real transfer twice, carefully.

`synthetic_arms_2026-07-31.md`, three arms, identical training, graded on the
independent 372-frame eval:

| arm | precision | recall | F1 | mAP50-95 |
| --- | --- | --- | --- | --- |
| real_only | 0.894 | 0.864 | 0.879 | 0.543 |
| mixed | 0.944 | 0.847 | 0.893 | 0.560 |
| synth_only | 0.538 | 0.553 | 0.545 | 0.263 |

Mixed beats real on precision by 0.049, CI [+0.030, +0.069]. Recall did not move,
and recall was the registered adopt criterion, so the verdict was keep real-only.
Synthetic-only loses 0.30 to 0.38 on every metric.

Two results matter more than the headline.

**Synthetic-to-real works when the mesh is right.** In the same Meshy model,
synthetic `mrs_buff_mk3`, an exact OnShape export, detects the real robot at
0.383 AP. The four Meshy opponents ranged 0.030 to 0.209 against a 0.084 floor
and a 0.210 real-trained ceiling, and the report traces the failures to stale
source thumbnails rather than reconstruction quality
(`meshy_grade_2026-07-16.md:14,54-58`).

**Adding synthetic widened the domain gap.** A linear probe separating real from
synthetic crops in backbone feature space scored 0.944 on the arm that never saw
a render and 0.962 on the arm trained on 17,995 of them. The cut-paste probe says
the same thing from the other side: pasted on blank grey, a real-trained detector
retains 30% of its score, a synthetic-trained one 127%. It scores a robot higher
on grey than in a real arena.

That probe is the sharpest instrument in the repo for this project, and it gives
a success criterion that does not require labelling anything: **a photoreal
renderer succeeds when probe accuracy goes down.** Every prior renderer pushed it
up.

The operator's own note is already on file: "Both corpora are ~98% synthetic and
share all 497 real frames, so swapping them changes which renderer I overfit, not
whether the model has seen a cage. Real cage footage is still the lever."

## Option 0: answer the migration question with real labels

Before any renderer. The question is whether the models work at the new position,
and there are 78 downloaded clips plus four untouched fixed angles per fight, one
of which is nearly the target viewpoint.

1. Extend `download_cage_video.py` to keep `Cage-N-Red-High` and
   `Cage-N-Blue-High` alongside the overhead feed. The regex at line 43 is the
   only change; `find_overhead` becomes `find_fixed`.
2. Build a labelled subdataset per `docs/adding_eval_recordings.md`:
   `make_eval_dataset.py`, `edit_labels.py`, then `validate_yolo_dataset.py` to
   write `validation_state.json`.
3. Grade the deployed engines with `score.py` against it.

Cost is labelling time, roughly 100 frames per view. It produces the first
ground truth at any mount other than the robot-mounted ZED, which every one of
the 688 `pass` frames in `nhrl_keypoints_eval_test` currently is. It is also a
hard prerequisite for validating any simulator, because without it there is
nothing to score a rendered arm against.

Watch the contamination trap. `nhrl_robots_bbox_2class` already contains 22
cage-high scenes, and `rembg_cage_high_plan.md:157` found 18 of them in training.
Its newest cage-high scene is 2026-03-07, so pick clips from 2026-04-04 or later
and the overlap is zero, as `stationary_bgsub_methods_2026-09-07.md:100-105`
verified.

## Option A: build the cage, capture the light

Model the cage by hand from measurements. Light it with a real HDRI shot inside
the cage. Render in the existing BlenderProc pipeline.

Geometry is a weekend of modelling: an octagonal mat 2.30 to 2.40 m across, wall
panels, the steel frame, the ceiling truss and the LED tube fixtures as emissive
geometry in their measured positions. Surface detail is what sells it, and it
comes from the footage already on disk. The overhead feed is a rectified,
metrically calibrated, near top-down view of the mat, so a mean or median
composite over a few hundred robot-free frames gives a clean orthographic albedo
texture of the floor, scuffs, logos and all, straight through the existing
homography.

Lighting is where the THETA V matters. A bracketed 360 capture from inside the
cage under match lighting produces a real HDRI, and Cycles takes it directly.
The existing panoramas do not substitute: they are 8-bit JPEGs, and `havoc*.jpg`
carries no EXIF I could read, so I cannot say what it depicts or under what
lighting. LDR is the operative limit either way. An HDRI captures the LED bars
at their true intensity ratio, which is what drives the specular highlights on
bare aluminium and the bloom.

What this option gets right: exact labels for free, full control over robot pose
and count, the whole `gating.py` and `annotations.py` layer carries over
unchanged, and it plugs into `SimConnection` for closed-loop runs.

What it will get wrong: hand-modelled geometry is approximate, and the glass is
hard to match by eye.

## Option B: reconstruct from video

What you described: feed it footage, get an environment.

Feasible, with one correction about which footage. The five fixed cameras are far
too sparse a baseline for structure-from-motion, and they drift. The
reconstruction asset is the moving cameras: `MobileCam-1/3/4` and `PTZ-Bowl-1`
sweep the cage from many viewpoints across a fight, which is what COLMAP wants.
Better still is a deliberate capture, two minutes walking a phone around and
above the cage between matches. That single clip will beat anything scavenged
from broadcast feeds, and it costs one event trip.

Two reconstruction targets, with different failure modes:

**Photogrammetry to a textured mesh.** COLMAP or GLOMAP for poses, then a dense
mesh. Gives geometry Blender can relight, which keeps the Option A lighting story
and the exact-label story intact. Quality on flat scuffed surfaces is good.

**3D Gaussian splatting.** Highest appearance fidelity per hour spent, and it
reproduces the glare and reflections that are hardest to model. The cost is that
a splat bakes the lighting it was captured under, so you cannot relight it, and
compositing a Cycles-rendered robot into it with correct shadows and correct
specular response is unsolved in this pipeline. Neither COLMAP nor gsplat is
installed; ffmpeg and Blender 5.2.1 are.

Both fail on the same thing. Polycarbonate is transparent, specular and
scratched, and multi-view stereo cannot triangulate it. Expect the panels to
reconstruct as noise or vanish. Model them analytically instead: a glass shader
at index 1.586, measured thickness, with a scratch and dust roughness map.

## Option C: hybrid

Reconstruct the static cage for background and geometry, keep the robots as CAD
in Cycles, light both from the captured HDRI, composite with real shadow
catchers. This is the highest ceiling and the most moving parts. It is Option A
plus Option B, and it only makes sense once one of them has been shown to move
the domain probe.

## Option D: image-space translation, no 3D at all

Skip the renderer. Take existing synthetic frames and learn a mapping onto NHRL
cage appearance with CycleGAN or a diffusion image-to-image pass conditioned on
real cage frames. Labels survive because geometry does not change.

Cheapest by a wide margin and it attacks the measured problem, the domain gap,
head on. It also risks the failure this repo has already documented once: a
model that looks right and moves detector metrics the wrong way, with no physical
account of why. Worth a one-day probe on the linear-probe metric before spending
a week on geometry, because it would falsify or support the whole premise quickly.

## Comparison

| | effort | appearance fidelity | labels | relightable | new-viewpoint control |
| --- | --- | --- | --- | --- | --- |
| 0. Real labels | low | real | hand-labelled | n/a | only the 5 fixed mounts |
| A. Built cage | medium | medium | exact, free | yes | any |
| B. Photogrammetry | medium-high | medium-high | exact, free | yes | any |
| B. Splatting | medium | high | hard | no | any |
| C. Hybrid | high | high | exact, free | yes | any |
| D. Translation | low | high | inherited | n/a | none |

## The sensor model, shared by every option

None of the options is worth much without this layer, and it does not exist today.
A rendered frame becomes a camera frame through the following stages, all
measured rather than guessed.

- **Intrinsics from the real calibration.** Load `config/cameras/<serial>.toml`
  and set Blender's focal length and sensor size to match `fx`, `fy`, `cx`, `cy`.
  Nothing in `synthgen/` writes intrinsics today.
- **Distortion by round trip, not by omission.** Render pinhole at a wider FOV
  than the target, apply the measured `k1 k2 p1 p2 k3` with `cv2.remap`, then
  apply the same rectification the camera applies in
  `src/rgbd_camera/camera_calibration.cpp`. The detector sees rectified frames in
  the field, so what matters is not raw barrel distortion but the resampling
  softness and edge stretch that rectification leaves behind. Running the round
  trip reproduces exactly that.
- **Motion blur from real velocity.** The AR0234CS is global shutter, so skip
  rolling shutter entirely and model exposure instead. Blur length is exposure
  time times robot velocity. Today `imaging.py:33-69` applies an OpenCV line
  kernel at a uniform random angle 0 to 2 pi, uncorrelated with any motion. In a
  sim that owns robot velocity, use Cycles motion blur or at least point the
  kernel along the velocity vector.
- **Noise fit from real frames.** Poisson shot plus Gaussian read, with
  parameters fit from flat-field captures at the gain the camera will run.
- **Vignetting and lateral chromatic aberration** fit from the same flat field.
- **Colour transfer.** A 3x3 matrix fit from a colour target shot through an
  offcut of cage panel, which the migration plan already asks for in section 6.
- **Glare and bloom.** If the LED tubes are emissive geometry with correct
  intensity, Cycles produces this. Otherwise it has to be added in post, and
  post-hoc bloom is the artifact most likely to read as fake.

Note one seam detail: `sim_connection.cpp:209` hard-zeros
`camera_info.distortion`, so the sim path currently assumes a rectified pinhole.
That matches the plan above, since the round trip ends rectified.

## Validation gates

Three, in increasing cost. Do not skip to the third.

**1. Domain probe, no labels needed.** Run the
`interpret_context_vs_appearance.py` feature-space probe on real cage crops
against rendered crops. Baseline to beat is 0.944 real-only and 0.962 mixed.
A renderer that drops this below 0.90 is doing something no previous renderer in
this repo did. This gate costs an afternoon and can be run on every iteration of
the cage model.

**2. Image statistics.** Compare rendered and real cage frames on gradient
histograms, noise power spectrum and highlight distribution. Cheap, and it
catches the specific ways a Cycles render reads as synthetic.

**3. `score.py` against Option 0's labelled cage-high subdataset.** The real
gate. Train an arm on rendered frames at the target mount, grade it on real
labelled frames from that mount, compare against a real-trained arm. Use the same
paired bootstrap the prior experiments used, and register the adopt criterion
before running it, because `synthetic_arms` shows how easy it is to land a
significant gain on an unregistered metric and a null on the registered one.

## What I would do

Option 0 first, on its own merits. It answers the migration question directly,
it is the only path to ground truth at the new mount, and gate 3 is impossible
without it.

Then Option A, with Option D run as a one-day probe alongside it purely to test
the domain-gap premise before committing to geometry. Option A extends
infrastructure that already works, reuses the annotation and gating layer
unchanged, and produces exact labels at any camera pose, which is the thing the
migration actually needs before the camera exists.

Option B is the one to reach for if Option A's cage looks obviously wrong next to
real footage, and the input should be a deliberate phone orbit at the next event,
not broadcast feeds. Add the 360 bracketed capture to the same trip.

Hold Option C until one of the others has moved gate 1.

## Open questions

- **Can we get inside a cage between matches with a tripod?** The 360 HDRI
  capture and the phone orbit both need it, and both are worth more than any
  amount of modelling from footage. Same question the fiducial board raises in
  the migration plan section 2.
- **Which cage?** Cage 1 publishes six fixed cameras, cage 2 publishes five,
  cage 5 publishes three. Cage 1 is the best reconstruction target and the
  `Ceiling-High` feed is a free extra viewpoint.
- **Do mat dimensions differ per cage?** The migration plan already flags 8 ft
  nominal running about 6% long against measured 2.30 to 2.40 m mats. The floor
  texture composite depends on the same number.
- **Is the LED fixture layout the same across cages and events?** If NHRL
  re-rigs, an HDRI has a shelf life and the emissive-geometry model is the more
  durable of the two.
- **Does the 4K source beat the 1080p proxy for texture extraction?** The
  overhead feed is 3840x2160 at source and range-seekable, and the floor texture
  is the one asset where the extra resolution plausibly pays.

## Next steps

1. Widen the camera regex in `download_cage_video.py:43` and pull `Red-High` and
   `Blue-High` for six fights from 2026-04-04 or later, which keeps the corpus
   contamination-free.
2. Label roughly 100 `Red-High` frames and run `score.py` on the deployed
   engines. This is the migration answer, and it lands before any renderer.
3. Composite a floor albedo texture from robot-free overhead frames through the
   existing homography, as a standalone script with an inspectable output.
4. Run the domain probe on current `training/data/synthetic/` renders against
   real cage crops to record the starting number before changing anything.
