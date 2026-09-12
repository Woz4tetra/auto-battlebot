# Cage-high scene: NHRL 3 lb cage from the fixed Cage-2-Overhead-High camera

Builds a Blender scene of the NHRL 3 lb cage tuned to one vantage point, NHRL's fixed
`Cage-2-Overhead-High` iPhone feed, and grades renders against robot-free frames from the
downloaded clips. Everything under `runs/cage_scene/` is generated and gitignored.

Cage 1 is the 12 lb / 30 lb cage and has a larger mat; the 3 lb fights are in Cage 2 (and
one May clip in Cage 5). Filter by weight class, not cage number, before fitting a 2.35 m mat.

## Pipeline

```bash
source scripts/activate_python.sh

# 1. Robot-free rectified targets: median frame per clip, undistorted with the measured
#    iPhone intrinsics (config/cameras/brettzone_cage_high.toml), hull mask beside it.
venv/bin/python playground/cage_scene/extract_targets.py data/downloads/brettzone_cage_high \
    --cage 2 --weight-class 3lb --out runs/cage_scene
venv/bin/python playground/cage_scene/extract_targets.py data/downloads/mrsbuff_may26 \
    --cage 2 --out runs/cage_scene
#    -> targets/<clip>/{target.png, hull_mask.png, meta.json}, camera_rect.json

# 2. Camera pose per clip from the visible mat edges (the near edge runs off the bottom of
#    the frame, so three lines), seeded by the true_battlebot Cage 2 prior; one pose per event.
venv/bin/python playground/cage_scene/fit_cage_camera.py runs/cage_scene --event-pose
#    -> poses/<clip>.toml, poses/cage2_<event>.toml, poses/summary.csv, poses/*_overlay.png
#    Check summary.csv: status "ok" means height 1.0-2.0 m, tilt 20-40 deg, |yaw| < 10 deg,
#    three sides seen. Clips that fail are left out of the event poses and textures.

# 3. Orthographic mat albedo per event, median over that event's clips.
venv/bin/python training/synthetic/build_cage_floor_texture.py runs/cage_scene --px-per-m 1024
#    -> training/data/environments/nhrl_3lb_cage/mat_albedo/cage2_<event>_albedo.png
#       (+ .json, _preview.png); --out moves it elsewhere

# 4. Render the empty cage from the event poses (Docker, GPU), solving exposure from the mat.
training/synthetic/docker/run_synthetic.sh --gpu auto-battlebot-synthetic blenderproc run \
    /workspace/training/synthetic/render_cage_view.py -- \
    --spec /workspace/training/synthetic/cage/cage2_overhead_high.toml \
    --pose /workspace/runs/cage_scene/poses --pose-glob 'cage2_*.toml' \
    --camera-rect /workspace/runs/cage_scene/camera_rect.json \
    --out /workspace/runs/cage_scene/renders/r001 --samples 64 \
    --auto-exposure /workspace/runs/cage_scene/targets \
    --save-blend /workspace/runs/cage_scene/cage2_overhead_high.blend
#    -> renders/r001/<pose>.png, <pose>_matmask.png, render_meta.json; the .blend

# 5. Grade against the targets, per region (full frame, inside the mat hull, outside).
venv/bin/python playground/cage_scene/grade_render.py --renders runs/cage_scene/renders/r001 \
    --targets runs/cage_scene/targets --out runs/cage_scene/grades/r001
#    -> grades/r001/{metrics.csv, report.md, <clip>_sheet.png}

# 6. Sweep spec values: renders each variant, grades it, prints one table.
venv/bin/python playground/cage_scene/sweep_spec.py runs/cage_scene --prefix h \
    --grid frame.height_above_mat 0.03 0.05 0.08
venv/bin/python playground/cage_scene/grade_render.py --compare runs/cage_scene/grades/r001 \
    runs/cage_scene/grades/h_height_above_mat_0.08

# 6b. Robot lighting against real frames: pick frames with MRS BUFF MK3, recover its pose
#     from the yolo26x-pose keypoints through the fitted camera, render it there, grade the
#     robot box, its shadow ring and the rest of the mat, and sweep lighting on that.
venv/bin/python playground/cage_scene/pick_robot_frames.py runs/cage_scene \
    data/downloads/mrsbuff_may26 --per-clip 3 --stride 90
#    -> robot_frames/<clip>_f<frame>/{frame.png, pose.json, overlay.png}
venv/bin/python playground/cage_scene/sweep_spec.py runs/cage_scene --prefix rl \
    --variants lighting.json --robot-frames runs/cage_scene/robot_frames --samples 64
#    -> renders/rl_<variant>/<frame>.png (+ _matmask, _robotmask), grades/rl_<variant>/report.md
#       and <frame>_robot_sheet.png (real crop | rendered crop)

# 7. 100 labelled samples: MRS BUFF MK3 plus Meshy opponents on the mat, YOLO pose labels.
training/synthetic/docker/run_synthetic.sh --gpu auto-battlebot-synthetic blenderproc run \
    /workspace/training/synthetic/render_cage_samples.py -- \
    --spec /workspace/training/synthetic/cage/cage2_overhead_high.toml \
    --config /workspace/training/synthetic/config.toml \
    --poses /workspace/runs/cage_scene/poses \
    --camera-rect /workspace/runs/cage_scene/camera_rect.json \
    --out /workspace/runs/cage_scene/samples --num-images 100 --samples 128 --seed 0
#    -> samples/{images, labels, data.yml, sheet.png}
```

Two spec values are load-bearing and non-obvious. `mat.albedo_gain` (0.35) darkens the
albedo photo so the lights, refitted to the mat by auto exposure, light the robots as bright
as the footage does; at 1.0 the robots render half as bright as real. `lights.grid` sets robot
shadow crispness through tube length and radius, and cage shadow placement through the rig
footprint; keep the footprint over the mat or the posts start casting new shadows. The house bot is the
textured GLB, which ships lying on its side: `house_bot_box.roll_deg` (-90) stands it up about
its +x LED-face normal, `yaw_deg` turns the face to the camera, `front_fill` is a camera-side
spot on its vertical face, and only texels with HSV saturation above 0.35 glow. `led_emission` is in output units: the
emission is divided by the solved exposure gain, so 1.0 puts the LED colour at full scale
without clipping to white; the grey body stays steel.

The scene spec is `training/synthetic/cage/cage2_overhead_high.toml`; `--set section.key=value`
overrides any entry (lists index as `lights.tubes.0.strength`). Geometry lives in
`training/synthetic/synthgen/cage_spec.py` (pure, unit-tested) and is turned into Blender
objects by `synthgen/cage.py`.

## Frames

Written once in `auto_battlebot/perception/cage_calibration.py`. The fit solves in the
homography ("hfield") frame, mat centre, z up, with the corner winding of the C++
`field_pose.cpp`; the Blender world W is that frame yawed by -90 deg so +y points away from
the camera. Poses are stored as `tf_camera_from_fieldcenter` in the `config/cages/*.toml`
format, so `CalibratedFieldFilter` can read them unchanged.

## Grading

`grade_render.py` writes long-form `metrics.csv` (clip, region, metric, source, value).
Sources: `render`, `baseline_mean` (the mean of all targets scored against each target, the
clip-to-clip floor from iPhone exposure and mat wear) and `baseline_flat` (the target's own
mat-mean colour everywhere). Metrics: `l1_gray`, `l1_rgb`, `ssim_gray`, `edge_chamfer_px`,
`lab_dmean_*`, `lab_dstd_*`, `mat_iou`. Lower is better except `ssim_gray` and `mat_iou`.

With `--robot-frames` the regions are the detected robot box, a shadow ring around it on the
mat, and the rest of the mat, plus `shadow_drop_gray` (how much darker the ring is than the
rest of the mat, render against target), `robot_box_iou` and the robot's mean grey in both.

## Opening the .blend

`runs/cage_scene/cage2_overhead_high.blend` is written by Blender 4.2.1 inside the container.
The host snap Blender (5.2) opens it for inspection; do not save back from 5.2, the pipeline
regenerates it. Texture and albedo paths inside are container paths under `/workspace`; the
host checkout is the same tree, so relink `/workspace` to the repo root if textures show pink.

## Tests

```bash
venv/bin/python -m pytest tests/python/test_cage_calibration.py -q
PYTHONPATH=training/synthetic venv/bin/python -m pytest training/synthetic/tests -q
```
