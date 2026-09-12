# MassDestruction arena scene, from one broadcast clip

Builds a Blender scene of the MassDestruction arena (Charles River Museum, Resurgence Six)
tuned to the Omega broadcast camera, and grades renders against a robot-free frame from that
feed. Everything under `runs/massd_scene/` is generated and gitignored.

The NHRL cage-high flow in `../cage_scene/` is the template. Two things differ:

- **The camera is uncalibrated.** It is a YouTube broadcast, so the focal length is fitted
  with the pose from the floor's two vanishing points. `fit_massd_camera.py` does it.
- **One frame, not a median.** The fights run the whole clip, so a per-pixel median still
  carries robot ghosts. A between-round frame is genuinely empty; `--scan` ranks candidates
  and a human picks one.

## Pipeline

```bash
source scripts/activate_python.sh

# 1. Find an empty frame, then keep it. Banners are cropped to 1920x886 (rows 96..982),
#    the crop data/downloads/mass_destruction/MANIFEST.md uses for 1080p streams.
venv/bin/python playground/massd_scene/extract_target.py \
    data/downloads/massd_resurgence6_mrsbuff/r1_beeroll_vs_mrsbuff.mp4 --scan 12
venv/bin/python playground/massd_scene/extract_target.py \
    data/downloads/massd_resurgence6_mrsbuff/r1_beeroll_vs_mrsbuff.mp4 \
    --frame 8140 --event resurgence6 --out runs/massd_scene
# -> targets/<clip>/{target.png, hull_mask.png, meta.json}

# 2. Focal length and pose together, from the floor. Scale in is --mat-size.
venv/bin/python playground/massd_scene/fit_massd_camera.py runs/massd_scene
# -> camera_rect.json, poses/<clip>.toml, poses/summary.csv, poses/<clip>_overlay.png
# Check summary.csv says "ok" and look at the overlay: the fit is exact by construction,
# so there is no residual to read.

# 3. Orthographic floor albedo, warped through the fitted pose (the cage-high tool, unchanged).
venv/bin/python training/synthetic/build_cage_floor_texture.py runs/massd_scene \
    --px-per-m 1024 --name massd --out training/data/environments/massd_arena/mat_albedo

# 3b. The scene loads its camera from training/data, not from runs/: copy the fit across.
cp runs/massd_scene/poses/r1_beeroll_vs_mrsbuff.toml \
    training/data/environments/massd_arena/camera/massd_resurgence6_broadcast.toml
cp runs/massd_scene/camera_rect.json training/data/environments/massd_arena/camera/

# 4. Render the empty arena from the fitted pose.
training/synthetic/docker/run_synthetic.sh --gpu auto-battlebot-synthetic blenderproc run \
    /workspace/training/synthetic/render_cage_view.py -- \
    --spec /workspace/training/synthetic/cage/massd_resurgence6.toml \
    --pose /workspace/training/data/environments/massd_arena/camera/massd_resurgence6_broadcast.toml \
    --camera-rect /workspace/training/data/environments/massd_arena/camera/camera_rect.json \
    --out /workspace/runs/massd_scene/renders/final --samples 256

# 5. Grade it (the cage-high grader, unchanged).
venv/bin/python playground/cage_scene/grade_render.py \
    --renders runs/massd_scene/renders/final --targets runs/massd_scene/targets \
    --out runs/massd_scene/grades/final

# 6. Sweep a spec value.
venv/bin/python playground/cage_scene/sweep_spec.py runs/massd_scene \
    --spec training/synthetic/cage/massd_resurgence6.toml \
    --variants lights.json --prefix lt --samples 64 --pose-glob "r1_*.toml"

# 6b. Robot lighting against real frames: the hand-drawn keypoints in
#     training/data/nhrl_cage_high_eval/<clip> stand in for the NHRL flow's pose model.
venv/bin/python playground/massd_scene/robot_frames_from_labels.py runs/massd_scene \
    training/data/nhrl_cage_high_eval/r1_beeroll_vs_mrsbuff
# -> robot_frames/<clip>_f<frame>/{frame.png, pose.json, overlay.png}
venv/bin/python playground/cage_scene/sweep_spec.py runs/massd_scene \
    --spec training/synthetic/cage/massd_resurgence6.toml \
    --variants gain.json --prefix rg --samples 64 --pose-glob "r1_*.toml" \
    --robot-frames runs/massd_scene/robot_frames

# 7. Labelled robot samples from the fitted pose.
training/synthetic/docker/run_synthetic.sh --gpu auto-battlebot-synthetic blenderproc run \
    /workspace/training/synthetic/render_cage_samples.py -- \
    --spec /workspace/training/synthetic/cage/massd_resurgence6.toml \
    --config /workspace/training/synthetic/config.toml \
    --poses /workspace/training/data/environments/massd_arena/camera \
    --pose-glob "massd_*.toml" \
    --camera-rect /workspace/training/data/environments/massd_arena/camera/camera_rect.json \
    --out /workspace/runs/massd_scene/samples --num-images 100 --samples 128 --seed 0
```

## Fitting the focal length

The plywood floor is a square seen corner-on, so its two edge families have two vanishing
points. With square pixels and the principal point at the image centre their orthogonality
gives one equation for `f`:

    (u1 - cx)(u2 - cx) + (v1 - cy)(v2 - cy) + f^2 = 0

Both vanishing points are found by RANSAC over LSD segments inside the floor hull. Nearly
every straight line in the arena runs along one of the two floor axes: the kick rails, the
painted squares, the pit rim, the plywood seams. With `f` in hand the two axis directions
fix the plane's orientation, and three floor edges fix the rest.

Checks worth trusting more than the (zero) residual:

- The fitted floor comes out square to 1.2% when the aspect is not constrained.
- The bright band above the far floor edge, which is the kick rail's inner face, reads the
  same height within 5 px at four points spread along that edge.

Steps 1 and 2 write to `runs/massd_scene`, which is scratch. What the Blender scene actually
loads lives in `training/data/environments/massd_arena/`; step 3 and 3b put it there.

## Scene spec

`training/synthetic/cage/massd_resurgence6.toml`, expanded by
`training/synthetic/synthgen/cage_spec.py` and built by `synthgen/cage.py`, the same pair the
NHRL cage uses. What MassD added to them:

- `[[pits]]` — a rectangular hole. The mat and the riser are cut around it and four walls and
  a floor close it off below.
- `panel.walls` — which walls get glass. BlenderProc's segmentation stops at glass, so a pane
  between the camera and the mat costs every label behind it. This camera is outside the
  cage, so only the two walls it looks across are glazed.
- `frame.bolt_face` — bolts through the rail's inner face rather than standing on its top,
  which is what a tall wooden kick rail shows.
