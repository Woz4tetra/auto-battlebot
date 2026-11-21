# Cage-high scene pipeline details

## Target extraction (`playground/cage_scene/extract_targets.py`)

Robot-free, rectified target frames from the fixed cage-high clips.

One target per clip: the per-pixel median over frames spread across the fight (robots move, so each pixel shows bare floor in most samples), remapped through the same rectification the C++ camera path applies, so a pinhole render at the rectified K lines up with it. The clip's cached hull polygon is undistorted the same way and rasterised beside it.

Output layout:

```
    <out>/camera_rect.json                 rectified K for every later stage
    <out>/targets/<clip>/target.png        1920x1080 rectified median frame
    <out>/targets/<clip>/hull_mask.png     255 inside the mat hull
    <out>/targets/<clip>/meta.json         event, cage, game, source url, sample count
    <out>/targets/<clip>/target_4k.png     with --fetch-4k: rectified median of N remote 4K frames
```

```
Usage:
    venv/bin/python playground/cage_scene/extract_targets.py \
        data/downloads/brettzone_cage_high --cage 2 --weight-class 3lb --out runs/cage_scene
    venv/bin/python playground/cage_scene/extract_targets.py \
        data/downloads/mrsbuff_may26 --cage 2 --out runs/cage_scene
```

## Camera fit (`playground/cage_scene/fit_cage_camera.py`)

Fit the cage-high camera pose per clip from the rectified mat hull.

For each `targets/<clip>/` written by extract_targets.py: fit straight lines to the mat outline where it is actually visible (the near edge runs off the bottom of the cage-high frame, so usually three sides), solve the pose against the known mat size at the rectified K by least squares from a seed pose, and check the answer is a camera above the mat at a plausible height and tilt. The seed is the true_battlebot Cage 2 prior when it exists, else the four-corner homography. A clip whose outline is fully in frame is solved from four corners first and refined on its four lines. Poses are written in the `config/cages/*.toml` format the C++ `CalibratedFieldFilter` reads, one per clip and, with --event-pose, one per event (median translation, chordal-mean rotation) for rendering sweeps.

`--landmarks <json>` adds hand-picked correspondences `[{"image": [u, v], "field": [x, y, z]}]` in hfield metres (see `auto_battlebot.perception.cage_calibration`) and refines the homography pose with solvePnP. Use it only when the grading says the far edge is off.

```
Usage:
    venv/bin/python playground/cage_scene/fit_cage_camera.py runs/cage_scene --event-pose
```


## Render grading (`playground/cage_scene/grade_render.py`)

Grade cage renders against the robot-free target frames, per region.

Each render `<stem>.png` under --renders is paired with the target frames it stands for: the clip of the same name, or every clip of the event an event pose (`cage2_<event>`) was fitted from. Metrics are computed on the full frame, inside the mat hull, and outside it, and are written in long form to `metrics.csv` with a `source` column so baselines sit beside the render:

```
    render            the render under test
    baseline_mean     the pixel mean of all targets, scored against each target (the
                      clip-to-clip floor from exposure and mat wear)
    baseline_flat     the target's own mat-mean colour painted everywhere
    baseline_previous --baseline-render, another render directory, if given
```

Metrics: l1_gray, l1_rgb (0..255, at --scale), ssim_gray (11x11 Gaussian, sigma 1.5), edge_chamfer_px (symmetric Canny edge distance, full resolution), lab_dmean_L/a/b, lab_dstd_L/a/b, mat_iou (overlap of the render's mat mask with the target hull, full-frame only). `report.md` holds the per-region means and a per-clip table, and `<stem>_sheet.png` shows target | render | abs diff | edge overlay.

```
Usage:
    venv/bin/python playground/cage_scene/grade_render.py \
        --renders runs/cage_scene/renders/r001 --targets runs/cage_scene/targets \
        --out runs/cage_scene/grades/r001
    venv/bin/python playground/cage_scene/grade_render.py --compare runs/cage_scene/grades/r001 \
        runs/cage_scene/grades/r002
```
