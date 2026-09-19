# Meatball basement scene: the drive-test box, from our own ZED

Builds a Blender scene of the plywood drive-test box in the basement, fitted to one frame
of our own ZED on a stand beside it, and grades renders against that frame. Scratch goes to
`runs/basement_scene/` (gitignored); what the scene loads lives in
`training/data/environments/meatball_basement/`.

The MassD flow in `../massd_scene/` is the template. Three things differ:

- **The camera is known and has depth.** The source is an SVO, so K comes from the
  recording and the SDK gives metric XYZ. The floor plane, the box size, the rail height,
  both stone walls and the concrete below are measured from the depth rather than solved
  from vanishing points.
- **The walls are geometry.** The stone stands 2.5 cm behind the far rail and 7 cm off the
  right one. A panorama at infinity cannot stand in for that, so each wall is a textured
  quad (`[[walls]]` in the spec). The panorama covers everything else.
- **No frame shows the box empty.** Robots sit on the floor for the whole of every
  plywood-era recording, so the target is a real frame with the robots patched out.

## Source material

| What | Where |
| --- | --- |
| Target frame | `data/svo/tests/2026-04-19T17-01-18.svo2` frame 4560 on pathfinder, left view, 1280x720 |
| Panorama | `simulation/assets/panoramas/drive_test_box_1.JPG`, RICOH THETA V, 2024-09-01, the room without the box |

`drive_test_box_2.JPG` shows the old painted box, not this one; do not use it. Every
basement SVO is 1280x720. The April plywood recordings (`2026-03-27T*`, `2026-04-19T*`,
`2026-04-20T18-02-50`) show this box; the late-April ones show it painted white, and the
August ones show a rebuilt box with tape start squares.

## The target: frame 4560 with its robots patched out

Frame 4560 is the one frame that shows all four rails and corners. Three robots were
replaced, and nothing else in the frame was touched (1.4 percent of its pixels changed):

- back-left and centre: from frames 4710 and 4500 of the same session, where that patch of
  floor was empty, aligned locally with ECC and colour-matched on a ring around the robot;
- back-right corner: that robot never moved all session, so its patch comes from a
  robot-free plate, the median of 89 frames (2850 to 5490, one per second) registered to
  frame 4200 by SIFT homography, with its own corner filled from `2026-04-19T18-42-50`.

This was done by hand and is not scripted. The result and the mask of changed pixels are
kept in `training/data/environments/meatball_basement/target/`, and
`runs/basement_scene/targets/<clip>/meta.json` records the donor frames.

## Pipeline

```bash
source scripts/activate_python.sh

# 1. Depth, K and the left image for the target frame. Needs pyzed, so run it on pathfinder
#    (its venv has the SDK; pull this branch there first) and copy the results back.
ssh -i ~/.ssh/pathfinder ben@pathfinder 'cd ~/auto-battlebot && venv/bin/python \
    playground/basement_scene/export_svo_depth.py data/svo/tests/2026-04-19T17-01-18.svo2 \
    --frame 4560 --out /tmp/basement_depth'
T=runs/basement_scene/targets/2026-04-19T17-01-18_f4560
mkdir -p $T
scp -i ~/.ssh/pathfinder ben@pathfinder:/tmp/basement_depth/xyz.npy $T/
scp -i ~/.ssh/pathfinder ben@pathfinder:/tmp/basement_depth/camera_rect.json runs/basement_scene/
cp training/data/environments/meatball_basement/target/plate_2026-04-19T17-01-18_f4560.png \
    $T/target.png
cat > $T/meta.json <<'JSON'
{"clip": "2026-04-19T17-01-18_f4560", "event": "basement", "event_group": "basement",
 "cage": "meatball", "source_frame": 4560,
 "source": "data/svo/tests/2026-04-19T17-01-18.svo2 frame 4560 (pathfinder), left view"}
JSON

# 2. Pose: floor plane from depth, box centre and yaw from the rail edges.
venv/bin/python playground/basement_scene/fit_basement_camera.py runs/basement_scene
# -> poses/<clip>.toml, poses/<clip>_overlay.png, poses/summary.csv, targets/<clip>/hull_mask.png

# 3. Floor albedo (the cage-high tool, with the strip behind the near rail mirrored in).
venv/bin/python training/synthetic/build_cage_floor_texture.py runs/basement_scene \
    --px-per-m 1024 --name meatball --fill mirror \
    --out training/data/environments/meatball_basement/mat_albedo

# 4. Stone-wall albedos; prints the [[walls]] blocks for the spec.
venv/bin/python playground/basement_scene/build_wall_textures.py runs/basement_scene \
    --out training/data/environments/meatball_basement/walls

# 5. Copy the fit to where the scene reads it.
cp runs/basement_scene/poses/2026-04-19T17-01-18_f4560.toml \
    training/data/environments/meatball_basement/camera/meatball_basement_zed_stand.toml
cp runs/basement_scene/camera_rect.json training/data/environments/meatball_basement/camera/

# 6. Render from the fitted pose, grade, sweep (the cage-high tools, unchanged).
training/synthetic/docker/run_synthetic.sh --require-gpu auto-battlebot-synthetic blenderproc run \
    /workspace/training/synthetic/render_cage_view.py -- \
    --spec /workspace/training/synthetic/cage/meatball_basement.toml \
    --pose /workspace/runs/basement_scene/poses --pose-glob '2026-*.toml' \
    --camera-rect /workspace/runs/basement_scene/camera_rect.json \
    --out /workspace/runs/basement_scene/renders/final --samples 256 \
    --save-blend /workspace/runs/basement_scene/meatball_basement.blend
venv/bin/python playground/cage_scene/grade_render.py --renders runs/basement_scene/renders/final \
    --targets runs/basement_scene/targets --out runs/basement_scene/grades/final
venv/bin/python playground/cage_scene/sweep_spec.py runs/basement_scene \
    --spec training/synthetic/cage/meatball_basement.toml --variants <variants.json> \
    --prefix lt --samples 64 --pose-glob "2026-*.toml"

# 7. Labelled robot frames in the basement (the generic pipeline, pinned to this venue).
training/synthetic/docker/run_synthetic.sh --require-gpu auto-battlebot-synthetic blenderproc run \
    render_scenes.py -- config_cage_meatball.toml --num-images 20 --out ../data/synthetic/sample/basement
```

## What the fit measured

From the ZED depth of the target frame, in the box frame (floor centre, z up, +y away from
the camera):

| Quantity | Value | How |
| --- | --- | --- |
| Floor plane | 2.2 mm rms over 64,449 points | RANSAC + SVD on plywood-coloured pixels |
| Camera | 0.564 m above the plywood, 1.36 m back from its centre, 63.9 deg off straight down, 0.4 deg yaw | plane, then rail-edge fit |
| Floor inside the rails | 1.52 m square | rail-top peaks in the depth; taken as 5 ft |
| Rail top | 0.0875 m above the plywood | depth histogram |
| Rail outer face | 0.088 m down to -0.035 m | depth down the near rail |
| Far wall | y = 0.825 m, 0.2 deg off square | depth behind the box |
| Right wall | x = 0.871 m, 0.8 deg off square | depth right of the box |
| Concrete | 0.78 m below the plywood | depth under the box |

The rail-edge fit lands the projected rail and floor edges 2.17 px from the target's Canny
edges on average. Depth alone put the right rail 5 cm off: the camera looks straight at its
inner face and the stereo smears it, which is why the fit ends on the image edges.

## Grade

`render_cage_view.py` from the fitted pose at 256 samples against the target, final spec:

| Region | L1 gray | SSIM | Edge chamfer | Lab dL |
| --- | --- | --- | --- | --- |
| Floor (inside the hull) | 4.17 | 0.973 | 0.32 px | -0.8 |
| Everything else | 33.4 | 0.506 | 3.02 px | -3.7 |

The floor numbers are the ones to trust. Outside the floor the target has the hoist, a
bucket, a bag and shelving that the scene does not model, so the panorama stands in there.
The lights, the albedo gains and the panorama strength came from two sweeps
(`runs/basement_scene/sweep_light*.json`): full-frame L1 went from 30.3 to 26.4 at 64
samples.

## Known gaps

- **The near rail's outer face** is lit in the photo and grey in the render: something
  behind the camera lights it, and the scene has only overhead tubes.
- **The legs are a stand-in.** The real support is hidden behind a bucket and a bag in
  every frame; the four 2x4 legs only stop the box floating.
- **Wall texture away from the target view** is tiled from one seen rectangle per wall
  (0.55 x 2.24 m far, 0.6 x 1.2 m right), so it repeats at that period.
- **The panorama is an 8-bit JPEG** from 2024, shot before the box moved in. It is backdrop
  and ambient fill only.
