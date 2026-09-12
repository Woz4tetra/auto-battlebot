# Synthetic Data Generator for YOLO Keypoint Training

Generates randomized training images with a target robot (bounding box + front/back keypoints) and distractor objects. Outputs YOLO-format labels compatible with the existing training pipeline.

## Prerequisites

### Recommended: Docker (lean synthetic image)

Use the containerized workflow for reproducibility and to avoid host Python/BlenderProc conflicts.

Build from repo root:

```bash
docker build -f training/synthetic/Dockerfile -t auto-battlebot-synthetic training/synthetic
```

Run an interactive shell in the container:

```bash
training/synthetic/docker/run_synthetic.sh auto-battlebot-synthetic
```

Enable GPU passthrough only when needed:

```bash
training/synthetic/docker/run_synthetic.sh --gpu auto-battlebot-synthetic
```

Or force CPU mode:

```bash
training/synthetic/docker/run_synthetic.sh --cpu auto-battlebot-synthetic
```

Run a command directly:

```bash
training/synthetic/docker/run_synthetic.sh \
  auto-battlebot-synthetic \
  blenderproc run prepare_robot_model.py -- config.toml --inspect
```

You can run synthetic tools from this container, including:

- `python training/synthetic/download_polyhaven_hdris.py ...`
- `python training/synthetic/download_ambientcg.py ...`
- `python training/synthetic/download_objaverse.py ...`
- `blenderproc run prepare_robot_model.py -- config.toml --inspect`
- `blenderproc run render_scenes.py -- config.toml`

### Host setup (optional/manual)

Set up an environment:

```bash
cd training/synthetic
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -e .
```

> **Note:** The `blenderproc run …` scripts import sibling modules (`synthgen`,
> `nhrl_common`) by top-level name. BlenderProc uses Blender's own Python, not
> the project venv, so run them from `training/synthetic/` with
> `PYTHONPATH="$PWD"` (e.g. `PYTHONPATH="$PWD" blenderproc run render_scenes.py -- config.toml`).
> The `docker/run_synthetic.sh` wrapper sets `PYTHONPATH` automatically.

## Quick Start

### 1. Export your robot from OnShape

Export the assembly as GLTF (`.glb`). Place it at the path specified in `config.toml`.

### 2. Inspect the model's material colors

```bash
blenderproc run prepare_robot_model.py -- config.toml --inspect
```

This prints every material name and RGB color in the GLTF. Use the output to fill in `[[robots.color_mapping]]` entries in `config.toml`.

### 3. Set keypoint positions

In `config.toml`, set `robot.keypoints.front` and `robot.keypoints.back` to the 3D coordinates (meters, relative to model origin) of your robot's front and back keypoints.

### 4. Download environment assets

Download HDRIs:

```bash
python download_polyhaven_hdris.py ../data/hdris --count 20
```

Download textures referenced by config:

```bash
python download_ambientcg.py ../data/cc_textures --from-config config.toml
```

### 5. Acquire distractor models

Download from Objaverse:

```bash
python download_objaverse.py ../data/distractor_models/objaverse --max-models 100
```

You can also place additional `.glb`, `.gltf`, `.obj`, or `.ply` files in custom distractor directories and add them under `[[distractors.sources]]` in `config.toml`.

#### NHRL robot distractors (Meshy.ai)

Generate a large pool of real NHRL robots as CAD distractors from BrettZone
thumbnails. These run in the project venv (`python`), except the preview which
runs under `blenderproc run`. The pipeline is split into steps so each stage can
be inspected before the next; all state lives in the output directory and every
step is resumable. Run from `training/synthetic/`.

1. Fetch the roster and cache thumbnails (no Meshy credits spent):

   ```bash
   python download_nhrl_bots.py ../data/distractor_models/robots --limit 300
   ```

   Prints the ranked selection and which thumbnails resolved. Bots are ranked by
   total fights across all weight classes; bracket/bye placeholders are dropped.

2. Review thumbnails and reject the ones unfit for image-to-3D (busy
   backgrounds, multiple bots, side-on or tiny shots, logos). No Meshy credits
   are spent here. Rejections are recorded as a `rejected` flag in the state
   file, which both this step and step 1 honor, so a rejected bot is never
   re-downloaded or meshed. Run on the **host** in a root project venv (needs
   `opencv` + `numpy`; it needs a GUI display):

   ```bash
   ../../venv/bin/python review_nhrl_thumbnails.py ../data/distractor_models/robots
   ```

   `a` accepts, `r` rejects (moves the PNG to `rejected_thumbnails/`), `n`/`p`
   navigate, `u` clears a decision. You can also bulk-reject a list triaged
   elsewhere (tokens may be bare names, `<name>.png`, or `trash:///` URIs):

   ```bash
   ../../venv/bin/python review_nhrl_thumbnails.py ../data/distractor_models/robots \
     --reject-file rejected.txt
   ```

3. Generate meshes via Meshy image-to-3D. Requires `MESHY_API_KEY`. Start with a
   pilot to check quality and credit burn, then run the full batch (resumable):

   ```bash
   python generate_nhrl_meshes.py ../data/distractor_models/robots --dry-run    # preview submissions
   MESHY_API_KEY=... python generate_nhrl_meshes.py ../data/distractor_models/robots --limit 10
   MESHY_API_KEY=... python generate_nhrl_meshes.py ../data/distractor_models/robots --limit 300
   ```

   GLBs land as `nhrl_<name>.glb` and rows are appended to
   `distractor_gpu_audit.csv`. Re-running never resubmits in-flight tasks.

4. Compute front/back keypoints and render top-down previews. This runs under
   BlenderProc so each robot is rendered with its real materials (an
   orthographic top-down PNG in `topdown/`). It writes a `<stem>.json` sidecar
   with the keypoints plus a `topdown` block (image path, model->pixel affine,
   footprint hull, `y_ground`, default axis) that the review step uses to overlay
   an aligned centerline. The footprint's principal axis gives a centered
   orientation line; which end is "front" is left arbitrary here and confirmed in
   the next step. Runs in the container (mounts the repo, so outputs land on the
   host):

   ```bash
   docker/run_synthetic.sh --gpu auto-battlebot-synthetic \
     blenderproc run compute_nhrl_keypoints.py -- ../data/distractor_models/robots
   ```

5. Confirm front/back direction. The mesh alone can't tell which end is the
   robot's front (thrust/weapon direction), so this shows the top-down render
   from step 4 and lets you **drag to draw the centerline** (front = the end you
   drag toward). The centerline stays through the model's center at any angle,
   and front/back snap to the silhouette's extremes along it. `f` flips, `a`
   accepts. Confirmed sidecars are marked `reviewed: true` and are never
   clobbered by a later `compute` run (even with `--overwrite`).

   Run on the **host** in a root project venv (needs `opencv` + `numpy`; it loads
   only the sidecar PNG + JSON, no mesh). It needs a GUI display, so it will not
   run inside the synthetic Docker image or under `blenderproc`:

   ```bash
   ../../venv/bin/python review_nhrl_keypoints.py ../data/distractor_models/robots
   ```

6. Optional: `preview_nhrl_keypoints.py` still renders blenderproc 6-view grids
   (red = front, blue = back) if you want to double-check the confirmed keypoints
   from all sides before the full render.

In `keypoints_bbox` annotation mode, these distractors are annotated under a
generic `nhrl_robot` class with front/back keypoints, sit flat on the field with
robot-like pose randomization, and get their own motion-blur probability. Tune
`robot_air_probability`, `robot_air_height_range`, and `motion_blur_probability`
under `[distractors]` in `config.toml`.

### 6. Generate synthetic images

```bash
blenderproc run render_scenes.py -- config.toml
```

Options:

```text
--num-images 5000       Override image count from config
--images-per-scene 4    Camera viewpoints per robot arrangement
--out DIR               Write DIR/images and DIR/labels instead of the config's paths
--render-samples 128    Path-tracing samples per pixel (each cage has its own count)
--start-index 10000     Resume from a specific frame index
--seed 42               Seed Python/numpy RNGs for reproducible debugging runs
-v / --verbose          Debug logging (per-robot skip detail, asset decisions)
-q / --quiet            Warnings and the run summary only
```

Some images come out of a real arena rather than the HDRI arena: one `[[cages]]` entry per
arena, each with its own share of the run. Same robots, distractors and label pipeline, but
the mat is the floor, the arena's own light rig is the light, and the camera is clamped to a
wall instead of sampled on a shell around the robots. The shipped config ships two, the NHRL
3 lb cage at 50% and the MassDestruction arena at 25%, leaving 25% HDRI arena; the NHRL
cage's house bot is labelled as its own `house_bot` class with keypoints. The run summary
reports the realized split per arena. See
`docs/experiments/perception_performance/cage_scene_render_match_2026-09-11.md` and
`massd_arena_scene_2026-09-11.md` for how each was fitted to footage, and `training/data/environments/nhrl_3lb_cage/` for its assets.

Every dropped frame is logged with a machine-readable reason
(`DROPPED KP_PROMINENT_ROBOT_UNLABELED — robot 2 ...`), and the run ends with a
summary of images written vs requested plus drop counts by reason. If a run
ends short of the requested image count, a warning explains why.

`render_scenes.py` is a thin entry point; the implementation lives in the
`synthgen/` package next to it. Pure logic (config parsing, annotation math,
gating policy, YOLO writers) is separated from Blender-dependent code and is
unit-tested — run the tests from the repo root without Blender or Docker:

```bash
venv/bin/pytest training/synthetic/tests
```

Note: the Docker image `COPY`s this directory at build time, but
`docker/run_synthetic.sh` bind-mounts the live repo over it, so `synthgen/`
changes take effect without a rebuild. Rebuild only if you run the image
without the mount.

### 7. Assemble the dataset

Combine synthetic images with real labeled images and split train/val/test:

```bash
python ../split_yolo_dataset.py \
  data/synthetic/images \
  data/synthetic/labels \
  data/synthetic_dataset \
  --train 0.9 --val 0.1
```

Visualize annotations:

```bash
python ../draw_yolo_annotations.py data/synthetic_dataset/train
```

Train:

```bash
cd ../../yolo
python train.py path/to/data.yaml yolo11n-pose
```

### Choosing where the cage camera goes

`render_scenes.py` draws each cage camera pose from the `[cages.mount]` ranges in `config.toml`.
To pick those ranges by eye rather than by editing numbers, fly the camera around the built cage:

```bash
training/synthetic/docker/run_synthetic.sh --gpu --port 8770 \
  auto-battlebot-synthetic blenderproc run pose_camera_server.py -- \
  --spec cage/cage2_overhead_high.toml \
  --camera-calibration ../../config/cameras/ecam25_h01r1_estimated.toml \
  --out ../data/cage_pose
```

Wait for `serving on port 8770` (about 10 seconds: the cage build plus one throwaway frame that
pays the OptiX kernel compile up front), then open <http://127.0.0.1:8770> and click the preview to
take the mouse.

| Key | Does |
| --- | --- |
| `W` `A` `S` `D` | move, level with the mat: the heading comes from yaw alone, so looking down does not fly you into the floor |
| `Space` / `Shift` | rise / fall |
| `Ctrl` | hold for fine movement |
| `Z` `X` | roll about the optical axis |
| `V` | cycle `pinhole` -> `distorted` -> `rectified` |
| `R` | rectification alpha, 1.0 or 0.0 |
| `B` | show the MRS BUFF MK3 mesh at the mat center and all eight compass directions (the first press stalls while it loads) |
| `N` | snap onto the nearest mount the sampler could actually draw |
| `M` / `C` | mark the current pose / clear every mark |
| `F` | render the current pose at full resolution and the spec's sample count |
| `Enter` | write the outputs |
| `Esc` | give the mouse back |

View, alpha, robots and full render are also buttons in the Controls panel, for when the mouse is
not captured. Each button shows the current value.

`R` switches the rectification alpha, the same `getOptimalNewCameraMatrix` alpha the C++
`Rectifier` passes. At 1.0 every source pixel is kept, so the frame is wider than the lens and
carries a black border; at 0.0 it is cropped to the largest all-valid rectangle, with no border and
a narrower field. On the estimated e-CAM25 calibration at 1280x720 that is fx 384, 118 x 86 deg and
a 36% border against fx 514, 102 x 56 deg and no border, which is a third more apparent robot size.
The shipped `Rectifier` uses 1.0; the toggle is there to see what 0.0 would buy before changing it.

The three views answer different questions. `pinhole` is what the batch pipeline renders today, at
the rectified matrix, and it is the one fast enough to fly in (about 15 fps at 640x360). `distorted`
is what the sensor sees, through the lens distortion in the calibration. `rectified` is that
distorted frame put back through the same `initUndistortRectifyMap` the C++ `Rectifier` builds on
the robot, so it shows what perception receives, black border included. Both distorted views render
about 4 fps because the distortion pass renders an enlarged frame and maps it down; pass
`--preview-percentage 25` to put every view above 11 fps at half the preview resolution.

The panel on the right reads out the live `CageMount`, flags each field red when it falls outside
the ranges in `config.toml`, says which pane the one-way glass will hide (a mount flown past a wall
plane loses that pane, because BlenderProc's segmentation stops at the polycarbonate), and projects
a robot-sized box at the mat center and at all eight compass directions. Those pixel widths are
what decides the `imgsz` question, so they are worth watching while choosing a mount rather than
discovering after a 40,000-frame render.

Fly to a pose worth using, press `M`, repeat for the spread you want, then press `Enter`. Every
file written shows up in the Marks and outputs panel as a download link, so a full-quality render or
a saved pose comes straight out of the page. Under `--out` you get:

| File | For |
| --- | --- |
| `mount_ranges.toml` | The `[cages.mount]` block covering every mark, to paste under the matching `[[cages]]` entry in `config.toml`. This is what the batch render consumes. |
| `<name>.toml` | One pose in `CageCalibration` form, for `render_cage_view.py --pose` |
| `camera_rect.json`, `<name>_camera.toml` | The rectified K and the calibration behind it, so the pose re-renders exactly |
| `<name>_render_command.txt` | The `render_cage_view.py` command line that reproduces the full artifact set |
| `<name>_<timestamp>_<view>_alpha<a>.png` | Whatever `F` rendered, at full resolution and the spec's sample count. Timestamped, so repeated renders accumulate rather than overwrite |

#### Running a detector over the renders

The render server cannot do this itself: it is Blender's embedded Python, which ships no TensorRT
or CUDA bindings, and adding them would put gigabytes into an image whose job is rendering. Run the
engine from the project venv instead, against the full-resolution PNGs the tool writes.

```bash
source scripts/activate_python.sh
python - <<'EOF'
from pathlib import Path

import cv2
from auto_battlebot.perception.trt_yolo import TrtYoloModel

# num_classes must match the engine. Getting it wrong misparses the output tensor into zero
# keypoints and near-zero recall, which looks exactly like a broken engine.
model = TrtYoloModel(
    "data/models/yolo26s_nhrl_robots_bbox_2class_rect384x640_2026-09-05_x86_64_sm89.engine",
    conf_threshold=0.25,
    num_classes=2,
)
# Full renders are timestamped, so take the most recent rectified one.
renders = Path("training/data/cage_pose").glob("*_rectified_*.png")
frame = cv2.imread(str(max(renders, key=lambda path: path.stat().st_mtime)))
for box, conf, class_id, keypoints in model.infer(frame):
    x1, y1, x2, y2 = box
    print(f"class {class_id} conf {conf:.2f} {x2 - x1:.0f}x{y2 - y1:.0f} px")
EOF
```

`TrtYoloModel` is the same decode path `training/model_eval/score.py` and the C++ pipeline use, so
the boxes are directly comparable to deployed numbers. Render in the `rectified` view for this: it
is the frame the perception stack actually receives, black border included.

To score renders against ground truth rather than eyeball them, they need labels; see
`training/model_eval/README.md`. For a sequence rather than a still,
`training/yolo/test_tensorrt_video.py` draws boxes onto a video.

## File Overview

| File | Runs via | Purpose |
| --- | --- | --- |
| `config.toml` | -- | Pipeline parameters |
| `prepare_robot_model.py` | `blenderproc run` | Inspect GLTF colors, validate and preview PBR mapping |
| `download_polyhaven_hdris.py` | `python` | Download random HDRIs from Poly Haven |
| `download_objaverse.py` | `python` | Download distractor models from Objaverse |
| `download_ambientcg.py` | `python` | Download PBR textures from ambientCG |
| `render_scenes.py` | `blenderproc run` | Main rendering pipeline (entry point for `synthgen/`) |
| `synthgen/` | imported | Rendering pipeline implementation (pure + Blender-side modules) |
| `tests/` | `pytest` | Unit tests for the pure `synthgen` modules (no Blender needed) |
| `coco_to_yolo.py` | `python` | Convert COCO-style labels to YOLO format |
| `cage/*.toml` | -- | NHRL cage scene specs (geometry, materials, lights, exposure) |
| `render_cage_view.py` | `blenderproc run` | Render the empty cage from fitted poses, for grading against footage |
| `render_cage_samples.py` | `blenderproc run` | Cage-only labelled sample set from the fitted poses |
| `pose_camera_server.py` | `blenderproc run` | Fly the cage camera with WASD in a browser, and save the mount ranges a batch render draws from |
| `web/pose_camera.html` | -- | The page `pose_camera_server.py` serves; edit it and reload, no rebuild needed |
| `build_cage_floor_texture.py` | `venv/bin/python` | Build the mat albedo the cage spec loads, from rectified targets |

## Directory Structure

After running the pipeline:

```text
data/
├── models/
│   └── robot.glb
├── distractor_models/
│   └── objaverse/
│       └── manifest.json
├── hdris/
├── cc_textures/
└── synthetic/
    ├── images/
    └── labels/
```

## Configuration Reference

See `config.toml` for all options. Key sections:

- `[[robots]]` model path, class ID, keypoint positions, color-to-material mapping, and selection weight
- `[materials.*]` PBR properties and texture sources
- `[distractors]` source directories, count range, scale range
- `[environment]` HDRI and texture paths
- `[camera]` distance/height/noise parameters for the HDRI-arena half
- `[[cages]]` one real arena per entry: its share of the run, spec, camera calibration, and `[cages.mount]` sampling ranges
- `[randomization]` material and lighting jitter
