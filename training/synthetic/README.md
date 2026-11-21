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

Cycles renders on every GPU the container can see. On a shared box, pick the GPUs with
`CUDA_VISIBLE_DEVICES` (host numbering); the wrapper turns it into the docker device list,
since docker does not forward the variable itself:

```bash
CUDA_VISIBLE_DEVICES=0 training/synthetic/docker/run_synthetic.sh --require-gpu auto-battlebot-synthetic ...
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

To eyeball one setting at a time, `--venue` pins a run to a `[[cages]]` name or to `arena`
(the HDRI half, which needs HDRIs in `training/data/hdris`) and `--damage off|all` overrides
the battle-damage split. Ten frames of each, two per scene, into
`training/data/synthetic/sample/<setting>/`:

```bash
r="training/synthetic/docker/run_synthetic.sh --require-gpu auto-battlebot-synthetic blenderproc run render_scenes.py -- config.toml --num-images 10 --images-per-scene 2"
$r --venue nhrl_cage   --damage off --seed 0 --out ../data/synthetic/sample/nhrl_cage
$r --venue massd_arena --damage off --seed 1 --out ../data/synthetic/sample/massd_cage
$r --venue arena       --damage off --seed 2 --out ../data/synthetic/sample/hdri
$r --venue nhrl_cage   --damage all --seed 3 --out ../data/synthetic/sample/nhrl_damage
```

A run into a directory that already holds frames resumes after them, so clear the
directory first for a fresh set.

Mrs Buff loses whole assemblies rather than a scatter of hidden parts. Her
`[[robots.damage_parts]]` in `config.toml` name six a batch can choose: `weapon_disk`, `wheels`,
`wheel_side_guards`, `weapon_module` (takes the disk with it), `top_plate` and `bottom_plate`
(each takes its decal, `top_sticker` or `bottom_sticker`, which are `selectable = false`). The
GLB groups faces by colour, so each part is a set of model-frame boxes, and the faces inside are
split into their own objects once at load. A `subset` part loses 1..n of its pieces, drawn by
`count_weights`: one wheel or one side guard is the likely loss. A piece is one box under
`boxes`, or several under `pieces` (a guard is a side wall plus a front arm). A damaged instance
loses `[damage].part_count` of the parts in `[damage].removable_parts` (empty allows all), and
`--damage-parts` pins that pre-selection for one batch. The manifest records which went, as
`"mechanism": "named"` with a `parts` map of piece indices. One set per part:

```bash
for part in weapon_disk wheels wheel_side_guards weapon_module top_plate bottom_plate; do
  $r --venue nhrl_cage --damage all --damage-parts $part --out ../data/synthetic/sample/mrs_buff_$part
done
```

Notes from code comments:

- `training/synthetic/docker/run_synthetic.sh` (`above set +e`):
  Probe GPU runtime support first to preserve interactive behavior and signals.
- `training/synthetic/docker/run_synthetic.sh` (`above docker_id_args=(-e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)")`):
  Run as the host user so files created in the mounted repo are not owned by root. The entrypoint reads these and drops privileges.

### Camera views

Cage scenes write one of three views, set per cage with `view` in `[[cages]]` or for every cage
with `--view`. `synthgen/lens.py` owns all three, and `pose_camera_server.py` shows its `V` views
through the same module, so a frame flown in the preview renders the same way in the batch.

| `view` | Frame written | Render cost |
| --- | --- | --- |
| `pinhole` (default) | Rendered at the rectified matrix, no distortion and no border | 1x |
| `distorted` | The raw sensor frame, through the calibration's OpenCV distortion model | 4x pixels (2560x1442 for 1280x720) |
| `rectified` | The sensor frame through the C++ `Rectifier`'s maps at `rectify_alpha`, black border included | 4x pixels, as `distorted` |

The warped views render a pinhole frame wide enough to hold every sensor ray, at the sensor's own
focal length, and remap it into the view. Labels follow: segmentation, instance and depth maps
warp nearest-neighbour, so boxes come from the warped silhouettes, and keypoints are mapped through
the same model, with visibility 0 for any the view cannot see (outside the sensor, or in the
rectified border). The HDRI arena half has no lens model and stays pinhole, so pin a view
experiment to a cage:

```bash
training/synthetic/docker/run_synthetic.sh --require-gpu auto-battlebot-synthetic \
  blenderproc run render_scenes.py -- config_cage_nhrl.toml --view distorted --out ../data/synthetic/nhrl_distorted
```

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

Notes from code comments:

- `training/synthetic/nhrl_audit.py`:

  ```text
  VRAM audit CSV helpers for distractor models (trimesh-based).

  Mirrors the ``distractor_gpu_audit.csv`` schema written by
  ``download_objaverse.py`` so ``render_scenes.py``'s pre-import VRAM budget
  check works for generated NHRL robots.  Kept separate from
  ``download_objaverse.py`` so the NHRL pipeline runs in the root venv without an
  ``objaverse`` dependency.
  ```

- `training/synthetic/nhrl_common.py`, `compute_bottom_slice_keypoints` (`above xz = vertices[:, [0, 2]]`):
  Axis comes from the footprint PCA; the centerline is based on the full mesh's bounding-box center so it stays in the model's middle at any orientation, and the extent reaches the true silhouette ends.
- `training/synthetic/nhrl_common.py`, `compute_bottom_slice_keypoints`:

  ```text
  Derive centered front/back keypoints for a Y-up mesh.

  The forward axis and lateral center come from the ground footprint's
  principal axis (see `footprint_axes`); the front/back extent is the full
  mesh's silhouette projected onto that axis, so the points reach the robot's
  true ends rather than the wheel-contact patch.  Returns the centerline
  ``(front, back)`` in the GLTF-native frame.  Front/back orientation is
  arbitrary for an unknown robot and is resolved by the human review pass.

  Returns ``None`` for a degenerate mesh (empty, flat, or no horizontal
  extent).
  ```

- `training/synthetic/nhrl_common.py`, `select_bots`:
  Filter, rank, and truncate the BrettZone bot list.
  Drops bracket/bye placeholders and bots below ``min_fights``, sorts by total fights descending (name as a deterministic tie-break), and returns up to ``ceil(limit * (1 + oversample))`` entries so downstream thumbnail 404s still leave ``limit`` usable bots.

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
`docs/experiments/perception/cage_scene_render_match_2026-09-11.md` and
`massd_arena_scene_2026-09-11.md` for how each was fitted to footage, and `training/data/environments/nhrl_3lb_cage/` for its assets.

A third entry, `meatball_basement`, is the drive-test box in the basement: an open plywood box
in a corner of stone walls, fitted from our own ZED (`training/synthetic/scenes/basement_scene/README.md`). It
is off in the default mix; `config_cage_meatball.toml` renders a run entirely in it. Its spec
uses the three scene features the basement added: `[[walls]]` (photo-textured vertical
planes), `[[blocks]]` (plain boxes such as legs) and `world.hdri` (an equirectangular panorama
as the world, in place of the flat background and backdrop cylinder).

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
  --camera-calibration ../../config/cameras/synthetic_cage_lens_118deg.toml \
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
a narrower field. On the estimated 118 degree lens calibration at 1280x720 that is fx 384, 118 x 86 deg and
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

Notes from code comments:

- `training/synthetic/synthgen/distractors.py`, `place_scene_distractors` (`above for distractor in active:`):
  Distractor robots are all treated as the 3 lb weight class; scale off this fixed base rather than whichever robot happens to be in the scene.
- `training/synthetic/synthgen/distractors.py`, `place_distractor` (`above roll = math.pi if random.random() < 0.5 else 0.0`):
  Grounded robot pose: flat on the field, random yaw, 50% inverted. GLB import is Z-up, so identity roll/pitch sits the model upright.
- `training/synthetic/synthgen/distractors.py`, `place_distractor`:

  ```text
  Place a distractor at a random position within the arena.

  ``scale_range`` is interpreted as multiples of ``base_size`` (the fixed 3 lb
  weight-class reference dimension), so [0.5, 2.0] means the distractor will be
  between half and twice the 3 lb base size regardless of its native
  dimensions.  The parent empty is used for transform so sub-parts keep their
  relative arrangement.

  With probability ``air_probability`` the object floats above the ground
  with a fully random tumbling orientation.  Otherwise it sits on the
  ground: generic objects keep a fully random orientation, while robot-like
  (CAD robot) distractors sit flat with a random yaw and a 50% chance of
  being inverted, mirroring the target robots' ground poses.
  ```


## Notes moved from code comments (2026-11-15)

- `training/synthetic/build_cage_floor_texture.py`, `combine_clips` (`above target_mean = np.mean(`):
  Restore the event's mean colour so the albedo carries the real mat tint, and fill what no clip saw (the near strip the camera cuts off) with that mean.

- `training/synthetic/build_cage_floor_texture.py`:

  ```text
  Orthographic mat albedo from the rectified cage-high targets.

  Each target is warped onto the mat plane through its fitted pose (the inverse of the
  homography K [r1 r2 t]) into a square texture over the mat, then the clips of one event are
  combined by a per-pixel median after normalising each clip to its own mat-region mean, so
  exposure drift between fights does not vote. The texture is in the hfield frame: pixel x
  runs along hfield +x, pixel y along hfield +y, origin at (-half, -half).

  Outputs `<out>/cage2_<event>_albedo.png` with a `.json` sidecar (px per metre, extent,
  clips used) and a downscaled `_preview.png`. The default `--out` is the cage environment
  directory the Blender spec reads the albedo from.

  Usage:
      venv/bin/python training/synthetic/build_cage_floor_texture.py runs/cage_scene --px-per-m 1024
      venv/bin/python training/synthetic/build_cage_floor_texture.py runs/cage_scene         --source target_4k.png
  ```

- `training/synthetic/compute_nhrl_keypoints.py` (`above def _clear_scene() -> None:`):
  nhrl_common is imported by top-level name. blenderproc runs this script under its own Python (not the venv), so it must be launched with PYTHONPATH including training/synthetic (set by the docker wrapper; see the docstring).
- `training/synthetic/compute_nhrl_keypoints.py` (`above import argparse`):

  ```text
  Compute front/back keypoints and render a top-down preview for robot meshes.

  Step 3 of the NHRL robot distractor pipeline.  Runs under BlenderProc so it can
  render each robot with its real materials (the "sheen" a flat vertex-color bake
  loses).  For every .glb it:
    - imports the mesh with bpy and reads world-space vertices,
    - derives centered front/back keypoints from the footprint (nhrl_common),
    - renders a full-material ORTHOGRAPHIC top-down PNG, and
    - writes a <stem>.json sidecar with the keypoints plus a "topdown" block
      (image path, model->pixel affine, footprint hull, y_ground, default axis)
      so review_nhrl_keypoints.py can overlay a draggable centerline that lines
      up exactly with the render, without loading the mesh itself.

  Usage (under BlenderProc; run from training/synthetic/ so nhrl_common is
  importable, the docker wrapper run_synthetic.sh sets PYTHONPATH; for a manual
  run prefix the command with PYTHONPATH="$PWD"):
      blenderproc run compute_nhrl_keypoints.py -- ../data/distractor_models/robots
      blenderproc run compute_nhrl_keypoints.py -- ../data/distractor_models/robots --overwrite
  ```

- `training/synthetic/docker/entrypoint.sh` (`above if [ "$(id -u)" = "0" ] && [ -n "${HOST_UID:-}" ] && [ -n "${HOST_GID:-}" ] && [ ...`):
  Step down to the host's UID/GID so files written into the mounted repo are owned by the host user instead of root. run_synthetic.sh passes HOST_UID/HOST_GID; when they are absent (image run directly) we keep the previous root behavior.
- `training/synthetic/docker/run_synthetic.sh` (`above docker_env_args=(-e PYTHONPATH=/workspace/training/synthetic)`):
  blenderproc runs Blender's own Python (not the project venv), so the synthetic scripts' sibling modules (synthgen, nhrl_common) are made importable via PYTHONPATH here instead of an in-script sys.path hack. Matches the container working directory (-w below).
- `training/synthetic/domain_render_report.py`:

  ```text
  Gate report for a merged domain-mix render: per-view counts, classes, keypoints, drops, damage.

  Step 3 of docs/experiments/perception/synthetic_domain_mix_plan_2026-09-12.md lists the
  checks a render passes before it goes to training. `validate_yolo_integrity.py --strict` covers the
  dataset's structure; this covers what that cannot see, all split by view because the distorted and
  rectified views cut objects at the frame edge and at the border where pinhole does not:

  - frames per view and venue from `manifest.jsonl`, against an even three-way split
  - instances per class, so our robots are not rare
  - keypoints flagged 0 (not visible), against the randomized pool's rate
  - frames dropped by the visibility gate, from each run's `render.log` in the parts directory
  - fully clean frames, the pool the damage-off arm draws from

      venv/bin/python training/synthetic/domain_render_report.py \
          training/data/synth_cage_nhrl_2026-09-13 \
          --baseline training/data/all_robot_keypoints/train

  Writes `gate_report.json` beside the dataset's manifest and exits 1 when a gate fails.
  ```

- `training/synthetic/download_nhrl_bots.py`, `fetch_all_bots` (`above time.sleep(0.3)`):
  Fair-use: stay well under the 5s real-time poll ceiling for archive data.
- `training/synthetic/download_nhrl_bots.py`:

  ```text
  Fetch the NHRL bot roster from BrettZone and cache thumbnails for meshing.

  Step 1 of the NHRL robot distractor pipeline.  Pages the BrettZone bots API,
  caches the roster locally, ranks bots by activity, downloads their thumbnails,
  and records which bots lack a usable thumbnail in the shared state file.  No
  Meshy credits are spent here.

  Usage:
      python download_nhrl_bots.py ../data/distractor_models/robots --limit 300
      python download_nhrl_bots.py ../data/distractor_models/robots --min-fights 5 --refresh-bot-list
  ```

- `training/synthetic/generate_nhrl_meshes.py`:

  ```text
  Generate 3D robot meshes from NHRL thumbnails via the Meshy image-to-3D API.

  Step 2 of the NHRL robot distractor pipeline.  Reads the state file written by
  ``download_nhrl_bots.py``, submits Meshy image-to-3D tasks for bots that have a
  thumbnail, polls them, downloads the resulting GLBs, and appends VRAM audit
  rows.  Fully resumable: submitted tasks are never resubmitted, and progress is
  persisted after every state transition.

  Requires the ``MESHY_API_KEY`` environment variable.

  Usage:
      MESHY_API_KEY=... python generate_nhrl_meshes.py ../data/distractor_models/robots --limit 10
      python generate_nhrl_meshes.py ../data/distractor_models/robots --dry-run
  ```

- `training/synthetic/nhrl_common.py`, `footprint_axes`:

  ```text
  Derive the horizontal principal axes of a mesh's ground footprint.

  *vertices* is an ``(N, 3)`` array in the GLTF-native frame (Y is up).  The
  bottom ``slice_fraction`` of the height is taken to isolate the footprint,
  then grown until it holds enough points and spans at least half the model's
  larger horizontal extent (so a sparse lowest-point cluster does not skew the
  axes).  2D PCA over the slice's ``(X, Z)`` coordinates yields the forward
  axis ``u1`` (largest variance, rotation-invariant) and the perpendicular
  lateral axis ``u2``.  ``u1`` is oriented deterministically so repeated runs
  are reproducible; which physical end is "front" is still arbitrary and is
  resolved by the human review pass.

  Returns ``(centroid_xz, u1, u2, slice_pts, y_ground)`` where ``centroid_xz``
  and the unit axes are 2D ``(X, Z)`` vectors, ``slice_pts`` is the ``(M, 3)``
  footprint slice, and ``y_ground`` is the slice's lowest Y.  Returns ``None``
  for a degenerate mesh (empty, flat, or no horizontal extent).
  ```

- `training/synthetic/pose_camera_server.py` (`DEFAULT_ROBOT_SIZE = (0.21, 0.18, 0.09)`):
  MRS BUFF MK3 is a 3 lb bot; its two keypoints sit 0.1528 m apart front to back, and the shell overhangs both. These are the box the pixel-width overlay projects, not a measured hull. length, width, height in meters
- `training/synthetic/preview_nhrl_keypoints.py` (`above import argparse`):

  ```text
  Render 6-view keypoint previews for NHRL robot distractor models.

  Step 4 of the NHRL robot distractor pipeline.  For each model it imports the
  GLB, reads the <stem>.json sidecar, drops a red marker at the front keypoint
  and a blue marker at the back keypoint, and renders a 3x2 grid of cardinal
  views (mirroring prepare_robot_model.py previews) so the bottom-slice
  heuristic can be eyeballed.

  Usage:
      blenderproc run preview_nhrl_keypoints.py -- ../data/distractor_models/robots
      blenderproc run preview_nhrl_keypoints.py -- ../data/distractor_models/robots --limit 10
  ```

- `training/synthetic/register_nhrl_thumbnail.py`:

  ```text
  Register a manually-downloaded NHRL robot thumbnail into the generation state.

  Use this when you drop a thumbnail PNG into ``<output_dir>/thumbnails/`` yourself
  (bypassing ``download_nhrl_bots.py``, e.g. for a specific opponent) and want a proper
  state entry so ``generate_nhrl_meshes.py`` can pick it up, without hand-editing
  ``.nhrl_generation_state.json``.

  The clean_name is derived from the display name the same way BrettZone does it:
  lowercase, then strip every non-alphanumeric character. ``Iron Warrior`` -> ``ironwarrior``.
  The thumbnail is expected at ``<output_dir>/thumbnails/<clean_name>.png`` unless
  ``--thumbnail`` overrides it.

  Usage:
    venv/bin/python training/synthetic/register_nhrl_thumbnail.py       training/data/distractor_models/robots "Iron Warrior"

    # several at once
    venv/bin/python training/synthetic/register_nhrl_thumbnail.py       training/data/distractor_models/robots "Sphinx" "Wreck Creation" "Iron Warrior"

    # override the derived clean_name / thumbnail file (single robot only)
    venv/bin/python training/synthetic/register_nhrl_thumbnail.py       training/data/distractor_models/robots "M.C.B." --clean-name mcb --thumbnail mcb.png

  Then generate just those with:
    venv/bin/python training/synthetic/generate_nhrl_meshes.py       training/data/distractor_models/robots --only ironwarrior sphinx wreckcreation --limit 3
  ```

- `training/synthetic/render_shards.py`:

  ```text
  Render one venue's domain-mix dataset across several GPUs, one view per run, then merge it.

  Step 3 of docs/experiments/perception/synthetic_domain_mix_plan_2026-09-12.md. The GPU
  queue runs one job at a time, so this is that one job: one worker per GPU renders its shard as one
  ``render_scenes.py`` container per view, back to back, and once every run has finished the runs are
  hardlinked into one flat dataset.

      venv/bin/python training/gpu_queue.py submit --name render_cage_nhrl --by <agent> -d 0 1 2 -- \
        venv/bin/python training/synthetic/render_shards.py config_cage_nhrl.toml \
          --out ../data/synth_cage_nhrl_2026-09-13 --total 20000 --gpus 0 1 2 --seed-base 0

  ``render_scenes.py`` runs inside the container, whose working directory is training/synthetic, so
  the config and ``--out`` are relative to that directory. Anything after a bare ``--`` goes to every
  ``render_scenes.py`` run, for smoke tests (``-- --images-per-scene 2 --damage all``).

  Runs write to ``<out>_parts/shard<i>_<view>/``. A rerun resumes: a finished run is skipped, a run a
  crash cut short continues one past its last frame on disk under a fresh seed, and the merge happens
  only once every run is complete. The parts stay after the merge; the dataset's files are hardlinks
  to them, so deleting the parts costs nothing but the directory entries.
  ```

- `training/synthetic/render_sprites.py` (`above SpriteSpec(`):
  The house bot GLB carries baked PBR textures already, so the colour mapping finds nothing to replace and leaves them alone. Its painted face points +X.
- `training/synthetic/render_sprites.py` (`above SpriteSpec(`):
  Mrs Buff MK2 is not in the training set, but it shares the MK3's OnShape palette, so the MK3's colour mapping textures it correctly. Its CAD origin sits at the back rather than the front, hence the explicit front direction.
- `training/synthetic/render_sprites.py`, `SpriteSpec`:

  ```text
  One sprite the viewer asks for by name.

  Attributes:
      name: Manifest key and PNG stem, and what a sim config's ``sprite`` key names. Sprites
          are named after the robot, not its role: which one is ours is a per-run choice.
      robot: ``[[robots]]`` entry to render, by its config ``name``.
      model_path: Override the config's model, keeping its materials and orientation. Used for
          robots the training set does not need but the viewer draws.
      front_model: Front direction in the model's own frame, for an override model that has
          no keypoints of its own. Same frame the config's ``[robots.keypoints]`` use, so it goes
          through the same Y-up to Z-up conversion.
  ```

- `training/synthetic/review_nhrl_keypoints.py`:

  ```text
  Interactively confirm front/back keypoint direction for NHRL robot models.

  Step 3.5 of the NHRL robot distractor pipeline, run on the HOST after
  ``compute_nhrl_keypoints.py`` (which renders each robot's full-material
  orthographic top-down and records the model->pixel affine in the sidecar).
  Which end is *front* (thrust/weapon direction), and even which axis, cannot be
  recovered from a static mesh, so this shows the render and lets an operator
  **draw the centerline**: drag across the robot to set the forward axis at any
  angle (front = the end you drag toward).  Front/back snap to the footprint's
  extremes along that direction, centered on the body.  Confirmed sidecars are
  marked ``reviewed: true`` so recompute never clobbers them.  Bad meshes can be
  marked ``rejected: true``; distractor discovery skips them so they never render.

  The review starts on the first model that is neither reviewed nor rejected, so
  re-running the tool resumes where the last pass left off.

  This tool loads only the sidecar PNG + JSON (no mesh, no trimesh, no rendering),
  so it is light and host-only, but it needs a GUI display.  Run in a root project
  venv (needs ``opencv-python`` + ``numpy``):

      venv/bin/python training/synthetic/review_nhrl_keypoints.py         training/data/distractor_models/robots

  Controls:
      drag       draw the centerline direction (front = drag end)
      f          flip front/back
      a / space  accept (mark reviewed) and go to next
      r          reject the model (mark rejected) and go to next
      n / .      next        p / ,      previous
      u          reset to geometry default (clears reviewed/rejected)
      q / esc    quit
  ```

- `training/synthetic/review_nhrl_thumbnails.py`:

  ```text
  Review NHRL bot thumbnails and reject the ones unfit for Meshy image-to-3D.

  Step 1.5 of the NHRL robot distractor pipeline, run on the HOST after
  ``download_nhrl_bots.py`` and BEFORE ``generate_nhrl_meshes.py`` so no Meshy
  credits are spent on bad source images (busy backgrounds, multiple bots, tiny/
  side-on shots, logos).  Decisions are recorded in the shared state file
  (``.nhrl_generation_state.json``) as a ``rejected`` flag on each bot entry.  A
  rejected bot is skipped by both ``generate_nhrl_meshes.py`` (never meshed) and
  ``download_nhrl_bots.py`` (never re-downloaded, even after its thumbnail is
  deleted), so the veto is sticky across re-runs.

  Two ways to use it:

  1. Interactive review (needs a GUI display + ``opencv-python``):

         ../../venv/bin/python review_nhrl_thumbnails.py ../data/distractor_models/robots

     Shows each downloaded thumbnail; ``a``/space accepts, ``r`` rejects (and moves
     the PNG to ``rejected_thumbnails/``), ``n``/``p`` navigate, ``u`` clears the
     decision, ``q`` quits.  Resumes on the first un-reviewed thumbnail.

  2. Bulk reject from a list (no GUI), for names you have already triaged
     elsewhere.  Tokens may be bare clean names, ``<name>.png``, or ``trash:///``
     URIs; pass them as arguments, from a file, or on stdin:

         ../../venv/bin/python review_nhrl_thumbnails.py ../data/distractor_models/robots            --reject yeschef mako apex
         ../../venv/bin/python review_nhrl_thumbnails.py ../data/distractor_models/robots            --reject-file rejected.txt
         cat rejected.txt | review_nhrl_thumbnails.py ../data/distractor_models/robots --reject -

  Controls (interactive):
      a / space  accept and go to next
      r          reject (move PNG out of the pool) and go to next
      n / .      next        p / ,      previous
      u          clear the decision (un-review, un-reject)
      q / esc    quit
  ```

- `training/synthetic/synthgen/cage.py`, `set_led_emission`:
  Scale the house bot LED emission by the inverse exposure gain.
  `led_emission` is the LED brightness in output units: 1.0 puts the texel colour at full scale after the view exposure, so pink stays pink instead of clipping to white.
- `training/synthetic/synthgen/cage_scene.py`:

  ```text
  The NHRL cage as one half of the scene mix (requires Blender).

  `synthgen.cage` builds the cage that `render_cage_view.py` grades against NHRL footage, from
  the fitted pose of their fixed `Cage-2-Overhead-High` phone. This module wraps the same build
  in what the generic pipeline needs to render cage scenes and HDRI-arena scenes in one run:

  - every cage object in one Blender collection, so the whole cage hides and shows at once;
  - a snapshot of the render settings the cage look depends on (Standard view transform with the
    solved exposure gain, glass bounces, its own sample count and denoiser, the rectified camera
    intrinsics), restored exactly when a generic scene renders;
  - camera poses sampled from `synthgen.cage_mount` instead of the fitted event poses, so the
    mount varies the way ours would.

  The cage geometry, materials, lights and exposure are the spec's, unchanged: the numbers in
  `cage/cage2_overhead_high.toml` were fitted against footage and the match is only as good as
  they are.
  ```

- `training/synthetic/synthgen/cage_spec.py`:
  Cage geometry spec: the TOML schema, dotted overrides, and the primitives it expands to.
  Pure module (no Blender). `cage.py` turns the boxes and cylinders returned here into mesh objects; tests check the geometry without a renderer. All lengths are metres in the W frame: mat centre at the origin, z up, +y away from the fixed camera, +x to its right. The mat top surface is z = 0.
- `training/synthetic/synthgen/freefly.py`, `clamp_mount_to_ranges`:
  The nearest mount the batch sampler could actually draw from *ranges*.
  Every scalar is clamped into its range. The wall is kept if `ranges.walls` allows it; otherwise the caller should re-read the mount with `walls=ranges.walls` first, since moving a camera to another wall renames `along_m` and `inset_m` rather than clamping them.
- `training/synthetic/synthgen/freefly.py`, `mount_from_cam2world`:
  The `CageMount` that describes *cam2world* exactly, picking among *walls*.
  Every pose is some mount, so nothing is lost here. The wall choice only decides which numbers say so: pick a different wall and `along_m` and `inset_m` change to match.
- `training/synthetic/synthgen/freefly.py` (`above PITCH_LIMIT_DEG = 89.9`):
  Straight up or straight down leaves yaw and roll expressing the same rotation, so the client never gets to send either.
- `training/synthetic/synthgen/gating.py`, `RobotPixelStats`:

  ```text
  Per-robot pixel measurements for one frame, computed from the segmaps.

  Attributes:
      instance_id: The robot's instance id in the instance segmap.
      name: Robot name for log messages.
      tracked: True when per-instance stats are available (instance segmap
          was rendered). Without it, robots share one category id and
          per-robot prominence cannot be established.
      visible_px: Robot pixels in the rendered (occluded) segmap.
      bbox_w_px: Bounding-box width in pixels (0 when the robot has no bbox).
      bbox_h_px: Bounding-box height in pixels.
      unobstructed_px: Robot pixels in the distractor-free clean pass, or
          None when the clean pass was unavailable (or skipped).
  ```

- `training/synthetic/synthgen/gating.py`:

  ```text
  Frame keep/drop policy (no Blender imports).

  Every decision about whether a robot gets annotated and whether a frame gets
  written lives here, so the policy is unit-testable and every negative outcome
  carries a reason.

  Keypoint-mode policy: a robot that fails its annotation gate is left unlabeled
  and the frame is kept, UNLESS the robot is prominently visible, an unlabeled
  prominent robot would be a false negative in the training data, so those frames
  are dropped instead. This replaces the old behavior where any gate failure
  discarded the whole frame (even for a few-pixel sliver at the frame edge),
  which was the main source of unexplained frame drops.
  ```

- `training/synthetic/synthgen/keypoints.py`, `build_house_bot_annotation`:

  ```text
  Annotate the cage's house bot, boxed from the pixels of its own category.

  There is exactly one house bot in a scene and it never moves, so it is boxed from the
  category segmap rather than an instance id. A robot standing in front of it shrinks the
  box to what is still visible, and hides it entirely once nothing is left.

  Returns:
      The annotation, or None when the house bot is out of frame or too small to label.
  ```

- `training/synthetic/synthgen/lens.py`, `_warp` (`above out = cv2.remap(`):
  Constant zero, not replication: the black border is what a rectified frame looks like.
- `training/synthetic/synthgen/lens.py`, `render_normalized_to_output`:
  A Blender-normalized render point (top-left origin) in the output frame, or None if unseen.
  This is the form `world_to_camera_view` gives, with y already flipped to the image convention.
- `training/synthetic/synthgen/lens.py`, `LensView`:

  ```text
  One view at one output size: the camera Blender renders with, and how to warp its frames.

  Attributes:
      view: One of `synthgen.constants.VIEWS`.
      alpha: The rectification alpha. Sets `rect_k`, and the border of the rectified view.
      size: (width, height) of the frames this view writes.
      render_k: Intrinsics Blender renders with, at `render_size`.
      render_size: (width, height) Blender renders at; wider than `size` for warped views.
      sensor_k: The calibration's matrix scaled to `size`.
      distortion: OpenCV (k1, k2, p1, p2, k3).
      rect_k: The rectified matrix for `alpha` at `size`.
      max_radius: Largest undistorted normalized radius any sensor pixel sees. A render point
          beyond it is outside the sensor, whatever the distortion polynomial folds it back to.
      distortion_map: Output pixel -> render pixel, for the distorted and rectified views.
      rectify_map: Rectified pixel -> sensor pixel, for the rectified view.
      valid: Output pixels that hold scene content, or None when every pixel does.
  ```

- `training/synthetic/synthgen/lens.py` (`above _REMAP_DTYPES = (np.uint8, np.uint16, np.int16, np.float32, np.float64)`):
  cv2.remap takes these dtypes directly; anything else goes through float32, exact for ids < 2**24.
- `training/synthetic/synthgen/lens.py` (`above RENDER_MARGIN_PX = 2`):
  Blank render pixels kept around the sensor's footprint, so bilinear sampling never reads past it.
- `training/synthetic/synthgen/lens.py` (`above EXTENT_SAMPLES = 65`):
  Sensor pixels sampled per axis when sizing the wide render. The undistorted extent of a lens is set by its frame boundary, and the grid covers a lens whose extent peaks inside the frame.
- `training/synthetic/synthgen/lens.py`:

  ```text
  The camera views a render is written in, shared by the batch pipeline and the pose server.

  Pure module (numpy and OpenCV, no Blender). `synthgen.cage_scene` and `synthgen.pipeline` use it to
  write training frames, and `synthgen.preview_render` uses it to show the same frames live, so an
  experiment trains on exactly the warp that was flown in the preview.

  **Views.**

  * `pinhole` renders at the rectified matrix and needs no warp. It is what the batch pipeline wrote
    before views existed.
  * `distorted` is the raw sensor frame. Blender renders a pinhole frame wide enough to hold every
    sensor ray, at the sensor's own focal length so the image centre keeps its detail, and a remap
    pulls the sensor image out of it.
  * `rectified` is that sensor frame put back through `rectify_maps`, the maps the C++ `Rectifier`
    builds, black border included.

  The warp maps come from OpenCV's distortion model rather than BlenderProc's `set_lens_distortion`:
  the calibration is an OpenCV calibration and `Rectifier` is OpenCV, so labels projected through
  this model land where the robot's own undistortion puts them.

  **Labels.** Colour warps bilinear. Segmentation, instance and depth maps warp nearest-neighbour, so
  ids stay ids and boxes drawn from them follow the lens. Keypoints are projected by Blender into
  the wide render and mapped here, through the same model, into the output frame.

  **Pixels.** OpenCV puts pixel centres on integers; Blender's normalized camera view puts the frame
  edges at 0 and 1. So ``u_opencv = x_norm * width - 0.5``, in both directions.
  ```

- `training/synthetic/synthgen/logsetup.py`:
  Logging configuration for the synthgen package.
  Every module gets its logger via ``get_logger(__name__)``; the entry point calls ``configure()`` once. BlenderProc/Blender write their own stdout noise, so the format stays terse and level names only appear for WARNING and above.
- `training/synthetic/synthgen/materials.py`, `get_material_base_color` (`above return None`):
  Non-constant source (image texture, procedural, etc.), the material has intentional visual content that shouldn't be remapped.
- `training/synthetic/synthgen/pipeline.py`, `_extract_frame_annotations` (`above if visible_category_ids != {SEG_FLOOR_CLASS_ID}:`):
  Skip the (comparatively expensive) contour extraction for floor-only frames; decide_seg_frame reports them as SEG_FLOOR_ONLY regardless.
- `training/synthetic/synthgen/pipeline.py`, `_append_manifest_row`:

  ```text
  One JSON line per written frame: which arena, view, mount, and what was damaged.

  This is what makes the damage arms a filter rather than a second render: a damage-off
  arm keeps the frames whose every instance reports ``damage == 0``. The mount is here
  too, so a frame can be traced back to the camera placement that produced it. The view
  is the only record of which lens a frame went through once per-view runs are merged
  into one directory; the HDRI half has no lens model and is always pinhole.
  ```

- `training/synthetic/synthgen/pipeline.py`, `SceneState` (`above damage: tuple[InstanceDamage, ...] = ()`):
  One entry per instance damage was rolled for, damaged or not.
- `training/synthetic/synthgen/preview_render.py`, `__init__` (`clone = original.copy()`):
  shares original.data, so no mesh is duplicated
- `training/synthetic/synthgen/preview_render.py`, `_finish` (`return path.read_bytes()`):
  Blender's own JPEG, already color managed
- `training/synthetic/synthgen/preview_render.py`, `_render_to` (`with _suppress_stdout():`):
  Blender prints a Fra:/Mem: line per render call
- `training/synthetic/synthgen/preview_render.py`, `_build_setup` (`above lens = build_lens_view(self._calibration, size, view, alpha)`):
  Warped views build the lens at their own size, because its maps index the wide render in pixels.
- `training/synthetic/synthgen/preview_render.py`, `_apply` (`above restore_render_state(self._cage_state)`):
  Put back the complete drift-prone set first, so nothing survives from the other mode.
- `training/synthetic/synthgen/preview_render.py`, `_apply` (`above key = (view, preview, 1.0 if view == "distorted" else alpha)`):
  The distorted view is the raw sensor frame, which no rectification alpha touches, so it shares one cache entry instead of paying the lens-distortion solve again per alpha.
- `training/synthetic/synthgen/preview_render.py`, `__init__` (`above self._k_rect = {alpha: rectify_maps(calibration, render_size, alpha)[2] for alpha...`):
  Both rectification alphas up front: the page toggles between them live, and each one is a different rectified matrix, so the pinhole view and the overlay both have to follow.
- `training/synthetic/synthgen/preview_render.py` (`PREVIEW_PATH = Path("/dev/shm/ab_pose_preview.jpg")`):
  tmpfs, so the render never touches a disk. One fixed name: a varying one fills /dev/shm. noqa: S108 - tmpfs is the point
- `training/synthetic/synthgen/preview_server.py`, `serve` (`host: str = "0.0.0.0",`):
  noqa: S104 - inside the container; the host publishes to 127.0.0.1
- `training/synthetic/synthgen/preview_server.py`, `Handler` (`above protocol_version = "HTTP/1.1"`):
  Keep-alive matters at 15 fps: without it every frame pays a fresh TCP connection, and HTTP/1.1 keep-alive only works if every response carries a Content-Length.
- `training/synthetic/synthgen/preview_server.py`, `_pose` (`above return Response.json(`):
  Answering with the readout instead of 204 keeps the page's overlay locked to the pose it just sent. It is pure math on a handler thread, so it still never blocks on a render.
- `training/synthetic/synthgen/preview_server.py`, `overlay_geometry`:
  Mat outline and robot-sized boxes, projected for the page to draw over the frame.
  The robot pixel widths are the number that decides the `imgsz` question, so they belong on screen while a mount is being chosen rather than in a 40,000-frame render afterwards.
- `training/synthetic/synthgen/preview_server.py`, `mat_sample_points`:
  The nine mat points worth measuring a robot at, keyed by compass label.
  Fixed in the world frame, not relative to the camera: a mount is judged by how a robot reads across the whole mat at once, and fixed points keep the readout stable while flying.
- `training/synthetic/synthgen/preview_server.py` (`above NEAR_PLANE_M = 0.02`):
  Nothing real sits within 2 cm of the lens, and clipping there keeps projected pixel coordinates finite and small enough for a canvas to stroke.
- `training/synthetic/synthgen/preview_server.py`, `SceneInfo`:
  Static facts about the built scene, gathered once on the main thread.
  Everything here is fixed for the life of the process, so the handler can serve it without touching Blender.
- `training/synthetic/synthgen/robots.py`, `_parent_face_centres`:
  *obj*'s face centres in its parent's frame, as an ``(n, 3)`` array.
  matrix_local is relative to robot_parent, which is the frame keypoints live in.
- `training/synthetic/synthgen/shards.py`, `_collect_part` (`above images = sorted(`):
  Frames only: the pipeline also writes a `_debug_frame0.jpg` beside them, which has no label or manifest row and is not training data.
- `training/synthetic/tests/test_preview_server.py`:
  The mailbox coalesces, and the router answers without ever touching Blender.
  The render loop runs on the main thread and HTTP handlers run on their own, so the mailbox is the only shared state. Two things have to hold: a pose that arrives mid-render replaces the pending one rather than queueing behind it (otherwise releasing W plays back a backlog of stale frames), and commands do queue, because dropping a mark or a save would lose work.

- `training/synthetic/compute_nhrl_keypoints.py`, `_project` (`above def _project(mx: float, mz: float) -> np.ndarray:`):
  Derive the affine [x, z, 1] -> [px, py]. Ortho top-down makes model (x,z) -> pixel exactly affine and independent of height, so 3 points define it.

- `training/synthetic/download_ambientcg.py` (`above KEEP_SUFFIXES = {`):
  BlenderProc only uses these texture map suffixes (from CCMaterialLoader source).

- `training/synthetic/pose_camera_server.py`, `save_outputs` (`above write_camera_rect(out_dir, renderer.k_rect(current.alpha), renderer.render_size, ...`):
  The rectified K written here is the alpha that was on screen, so a re-render reproduces the framing that was actually being judged.
- `training/synthetic/pose_camera_server.py`, `handle_command` (`above stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")`):
  Timestamped, so repeated renders of the same name accumulate instead of overwriting each other. Same format the training scripts use for session directories.
- `training/synthetic/pose_camera_server.py`, `render_command` (`above return (`):
  Paths are absolute container paths: run_synthetic.sh runs with the working directory set to /workspace/training/synthetic, so a repo-relative script path would not resolve.
- `training/synthetic/pose_camera_server.py`, `RobotToggle`:
  MRS BUFF MK3 on the mat, loaded the first time it is asked for.
  One import, then linked duplicates at every mat sample point, so a single toggle shows how a robot reads at the center and at all eight compass directions at once. The import takes a few seconds, which is why it does not happen at startup: the tool is usable long before anyone wants to check a silhouette.
- `training/synthetic/pose_camera_server.py`, `resolve_under`:
  Relative paths are read against *root*, which for every flag here is `training/synthetic`.
  That is also the container's working directory, so the paths on the command line in this module's docstring mean the same thing whether they are typed for the container or for a checkout on the host.
- `training/synthetic/synthgen/cage_mount.py`, `mount_corners`:
  Horizontal extremes of every mount *ranges* can draw, as (wall, xy) pairs.
  A mount's position is linear in `along_m` and `inset_m`, so the four corners of that rectangle bound every draw on a wall.
- `training/synthetic/synthgen/cage_mount.py` (`above WALLS: tuple[str, ...] = ("near", "far", "left", "right")`):
  The wall a camera is mounted on, named from the fitted camera's point of view: it sits on the near wall, so +y in the W frame runs away from it.

- `training/synthetic/prepare_robot_model.py`, `get_material_base_color` (`above return None`):
  Non-constant source (image texture, procedural, etc.), the material has intentional visual content that shouldn't be remapped.
- `training/synthetic/synthgen/colorspec.py`, `match_material_type` (`above if nearest_match is not None and nearest_dist <= COLOR_MATCH_FALLBACK_MAX_DIST:`):
  CAD exports can introduce close shade variants for the same part color.

- `training/synthetic/render_shards.py`, `record_source`:
  The commit and uncommitted training/synthetic diff this attempt renders from.
  The containers mount the repo live, so a run started hours in reads whatever synthgen holds then. The patch is what makes the render traceable when the tree was not committed first.

- `training/synthetic/synthgen/asset_index.py`, `allocate_keypoint_instance_id`:
  Unique robot_instance_id for a keypoint-annotated CAD distractor.
  Returns 0 in segmentation mode (instance ids are managed there by the segmentation pass, and distractor keypoints are not emitted).
- `training/synthetic/synthgen/cage_scene.py`, `build_cage_stage` (`above mat_category_id = SEG_FLOOR_CLASS_ID if is_segmentation_mode else BACKGROUND_CATE...`):
  In keypoint mode the mat is background and the house bot is its own detector class; in segmentation mode the mat is the floor and the house bot is one more object on it.

- `training/synthetic/synthgen/camera.py`, `setup_scene_cameras` (`above min_dist = cam_cfg.min_distance`):
  Push the camera back far enough that the robots' bounding sphere can never dominate the frame (near-full-frame robots produce confusing annotations).

- `training/synthetic/synthgen/geometry.py`, `min_distance_for_frame_fraction`:
  On the image plane a point at angle theta lands at tan(theta)/tan(fov/2) of the half-frame. Solving tan(theta) = max_frame_fraction * tan(fov/2) for the sphere silhouette (sin(theta) = radius/distance) gives the closed form `distance = radius * sqrt(1 + k^2) / k` with `k = max_frame_fraction * tan(fov/2)`.
