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
--render-samples 128    Path-tracing samples per pixel
--start-index 10000     Resume from a specific frame index
--seed 42               Seed Python/numpy RNGs for reproducible debugging runs
-v / --verbose          Debug logging (per-robot skip detail, asset decisions)
-q / --quiet            Warnings and the run summary only
```

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
- `[camera]` distance/height/noise parameters
- `[randomization]` material and lighting jitter
