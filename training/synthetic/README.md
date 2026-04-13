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
  blenderproc run training/synthetic/prepare_robot_model.py -- training/synthetic/config.toml --inspect
```

You can run synthetic tools from this container, including:

- `python training/synthetic/download_polyhaven_hdris.py ...`
- `python training/synthetic/download_ambientcg.py ...`
- `python training/synthetic/download_objaverse.py ...`
- `blenderproc run training/synthetic/prepare_robot_model.py -- training/synthetic/config.toml --inspect`
- `blenderproc run training/synthetic/render_scenes.py -- training/synthetic/config.toml`

### Host setup (optional/manual)

Set up an environment:

```bash
cd training/synthetic
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -e .
```

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

### 6. Generate synthetic images

```bash
blenderproc run render_scenes.py -- config.toml
```

Options:

```text
--num-images 5000       Override image count from config
--render-samples 128    Path-tracing samples per pixel
--start-index 10000     Resume from a specific frame index
```

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
| `render_scenes.py` | `blenderproc run` | Main rendering pipeline |
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
