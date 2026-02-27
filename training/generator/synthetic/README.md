# Synthetic Data Generator for YOLO Keypoint Training

Generates randomized training images with a target robot (annotated with
bounding box + front/back keypoints) and distractor objects. Outputs
YOLO-format labels compatible with the existing training pipeline.

## Prerequisites

Install the project dependencies (from the repo root):

```bash
./scripts/setup_python.sh
```

This installs `blenderproc`, `objaverse`, and `pygltflib` (x86 only).

For SAM 3D distractor generation, install separately:

```bash
git clone https://github.com/facebookresearch/sam3d.git
cd sam3d && pip install -e .
```

## Quick Start

### 1. Export your robot from OnShape

Export the assembly as **GLTF** (`.glb`). Place it at the path specified in
`config.toml` (default: `training/data/models/robot.glb`).

### 2. Inspect the model's material colors

```bash
blenderproc run prepare_robot_model.py -- config.toml --inspect
```

Inspect GLTF material colors and prepare a textured robot model for rendering.

Usage:
List all material colors in a GLTF file (helps build the color_mapping in config.toml):
blenderproc run prepare_robot_model.py config.toml --inspect

Apply PBR textures and save a .blend file for visual inspection:
blenderproc run prepare_robot_model.py config.toml --save-blend robot_textured.blend

This prints every material name and RGB color in the GLTF. Use the output to
fill in the `[[robot.color_mapping]]` entries in `config.toml`.

### 3. Set the keypoint positions

In `config.toml`, set `robot.keypoints.front` and `robot.keypoints.back` to
the 3D coordinates (meters, relative to the model origin) of your robot's
front and back keypoints. Measure these in OnShape.

### 4. Download environment assets

**HDRIs** — download indoor/arena HDRIs from [Poly Haven](https://polyhaven.com/hdris)
and place them in `data/hdris/`.

**CC0 Textures** — download PBR texture packs from
[ambientCG](https://ambientcg.com) and extract to `data/cc_textures/`.
BlenderProc expects the ambientCG folder structure.

### 5. Acquire distractor models

**From Objaverse** (bulk download):

```bash
python download_objaverse.py data/distractor_models/objaverse --max-models 100
```

**From reference photos** (SAM 3D):

```bash
python generate_sam3d_models.py data/reference_photos data/distractor_models/sam3d
```

### 6. Generate synthetic images

```bash
blenderproc run render_scenes.py -- config.toml
```

Options:

```
--num-images 5000       Override the image count from config
--render-samples 128    Path-tracing samples per pixel (higher = slower but cleaner)
--start-index 10000     Resume from a specific frame index
```

### 7. Assemble the dataset

Combine synthetic images with real labeled images and split into
train/val/test:

```bash
python ../split_yolo_dataset.py \
    data/synthetic/images \
    data/synthetic/labels \
    data/synthetic_dataset \
    --train 0.9 --val 0.1
```

Visually verify annotations:

```bash
python ../draw_yolo_annotations.py data/synthetic_dataset/train
```

Then train with:

```bash
cd ../../yolo
python train.py path/to/data.yaml yolo11n-pose
```

## File Overview

| File                       | Runs via          | Purpose                                           |
| -------------------------- | ----------------- | ------------------------------------------------- |
| `config.toml`              | —                 | All pipeline parameters                           |
| `prepare_robot_model.py`   | `blenderproc run` | Inspect GLTF colors, validate PBR mapping         |
| `download_objaverse.py`    | `python`          | Download distractor models from Objaverse         |
| `generate_sam3d_models.py` | `python`          | Generate distractor meshes from photos via SAM 3D |
| `render_scenes.py`         | `blenderproc run` | Main rendering pipeline                           |

## Directory Structure

After running the full pipeline:

```
data/
├── models/
│   └── robot.glb                  # OnShape GLTF export
├── distractor_models/
│   ├── objaverse/                 # Downloaded Objaverse models
│   │   └── manifest.json
│   └── sam3d/                     # SAM 3D generated models
│       └── manifest.json
├── hdris/                         # Poly Haven HDRIs
├── cc_textures/                   # ambientCG PBR textures
└── synthetic/
    ├── images/                    # Rendered RGB images (000000.jpg, ...)
    └── labels/                    # YOLO annotations  (000000.txt, ...)
```

## Configuration Reference

See `config.toml` for all options. Key sections:

- `[robot]` — model path, class ID, keypoint positions, color-to-material mapping
- `[materials.*]` — PBR properties and CC texture asset names per material type
- `[distractors]` — model directories, count range, scale range
- `[environment]` — paths to HDRIs and CC textures
- `[camera]` — distance, height, and noise parameters
- `[randomization]` — jitter ranges for materials and lighting
