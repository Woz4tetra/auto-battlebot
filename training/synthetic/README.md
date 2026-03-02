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

**SAM 3D Body** (for distractor generation from reference photos) has heavier
dependencies. Install into the same environment as auto-battlebot and follow the
[official installation guide](https://github.com/facebookresearch/sam3d/blob/main/INSTALL.md). Summary:

1. **Activate the project environment** (from the auto-battlebot repo root)
   ```bash
   source ./scripts/activate_python.sh
   ```

2. **Install PyTorch** (if not already present) from [pytorch.org](https://pytorch.org/get-started/locally/) for your platform.

3. **Install Python dependencies** for SAM 3D Body
   ```bash
   pip install pytorch-lightning pyrender opencv-python yacs scikit-image einops timm dill pandas rich hydra-core hydra-submitit-launcher hydra-colorlog pyrootutils webdataset chump networkx==3.2.1 roma joblib seaborn wandb appdirs appnope ffmpeg cython jsonlines pytest xtcocotools loguru optree fvcore black pycocotools tensorboard huggingface_hub
   ```

4. **Install Detectron2**
   ```bash
   pip install 'git+https://github.com/facebookresearch/detectron2.git@a1ce2f9' --no-build-isolation --no-deps
   ```

5. **Optional:** MoGe — `pip install git+https://github.com/microsoft/MoGe.git`

6. **Optional:** SAM3 (minimal, for inference support)
   ```bash
   mkdir -p ~/sam3
   cd ~/sam3
   git clone https://github.com/facebookresearch/sam3.git
   cd sam3 && pip install -e . && pip install decord psutil
   cd ..   # back to wherever you were
   ```

7. **Clone SAM 3D Body** (the repo is not a pip package — no `setup.py` or `pyproject.toml`)
   ```bash
   mkdir -p ~/sam3
   cd ~/sam3
   git clone https://github.com/facebookresearch/sam3d.git
   ```
   Run the model's scripts from the `sam3d` directory, or add that directory to `PYTHONPATH` when using auto-battlebot's `generate_sam3d_models.py`.

8. **Model checkpoints** are on Hugging Face: [sam-3d-body-dinov3](https://huggingface.co/facebook/sam-3d-body-dinov3), [sam-3d-body-vith](https://huggingface.co/facebook/sam-3d-body-vith). You must **request access** on those repos and be logged in (`huggingface-cli login`) to download. Not available in comprehensively sanctioned jurisdictions.

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

**HDRIs (Poly Haven)**

- **Where:** [Poly Haven HDRIs](https://polyhaven.com/hdris)
- **Format:** `.hdr` or `.exr` only (the pipeline discovers files with these extensions).
- **Resolution:** 2K or 4K is enough for 640x640 renders and keeps download size reasonable; 1K is usable if you want to minimize storage.
- **How many:** **15-30** for good scene variety. More gives more diversity; fewer is fine for testing.
- **Which ones:** Prefer **indoor or arena-like** environments (studios, halls, warehouses) so lighting matches typical competition settings. You can pick at random from those, or add a few outdoor/abstract ones for extra variation.

Place all files directly in `data/hdris/` (no subfolders required).

**CC0 Textures (robot materials + ground)**

Use `download_ambientcg.py` to download textures from [ambientCG](https://ambientcg.com). The script downloads 2K-JPG zip files, extracts only the texture maps BlenderProc needs (Color, Roughness, NormalGL, Metalness, Displacement, AmbientOcclusion), and removes everything else.

Download all textures referenced in `config.toml` in one command:

```bash
python download_ambientcg.py ../data/cc_textures --from-config config.toml
```

Or download individual assets by name or URL:

```bash
python download_ambientcg.py ../data/cc_textures Plastic007 Metal012 Wood037
python download_ambientcg.py ../data/cc_textures "https://ambientcg.com/get?file=Plastic007_2K-JPG.zip"
```

Assets that already exist in the output directory are skipped, so it's safe to re-run.

- **Required:** Download the assets referenced in your `config.toml` under `[materials.*]` -> `cc_texture` (e.g. `Metal012`, `Metal030`, `Plastic006`, `Rubber001`, `Paper001`). Without these, the robot falls back to flat PBR values (no texture).
- **For ground variety:** Add **10-25** extra ambientCG materials (e.g. more metals, plastics, wood, concrete, tiles). The loader only uses assets whose folder name starts with one of its built-in keywords (e.g. `metal`, `plastic`, `wood`, `concrete`, `tiles`), so names like `Metal012` or `Wood037` work.
- **Non-default resolution:** Pass `--resolution 4K-JPG` (or any other ambientCG variant) to override the default `2K-JPG`.

### 5. Acquire distractor models

**From Objaverse** (bulk download):

```bash
python download_objaverse.py ../data/distractor_models/objaverse --max-models 100
```

**From reference photos** (SAM 3D):

```bash
python generate_sam3d_models.py ../data/reference_photos ../data/distractor_models/sam3d
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
| `config.toml`              | --                | All pipeline parameters                           |
| `prepare_robot_model.py`   | `blenderproc run` | Inspect GLTF colors, validate PBR mapping         |
| `download_objaverse.py`    | `python`          | Download distractor models from Objaverse         |
| `download_ambientcg.py`    | `python`          | Download PBR textures from ambientCG              |
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

- `[robot]` -- model path, class ID, keypoint positions, color-to-material mapping
- `[materials.*]` -- PBR properties and CC texture asset names per material type
- `[distractors]` -- model directories, count range, scale range
- `[environment]` -- paths to HDRIs and CC textures
- `[camera]` -- distance, height, and noise parameters
- `[randomization]` -- jitter ranges for materials and lighting
