# Synthetic Data Generator for YOLO Keypoint Training

Generates randomized training images with a target robot (annotated with
bounding box + front/back keypoints) and distractor objects. Outputs
YOLO-format labels compatible with the existing training pipeline.

## Prerequisites

Set up the subproject virtual environment:

```bash
cd training/synthetic
./setup.sh
source venv/bin/activate
```

This installs `blenderproc`, `objaverse`, `numpy`, and `opencv-python`.

To install optional dependencies for SAM 3D mesh conversion:

```bash
pip install -e '.[sam3d]'       # trimesh, plyfile, Pillow
pip install -e '.[rembg]'      # background removal for SAM 3D
```

**SAM 3D Objects** (for distractor generation from reference photos) has heavier
dependencies and needs a GPU with at least 32 GB VRAM. Follow the
[official setup guide](https://github.com/facebookresearch/sam-3d-objects/blob/main/doc/setup.md). Summary:

1. **Install Mamba** (if not already installed). Mamba is a fast drop-in
   replacement for conda. On Ubuntu:
   ```bash
   curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
   bash Miniforge3-$(uname)-$(uname -m).sh
   ```
   Follow the prompts, then restart your shell (or `source ~/.bashrc`).

2. **Clone the repo**
   ```bash
   git clone https://github.com/facebookresearch/sam-3d-objects.git ~/sam3/sam-3d-objects
   ```

3. **Create the environment and install**
   ```bash
   cd ~/sam3/sam-3d-objects
   mamba env create -f environments/default.yml
   mamba activate sam3d-objects

   export PIP_EXTRA_INDEX_URL="https://pypi.ngc.nvidia.com https://download.pytorch.org/whl/cu121"
   pip install -e '.[dev]'
   pip install -e '.[p3d]'
   export PIP_FIND_LINKS="https://nvidia-kaolin.s3.us-east-2.amazonaws.com/torch-2.5.1_cu121.html"
   pip install -e '.[inference]'
   pip install 'numpy>=1.26,<2'   # kaolin wheels are compiled against numpy 1.x
   ./patching/hydra
   ```

4. **Download checkpoints** from Hugging Face (requires [access approval](https://huggingface.co/facebook/sam-3d-objects))
   ```bash
   huggingface-cli download facebook/sam-3d-objects \
       --local-dir ~/sam3/sam-3d-objects/checkpoints/hf-download
   mv ~/sam3/sam-3d-objects/checkpoints/hf-download/checkpoints ~/sam3/sam-3d-objects/checkpoints/hf
   rm -rf ~/sam3/sam-3d-objects/checkpoints/hf-download
   ```

5. **Optional:** Install `rembg` for automatic background removal when generating masks from reference photos:
   ```bash
   pip install rembg
   ```
   Without `rembg`, the script uses a whole-image mask (treats every pixel as foreground).

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

**From reference photos** (SAM 3D Objects — requires setup above):

```bash
python generate_sam3d_models.py ../data/reference_photos ../data/distractor_models/sam3d
python generate_sam3d_models.py --sam3d-dir ~/sam3/sam-3d-objects ../data/reference_photos ../data/distractor_models/sam3d
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
| `generate_sam3d_models.py` | `python`          | Generate distractor meshes from photos via SAM 3D Objects |
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
