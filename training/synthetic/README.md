# Synthetic Data Generator for YOLO Keypoint Training

Generates randomized training images with a target robot (annotated with
bounding box + front/back keypoints) and distractor objects. Outputs
YOLO-format labels compatible with the existing training pipeline.

## Prerequisites

### Recommended: Docker (SF3D-ready)

Use the containerized workflow for reproducibility and to avoid host CUDA/pip
ABI conflicts.

Build from repo root:

```bash
docker build -f training/synthetic/Dockerfile -t auto-battlebot-synthetic training/synthetic
```

Run an interactive shell in the container:

```bash
training/synthetic/docker/run_synthetic.sh auto-battlebot-synthetic
```

If your Docker daemon does not have NVIDIA runtime enabled yet, the helper
auto-falls back to CPU mode. You can also force CPU mode:

```bash
training/synthetic/docker/run_synthetic.sh --cpu auto-battlebot-synthetic
```

To fail fast when GPU is required (no fallback), use:

```bash
training/synthetic/docker/run_synthetic.sh --require-gpu auto-battlebot-synthetic
```

Run a command directly:

```bash
training/synthetic/docker/run_synthetic.sh \
  auto-battlebot-synthetic \
  python training/synthetic/generate_reference_models.py --help
```

One-time model auth inside container (SF3D weights are gated on Hugging Face):

```bash
huggingface-cli login
touch /opt/hf/.has_hf_login
```

Model caches are persisted on host by default:

- Hugging Face cache: `${HOME}/.cache/huggingface` -> `/opt/hf`
- rembg U2Net cache: `${HOME}/.cache/u2net` -> `/root/.u2net`

To override U2Net cache location:

```bash
U2NET_HOST_CACHE=/path/to/cache training/synthetic/docker/run_synthetic.sh auto-battlebot-synthetic
```

You can run all synthetic tools from this container, including:

- `python training/synthetic/download_polyhaven_hdris.py ...`
- `python training/synthetic/download_ambientcg.py ...`
- `python training/synthetic/generate_reference_models.py ...`
- `python training/synthetic/benchmark_reference_models.py ...`
- `blenderproc run training/synthetic/render_scenes.py -- training/synthetic/config.toml`

The image includes both backends:

- `triposr` CLI (wrapper around TripoSR `run.py`)
- `python scripts/sf3d_backend_wrapper.py`

### Host setup (optional/manual)

**Install Mamba** (if not already installed). Mamba is a fast drop-in replacement
for conda. On Ubuntu:

```bash
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
```

Follow the prompts, then restart your shell (or `source ~/.bashrc`).

Set up the project environment:

```bash
cd training/synthetic
mamba create -n battlebot-synthetic python=3.11
mamba activate battlebot-synthetic
pip install -e .
```

This installs `blenderproc`, `objaverse`, and `opencv-python`.

To install optional dependencies for segmentation-aware reference model generation:

```bash
pip install -e '.[reference3d]'   # Pillow + trimesh + tqdm progress bars
```

For realism-first generation, use an external single-image 3D backend.
This repo now includes an SF3D backend wrapper:

```bash
# In Docker, SF3D lives at /opt/sf3d and wrapper is in this repo.
python scripts/sf3d_backend_wrapper.py --help
```

If no external backend is available, the pipeline automatically falls back to a
segmentation-driven silhouette extrusion backend.

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

Download a defined number automatically:

```bash
python download_polyhaven_hdris.py ../data/hdris --count 20
```

Useful options:

```bash
# Prefer 4K EXR files
python download_polyhaven_hdris.py ../data/hdris --count 20 --resolution 4k --format exr

# Fully random sampling (disable indoor keyword preference)
python download_polyhaven_hdris.py ../data/hdris --count 20 --no-indoor-preference
```

The downloader uses Poly Haven's API, picks random HDRIs (indoor-biased by default), and skips asset IDs that already exist in `data/hdris/`.

Place all HDRIs directly in `data/hdris/` (no subfolders required).

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

Or list a random set of texture IDs first (without downloading), then copy the generated command:

```bash
python download_ambientcg.py ../data/cc_textures --list-random 20
python download_ambientcg.py ../data/cc_textures --list-random 20 --seed 7
```

Assets that already exist in the output directory are skipped, so it's safe to re-run.

- **Required:** Download the assets referenced in your `config.toml` under `[materials.*]` -> `cc_texture` (e.g. `Metal012`, `Metal030`, `Plastic006`, `Rubber001`, `Paper001`). Without these, the robot falls back to flat PBR values (no texture).
- **For ground variety:** Add **10-25** extra ambientCG materials (e.g. more metals, plastics, wood, concrete, tiles). The loader only uses assets whose folder name starts with one of its built-in keywords (e.g. `metal`, `plastic`, `wood`, `concrete`, `tiles`), so names like `Metal012` or `Wood037` work.
- **Non-default resolution:** Pass `--resolution 4K-JPG` (or any other ambientCG variant) to override the default `2K-JPG`.
- **Discovery mode:** `--list-random N` prints a random set of available material IDs and a ready-to-run download command (use `--seed` for reproducible samples).

### 5. Acquire distractor models

**From Objaverse** (bulk download):

```bash
python download_objaverse.py ../data/distractor_models/objaverse --max-models 100
```

**From segmentation-labeled reference frames** (`nhrl_seg`-style YOLO-seg labels):

Expected format per image: a matching `.txt` row in YOLO-seg polygon format:
`class x1 y1 x2 y2 ... xn yn` (normalized coordinates).

```bash
python generate_reference_models.py \
  ../data/nhrl_seg \
  ../data/distractor_models/reference3d \
  --backend triposr \
  --backend-cmd 'triposr --input "{input}" --output "{output}"'

# Or use SF3D through the wrapper:
python generate_reference_models.py \
  ../data/nhrl_seg \
  ../data/distractor_models/reference3d \
  --backend triposr \
  --backend-cmd 'python scripts/sf3d_backend_wrapper.py --input "{input}" --output "{output}"'
```

Useful controls:

```bash
# Skip floor labels (example: label 1), evaluate several jittered candidates
python generate_reference_models.py \
  ../data/nhrl_seg \
  ../data/distractor_models/reference3d \
  --floor-labels 1 \
  --num-candidates 4 \
  --quality-threshold 0.35

# Backend fallback only (no external model), useful for smoke tests
python generate_reference_models.py \
  ../data/nhrl_seg \
  ../data/distractor_models/reference3d \
  --backend silhouette-extrude
```

Reference-model generation settings:

- `--backend`: `triposr` (primary) or `silhouette-extrude`.
- `--backend-cmd`: command template for the external single-image 3D backend.
- `--floor-labels`: label IDs to skip (for example floor class IDs).
- `--num-candidates`: candidates per instance; best-scoring mesh is kept.
- `--full-image-num-candidates`: candidates per image in plain image-folder mode (default: 1 mesh per image).
- `--quality-threshold`: drop low-quality meshes (or keep with `--keep-rejected`).
- `--target-native-size`: normalize distractor scale for stable scene placement.

### SF3D troubleshooting

- `No module named 'torch'` while building `texture_baker`:
  - Install torch first, then install `texture_baker` and `uv_unwrapper` with `--no-build-isolation`.
  - The provided Dockerfile already does this.
- `undefined symbol: ncclCommWindowDeregister` when importing torch:
  - This indicates CUDA/NCCL runtime mismatch on host.
  - Use the Docker image to keep CUDA + torch ABI aligned.
- `gpytoolbox` wheel build errors:
  - Ensure the container has `cmake`, compiler toolchain, and pinned python/torch versions.
  - The provided Docker image includes these dependencies.
- `docker run --gpus all` fails with `could not select device driver "" with capabilities: [[gpu]]`:
  - Install and configure NVIDIA Container Toolkit:
    ```bash
    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    ```
  - Verify GPU passthrough in Docker:
    ```bash
    docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
    ```
  - If you still need to proceed before GPU runtime is configured, use CPU mode:
    `training/synthetic/docker/run_synthetic.sh --cpu ...`.
- SF3D fails with `NotImplementedError: Could not run 'texture_baker_cpp::rasterize' ... CUDA backend`:
  - Cause: `texture_baker` was built CPU-only.
  - Fix: rebuild the image from updated Dockerfile (it now forces `USE_CUDA=1` during `texture_baker` install):
    ```bash
    docker build -f training/synthetic/Dockerfile -t auto-battlebot-synthetic training/synthetic
    ```

Run a quick benchmark against an existing baseline directory:

```bash
python benchmark_reference_models.py \
  --candidate-dir ../data/distractor_models/reference3d \
  --baseline-dir ../data/distractor_models/sam3d \
  --output-json ../data/distractor_models/reference3d/benchmark_report.json
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

| File                       | Runs via          | Purpose                                                   |
| -------------------------- | ----------------- | --------------------------------------------------------- |
| `config.toml`              | --                | All pipeline parameters                                   |
| `prepare_robot_model.py`   | `blenderproc run` | Inspect GLTF colors, validate PBR mapping                 |
| `download_polyhaven_hdris.py` | `python`       | Download a defined number of random HDRIs from Poly Haven |
| `download_objaverse.py`    | `python`          | Download distractor models from Objaverse                 |
| `download_ambientcg.py`    | `python`          | Download PBR textures from ambientCG, or list random IDs |
| `benchmark_reference_models.py` | `python`    | Compare candidate distractor models to a baseline directory |
| `render_scenes.py`         | `blenderproc run` | Main rendering pipeline                                   |

## Directory Structure

After running the full pipeline:

```
data/
├── models/
│   └── robot.glb                  # OnShape GLTF export
├── distractor_models/
│   ├── objaverse/                 # Downloaded Objaverse models
│   │   └── manifest.json
│   └── reference3d/               # Segmentation-aware generated models
│       └── manifest.json
├── hdris/                         # Poly Haven HDRIs
├── cc_textures/                   # ambientCG PBR textures
└── synthetic/
    ├── images/                    # Rendered RGB images (000000.jpg, ...)
    └── labels/                    # YOLO annotations  (000000.txt, ...)
```

## Configuration Reference

See `config.toml` for all options. Key sections:

- `[[robots]]` -- one or more robot models, each with model path, class ID, keypoint positions, color-to-material mapping, and selection weight
- `[materials.*]` -- PBR properties and CC texture asset names per material type
- `[distractors]` -- model directories, count range, scale range
- `[environment]` -- paths to HDRIs and CC textures
- `[camera]` -- distance, height, and noise parameters
- `[randomization]` -- jitter ranges for materials and lighting
