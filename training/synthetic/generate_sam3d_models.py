"""Generate 3D distractor models from reference photos using SAM 3D Objects.

The SAM 3D Objects repo (https://github.com/facebookresearch/sam-3d-objects)
produces Gaussian splats from single images.  This script runs inference on
a directory of reference photos, converts the splats to triangle meshes
(via convex hull of splat centers), and exports GLB files that BlenderProc
can import as distractor objects.

See README.md for full setup (clone, venv, install, checkpoints).

Usage (run from inside the sam-3d-objects venv):

    python generate_sam3d_models.py ../data/reference_photos ../data/distractor_models/sam3d
    python generate_sam3d_models.py --sam3d-dir ~/sam3/sam-3d-objects ../data/reference_photos ../data/distractor_models/sam3d
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".webp", ".bmp"}


def find_images(photo_dir: Path) -> list[Path]:
    """Recursively find all image files in a directory."""
    images = []
    for ext in IMAGE_EXTENSIONS:
        images.extend(photo_dir.rglob(f"*{ext}"))
        images.extend(photo_dir.rglob(f"*{ext.upper()}"))
    return sorted(set(images))


def create_mask(image_path: Path) -> np.ndarray:
    """Auto-generate a foreground mask for the image.

    Uses ``rembg`` for background removal when available, otherwise returns
    a whole-image mask (every pixel is foreground).
    """
    from PIL import Image

    img = Image.open(image_path).convert("RGB")
    w, h = img.size

    try:
        from rembg import remove

        result = remove(img)
        alpha = np.array(result.convert("RGBA"))[:, :, 3]
        mask = alpha > 128
        if mask.any():
            pct = mask.sum() / mask.size * 100
            print(f"    Auto-masked with rembg ({pct:.0f}% foreground)")
            return mask
    except ImportError:
        pass

    print("    Using whole-image mask (install rembg for better results)")
    return np.ones((h, w), dtype=bool)


def load_inference(sam3d_dir: Path, checkpoint_tag: str = "hf"):
    """Load the SAM 3D Objects inference pipeline.

    Returns ``(inference_fn, load_image_fn)``.
    """
    notebook_dir = sam3d_dir / "notebook"
    for p in [str(sam3d_dir), str(notebook_dir)]:
        if p not in sys.path:
            sys.path.insert(0, p)

    try:
        from inference import Inference, load_image
    except ImportError as exc:
        raise ImportError(
            "Failed to import SAM 3D Objects inference module.\n"
            "Make sure you have:\n"
            "  1. Cloned the repo:\n"
            "       git clone https://github.com/facebookresearch/sam-3d-objects.git\n"
            "  2. Installed dependencies:\n"
            "       cd sam-3d-objects && pip install -e '.[inference]'\n"
            "  See https://github.com/facebookresearch/sam-3d-objects/blob/main/doc/setup.md"
        ) from exc

    config_path = sam3d_dir / "checkpoints" / checkpoint_tag / "pipeline.yaml"
    if not config_path.exists():
        raise FileNotFoundError(
            f"Pipeline config not found: {config_path}\n"
            "Download checkpoints (requires Hugging Face access approval):\n"
            f"  huggingface-cli download facebook/sam-3d-objects "
            f"--local-dir {sam3d_dir / 'checkpoints' / (checkpoint_tag + '-download')}\n"
            f"  mv {sam3d_dir / 'checkpoints' / (checkpoint_tag + '-download') / 'checkpoints'}"
            f" {sam3d_dir / 'checkpoints' / checkpoint_tag}"
        )

    print(f"Loading SAM 3D Objects model from {config_path}...")
    inference = Inference(str(config_path), compile=False)
    return inference, load_image


def splat_ply_to_mesh(ply_path: Path, mesh_path: Path) -> bool:
    """Convert a Gaussian splat PLY to a triangle mesh (GLB/OBJ).

    Extracts splat center positions and builds a convex hull, which is a
    rough but serviceable approximation for distractor objects.
    """
    try:
        import trimesh

        cloud = trimesh.load(str(ply_path), process=False)
        if hasattr(cloud, "vertices") and len(cloud.vertices) >= 4:
            points = np.array(cloud.vertices)
        else:
            from plyfile import PlyData

            ply = PlyData.read(str(ply_path))
            v = ply["vertex"]
            points = np.column_stack([v["x"], v["y"], v["z"]])

        if len(points) < 4:
            print("    Too few points for mesh conversion")
            return False

        mesh = trimesh.PointCloud(points).convex_hull
        mesh.export(str(mesh_path))
        return True
    except Exception as exc:
        print(f"    Mesh conversion failed: {exc}")
        return False


def generate_model(
    inference,
    load_image_fn,
    image_path: Path,
    output_dir: Path,
    stem: str,
    output_format: str,
) -> str | None:
    """Run SAM 3D Objects on a single image and save the result.

    Returns the output filename on success, ``None`` on failure.
    """
    try:
        image = load_image_fn(str(image_path))
        mask = create_mask(image_path)

        output = inference(image, mask, seed=42)

        ply_path = output_dir / f"{stem}.ply"
        output["gs"].save_ply(str(ply_path))

        if output_format == "ply":
            return ply_path.name

        mesh_path = output_dir / f"{stem}.{output_format}"
        if splat_ply_to_mesh(ply_path, mesh_path):
            ply_path.unlink(missing_ok=True)
            return mesh_path.name

        print("    Keeping raw PLY (mesh conversion failed)")
        return ply_path.name

    except Exception as exc:
        print(f"  Failed on {image_path.name}: {exc}")
        return None


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("photo_dir", type=Path, help="Directory of reference photos")
    parser.add_argument("output_dir", type=Path, help="Directory to save 3D models")
    parser.add_argument(
        "--sam3d-dir",
        type=Path,
        default=Path("~/sam3/sam-3d-objects").expanduser(),
        metavar="DIR",
        help="Path to the cloned sam-3d-objects repo (default: ~/sam3/sam-3d-objects)",
    )
    parser.add_argument(
        "--checkpoint-tag",
        type=str,
        default="hf",
        help="Checkpoint subdirectory name (default: hf)",
    )
    parser.add_argument(
        "--format",
        type=str,
        default="glb",
        choices=["glb", "obj", "ply"],
        help="Output format (default: glb). "
        "glb/obj convert Gaussian splats to triangle meshes via convex hull; "
        "ply keeps the raw Gaussian splat.",
    )
    parser.add_argument(
        "--max-images",
        type=int,
        default=None,
        help="Maximum number of images to process",
    )
    args = parser.parse_args()

    sam3d_dir = args.sam3d_dir.resolve()
    if not sam3d_dir.is_dir():
        parser.error(
            f"sam-3d-objects repo not found at {sam3d_dir}\n"
            "Clone with:\n"
            "  git clone https://github.com/facebookresearch/sam-3d-objects.git "
            f"{sam3d_dir}"
        )

    photo_dir: Path = args.photo_dir
    output_dir: Path = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    images = find_images(photo_dir)
    if not images:
        print(f"No images found in {photo_dir}")
        return

    if args.max_images:
        images = images[: args.max_images]

    print(f"Found {len(images)} reference photos")

    inference, load_image_fn = load_inference(sam3d_dir, args.checkpoint_tag)

    manifest: dict[str, dict] = {}
    success_count = 0

    for i, image_path in enumerate(images):
        stem = image_path.stem
        print(f"  [{i + 1}/{len(images)}] Processing {image_path.name}...")

        filename = generate_model(
            inference, load_image_fn, image_path, output_dir, stem, args.format
        )
        if filename is not None:
            manifest[stem] = {
                "source_image": str(image_path),
                "mesh_file": filename,
            }
            success_count += 1

    manifest_path = output_dir / "manifest.json"
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)

    print(f"\nGenerated {success_count}/{len(images)} models in {output_dir}")
    print(f"Manifest written to {manifest_path}")


if __name__ == "__main__":
    main()
