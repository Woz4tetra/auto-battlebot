"""Generate 3D distractor models from reference photos using Meta SAM 3D.

Requires the SAM 3D repository to be installed:
    git clone https://github.com/facebookresearch/sam3d.git
    cd sam3d && pip install -e .

Usage:
    python generate_sam3d_models.py data/reference_photos data/distractor_models/sam3d
    python generate_sam3d_models.py data/reference_photos data/distractor_models/sam3d --device cuda:0
"""

import argparse
import json
from pathlib import Path

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".webp", ".bmp"}


def find_images(photo_dir: Path) -> list[Path]:
    """Recursively find all image files in a directory."""
    images = []
    for ext in IMAGE_EXTENSIONS:
        images.extend(photo_dir.rglob(f"*{ext}"))
        images.extend(photo_dir.rglob(f"*{ext.upper()}"))
    return sorted(set(images))


def load_sam3d_model(device: str):
    """Load the SAM 3D Objects model.

    This function wraps the SAM 3D import so we fail fast with a clear message
    if the package isn't installed.
    """
    try:
        from sam3d import SAM3DObjects
    except ImportError:
        raise ImportError(
            "SAM 3D is not installed. Install it from:\n"
            "  git clone https://github.com/facebookresearch/sam3d.git\n"
            "  cd sam3d && pip install -e .\n"
            "See https://ai.meta.com/sam3d/ for details."
        )

    print(f"Loading SAM 3D Objects model on {device}...")
    model = SAM3DObjects.from_pretrained(device=device)
    return model


def generate_mesh(model, image_path: Path, output_path: Path) -> bool:
    """Run SAM 3D inference on a single image and save the mesh.

    Returns True on success, False on failure.
    """
    try:
        result = model.predict(str(image_path))
        mesh = result.mesh

        if output_path.suffix == ".glb":
            mesh.export(str(output_path), file_type="glb")
        elif output_path.suffix == ".obj":
            mesh.export(str(output_path), file_type="obj")
        else:
            mesh.export(str(output_path))

        return True
    except Exception as e:
        print(f"  Failed on {image_path.name}: {e}")
        return False


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("photo_dir", type=Path, help="Directory of reference photos")
    parser.add_argument("output_dir", type=Path, help="Directory to save 3D models")
    parser.add_argument(
        "--device",
        type=str,
        default="cuda:0",
        help="Torch device (default: cuda:0)",
    )
    parser.add_argument(
        "--format",
        type=str,
        default="glb",
        choices=["glb", "obj"],
        help="Output mesh format (default: glb)",
    )
    parser.add_argument(
        "--max-images",
        type=int,
        default=None,
        help="Maximum number of images to process",
    )
    args = parser.parse_args()

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

    model = load_sam3d_model(args.device)

    manifest = {}
    success_count = 0
    for i, image_path in enumerate(images):
        stem = image_path.stem
        output_path = output_dir / f"{stem}.{args.format}"

        print(f"  [{i + 1}/{len(images)}] Processing {image_path.name}...")
        if generate_mesh(model, image_path, output_path):
            manifest[stem] = {
                "source_image": str(image_path),
                "mesh_file": output_path.name,
            }
            success_count += 1

    manifest_path = output_dir / "manifest.json"
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)

    print(f"\nGenerated {success_count}/{len(images)} meshes in {output_dir}")
    print(f"Manifest written to {manifest_path}")


if __name__ == "__main__":
    main()
