"""Generate 3D distractor models from reference photos using SAM 3D Objects.

The SAM 3D Objects repo (https://github.com/facebookresearch/sam-3d-objects)
produces Gaussian splats from single images.  This script runs inference on
a directory of reference photos, converts the splats to vertex-colored
triangle meshes via Poisson surface reconstruction, and exports GLB files
that BlenderProc can import as distractor objects.

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


_SH_C0 = 0.2820947917738781


def splat_ply_to_mesh(
    ply_path: Path,
    mesh_path: Path,
    *,
    poisson_depth: int = 8,
    opacity_threshold: float = 0.05,
    density_quantile: float = 0.05,
) -> bool:
    """Convert a Gaussian splat PLY to a vertex-colored triangle mesh.

    Reads splat positions, opacities, and zeroth-order spherical-harmonic
    colour coefficients from the PLY, builds an Open3D point cloud, runs
    Poisson surface reconstruction, trims low-density extrapolation
    artefacts, and exports the result with vertex colours preserved.
    """
    try:
        import open3d as o3d
        import trimesh
        from plyfile import PlyData

        ply = PlyData.read(str(ply_path))
        v = ply["vertex"]
        names = {p.name for p in v.properties}

        positions = np.column_stack([v["x"], v["y"], v["z"]])

        if "opacity" in names:
            opacity = 1.0 / (1.0 + np.exp(-np.array(v["opacity"], dtype=np.float64)))
            keep = opacity >= opacity_threshold
            positions = positions[keep]
        else:
            keep = np.ones(len(positions), dtype=bool)

        if len(positions) < 64:
            print(f"    Too few splats after filtering ({len(positions)})")
            return False

        if {"f_dc_0", "f_dc_1", "f_dc_2"}.issubset(names):
            sh = np.column_stack(
                [
                    np.array(v["f_dc_0"])[keep],
                    np.array(v["f_dc_1"])[keep],
                    np.array(v["f_dc_2"])[keep],
                ]
            )
            colors = np.clip(0.5 + _SH_C0 * sh, 0.0, 1.0)
        else:
            colors = np.full((len(positions), 3), 0.5)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(positions)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30)
        )
        pcd.orient_normals_consistent_tangent_plane(k=15)

        mesh_o3d, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=poisson_depth, linear_fit=True
        )

        densities = np.asarray(densities)
        threshold = np.quantile(densities, density_quantile)
        vertices_to_remove = densities < threshold
        mesh_o3d.remove_vertices_by_mask(vertices_to_remove)

        vertices = np.asarray(mesh_o3d.vertices)
        faces = np.asarray(mesh_o3d.triangles)
        vertex_colors = np.asarray(mesh_o3d.vertex_colors)

        tm = trimesh.Trimesh(
            vertices=vertices,
            faces=faces,
            vertex_colors=(vertex_colors * 255).astype(np.uint8),
            process=False,
        )
        tm.export(str(mesh_path))
        print(
            f"    Mesh: {len(tm.vertices)} verts, {len(tm.faces)} faces "
            f"(poisson depth={poisson_depth})"
        )
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
    *,
    poisson_depth: int = 8,
    opacity_threshold: float = 0.05,
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
        if splat_ply_to_mesh(
            ply_path,
            mesh_path,
            poisson_depth=poisson_depth,
            opacity_threshold=opacity_threshold,
        ):
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
        "glb/obj convert Gaussian splats to vertex-colored triangle meshes "
        "via Poisson surface reconstruction; ply keeps the raw Gaussian splat.",
    )
    parser.add_argument(
        "--max-images",
        type=int,
        default=None,
        help="Maximum number of images to process",
    )
    parser.add_argument(
        "--poisson-depth",
        type=int,
        default=8,
        help="Octree depth for Poisson surface reconstruction (default: 8). "
        "Higher values capture finer detail but are slower.",
    )
    parser.add_argument(
        "--opacity-threshold",
        type=float,
        default=0.05,
        help="Minimum splat opacity to include in reconstruction (default: 0.05). "
        "Lower values keep more semi-transparent splats.",
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
            inference,
            load_image_fn,
            image_path,
            output_dir,
            stem,
            args.format,
            poisson_depth=args.poisson_depth,
            opacity_threshold=args.opacity_threshold,
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
