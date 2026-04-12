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
SMOOTHING_PRESETS = {
    "light": {"iterations": 6, "lambda": 0.35, "mu": -0.37},
    "medium": {"iterations": 12, "lambda": 0.5, "mu": -0.53},
    "strong": {"iterations": 24, "lambda": 0.6, "mu": -0.62},
}


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
    smoothing_method: str | None = "taubin",
    smoothing_iterations: int = 12,
    smoothing_lambda: float = 0.5,
    smoothing_mu: float = -0.53,
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
        mesh_o3d.remove_unreferenced_vertices()

        if smoothing_method == "taubin":
            mesh_o3d = mesh_o3d.filter_smooth_taubin(
                number_of_iterations=smoothing_iterations,
                lambda_filter=smoothing_lambda,
                mu=smoothing_mu,
            )
        elif smoothing_method == "laplacian":
            mesh_o3d = mesh_o3d.filter_smooth_laplacian(
                number_of_iterations=smoothing_iterations,
                lambda_filter=smoothing_lambda,
            )

        mesh_o3d.remove_degenerate_triangles()
        mesh_o3d.remove_duplicated_triangles()
        mesh_o3d.remove_duplicated_vertices()
        mesh_o3d.remove_non_manifold_edges()
        mesh_o3d.remove_unreferenced_vertices()
        mesh_o3d.compute_vertex_normals()

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
        smoothing_desc = (
            "off"
            if smoothing_method is None
            else (
                f"{smoothing_method} (iter={smoothing_iterations}, "
                f"lambda={smoothing_lambda:.3f}, mu={smoothing_mu:.3f})"
                if smoothing_method == "taubin"
                else f"{smoothing_method} (iter={smoothing_iterations}, "
                f"lambda={smoothing_lambda:.3f})"
            )
        )
        print(
            f"    Mesh: {len(tm.vertices)} verts, {len(tm.faces)} faces "
            f"(poisson depth={poisson_depth}, density q={density_quantile:.3f}, "
            f"smoothing={smoothing_desc})"
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
    density_quantile: float = 0.05,
    smoothing_method: str | None = "taubin",
    smoothing_iterations: int = 12,
    smoothing_lambda: float = 0.5,
    smoothing_mu: float = -0.53,
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
            density_quantile=density_quantile,
            smoothing_method=smoothing_method,
            smoothing_iterations=smoothing_iterations,
            smoothing_lambda=smoothing_lambda,
            smoothing_mu=smoothing_mu,
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
    parser.add_argument(
        "--density-quantile",
        type=float,
        default=0.03,
        help="Remove Poisson vertices below this density quantile (default: 0.03). "
        "Lower values keep more surface detail and fill.",
    )
    parser.add_argument(
        "--smoothing-preset",
        type=str,
        default="medium",
        choices=["off", "light", "medium", "strong"],
        help="Mesh smoothing strength preset for glb/obj outputs (default: medium).",
    )
    parser.add_argument(
        "--smoothing-method",
        type=str,
        default="taubin",
        choices=["taubin", "laplacian"],
        help="Mesh smoothing algorithm (default: taubin).",
    )
    parser.add_argument(
        "--smoothing-iterations",
        type=int,
        default=None,
        help="Override smoothing iterations (default comes from --smoothing-preset).",
    )
    parser.add_argument(
        "--smoothing-lambda",
        type=float,
        default=None,
        help="Override smoothing lambda/filter strength (default from preset).",
    )
    parser.add_argument(
        "--smoothing-mu",
        type=float,
        default=None,
        help="Override Taubin mu (ignored for laplacian, default from preset).",
    )
    args = parser.parse_args()

    if not 0.0 <= args.opacity_threshold <= 1.0:
        parser.error("--opacity-threshold must be in [0.0, 1.0]")
    if not 0.0 <= args.density_quantile < 1.0:
        parser.error("--density-quantile must be in [0.0, 1.0)")
    if args.smoothing_iterations is not None and args.smoothing_iterations <= 0:
        parser.error("--smoothing-iterations must be > 0")
    if args.smoothing_preset == "off" and any(
        value is not None
        for value in (args.smoothing_iterations, args.smoothing_lambda, args.smoothing_mu)
    ):
        parser.error(
            "Smoothing overrides require --smoothing-preset light/medium/strong"
        )

    if args.smoothing_preset == "off":
        smoothing_method: str | None = None
        smoothing_iterations = 0
        smoothing_lambda = 0.0
        smoothing_mu = 0.0
    else:
        preset = SMOOTHING_PRESETS[args.smoothing_preset]
        smoothing_method = args.smoothing_method
        smoothing_iterations = (
            args.smoothing_iterations
            if args.smoothing_iterations is not None
            else int(preset["iterations"])
        )
        smoothing_lambda = (
            args.smoothing_lambda
            if args.smoothing_lambda is not None
            else float(preset["lambda"])
        )
        smoothing_mu = (
            args.smoothing_mu if args.smoothing_mu is not None else float(preset["mu"])
        )

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
    if args.format in {"glb", "obj"}:
        if smoothing_method is None:
            print(
                f"Mesh quality: poisson depth={args.poisson_depth}, "
                f"density quantile={args.density_quantile:.3f}, smoothing=off"
            )
        else:
            extra = (
                f", mu={smoothing_mu:.3f}" if smoothing_method == "taubin" else ""
            )
            print(
                f"Mesh quality: poisson depth={args.poisson_depth}, "
                f"density quantile={args.density_quantile:.3f}, "
                f"smoothing={smoothing_method} (iter={smoothing_iterations}, "
                f"lambda={smoothing_lambda:.3f}{extra})"
            )
    elif args.smoothing_preset != "off":
        print("Smoothing options are ignored when --format ply is used.")

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
            density_quantile=args.density_quantile,
            smoothing_method=smoothing_method,
            smoothing_iterations=smoothing_iterations,
            smoothing_lambda=smoothing_lambda,
            smoothing_mu=smoothing_mu,
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
