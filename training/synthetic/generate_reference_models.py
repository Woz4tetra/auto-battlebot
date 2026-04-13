"""Generate distractor meshes from segmentation-labeled reference images.

This script replaces the SAM3D-specific workflow with a backend-agnostic,
segmentation-aware pipeline:

1) Discover image/annotation pairs (YOLO-seg `.txt` polygons or `*_mask.png`).
2) Extract per-instance masked crops (excluding configured floor labels).
3) Generate candidate meshes with a chosen backend (`triposr` or fallback).
4) Post-process and quality-score candidates.
5) Export flat GLB files + a manifest compatible with render_scenes.py.
"""

from __future__ import annotations

import argparse
import json
import random
import shlex
import subprocess
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Iterable, Iterator, TypeVar

import numpy as np
from PIL import Image, ImageDraw

IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".webp", ".bmp")

if TYPE_CHECKING:
    import trimesh

T = TypeVar("T")


def progress_iter(
    iterable: Iterable[T], *, total: int | None = None, desc: str = ""
) -> Iterator[T]:
    """Wrap an iterable with a progress bar when tqdm is available."""
    try:
        from tqdm import tqdm  # type: ignore

        yield from tqdm(iterable, total=total, desc=desc, unit="item")
        return
    except ModuleNotFoundError:
        pass

    if total is not None and total > 0:
        print(f"{desc}: 0/{total}")
    for idx, item in enumerate(iterable, start=1):
        if total is not None and total > 0:
            step = max(1, total // 10)
            if idx == total or idx % step == 0:
                print(f"{desc}: {idx}/{total}")
        yield item


@dataclass(frozen=True)
class InstanceSample:
    sample_id: str
    image_path: Path
    annotation_path: Path
    annotation_format: str
    label_id: int
    bbox_xyxy: tuple[int, int, int, int]
    pixel_area: int
    polygon_norm: tuple[tuple[float, float], ...] | None = None


@dataclass
class MeshCandidate:
    mesh_path: Path
    quality_score: float
    diagnostics: dict[str, float | int | str | bool]


def find_image_mask_pairs(dataset_root: Path) -> list[tuple[Path, Path]]:
    pairs: list[tuple[Path, Path]] = []
    mask_files = sorted(dataset_root.rglob("*_mask.png"))
    for mask_path in mask_files:
        stem = mask_path.stem
        if not stem.endswith("_mask"):
            continue
        image_stem = stem[: -len("_mask")]
        image_path = None
        for ext in IMAGE_EXTENSIONS:
            candidate = mask_path.with_name(f"{image_stem}{ext}")
            if candidate.exists():
                image_path = candidate
                break
        if image_path is not None:
            pairs.append((image_path, mask_path))
    return pairs


def parse_yolo_seg_rows(label_path: Path) -> list[tuple[int, list[tuple[float, float]]]]:
    rows: list[tuple[int, list[tuple[float, float]]]] = []
    try:
        text = label_path.read_text(encoding="utf-8")
    except OSError:
        return rows

    for line in text.splitlines():
        parts = line.strip().split()
        if not parts:
            continue
        try:
            class_id = int(float(parts[0]))
            values = [float(v) for v in parts[1:]]
        except ValueError:
            continue

        # YOLO-seg row: class x1 y1 ... xn yn
        if len(values) < 6 or len(values) % 2 != 0:
            continue
        polygon = [(values[i], values[i + 1]) for i in range(0, len(values), 2)]
        if len(polygon) < 3:
            continue
        rows.append((class_id, polygon))
    return rows


def _match_image_for_label(
    label_path: Path, image_paths_by_stem: dict[str, list[Path]]
) -> Path | None:
    stem = label_path.stem
    for ext in IMAGE_EXTENSIONS:
        same_dir = label_path.with_suffix(ext)
        if same_dir.exists():
            return same_dir

    label_text = str(label_path)
    if "/labels/" in label_text:
        for ext in IMAGE_EXTENSIONS:
            candidate = Path(label_text.replace("/labels/", "/images/")).with_suffix(ext)
            if candidate.exists():
                return candidate

    matches = image_paths_by_stem.get(stem, [])
    if len(matches) == 1:
        return matches[0]
    if matches:
        # Prefer shortest path as a deterministic tie-breaker.
        return sorted(matches, key=lambda p: len(str(p)))[0]
    return None


def find_yolo_seg_pairs(dataset_root: Path) -> list[tuple[Path, Path]]:
    images = find_plain_images(dataset_root)
    images_by_stem: dict[str, list[Path]] = {}
    for img in images:
        images_by_stem.setdefault(img.stem, []).append(img)

    pairs: list[tuple[Path, Path]] = []
    label_paths = sorted(dataset_root.rglob("*.txt"))
    for label_path in progress_iter(
        label_paths, total=len(label_paths), desc="Scanning YOLO-seg labels"
    ):
        image_path = _match_image_for_label(label_path, images_by_stem)
        if image_path is None:
            continue
        if parse_yolo_seg_rows(label_path):
            pairs.append((image_path, label_path))
    return pairs


def find_yolo_seg_pairs_sampled(
    dataset_root: Path,
    *,
    floor_labels: set[int],
    max_frames_per_class: int | None,
) -> list[tuple[Path, Path]]:
    """Discover YOLO-seg pairs with optional per-class frame caps.

    When max_frames_per_class is set, this limits how many source frames are
    selected for each class (counted once per image, even if multiple polygons
    of the same class exist), and stops scanning once all discovered classes
    have reached the cap.
    """
    if max_frames_per_class is None:
        return find_yolo_seg_pairs(dataset_root)

    images = find_plain_images(dataset_root)
    images_by_stem: dict[str, list[Path]] = {}
    for img in images:
        images_by_stem.setdefault(img.stem, []).append(img)

    label_paths = sorted(dataset_root.rglob("*.txt"))
    class_frame_counts: dict[int, int] = {}
    pairs: list[tuple[Path, Path]] = []
    for label_path in progress_iter(
        label_paths, total=len(label_paths), desc="Scanning YOLO-seg labels"
    ):
        image_path = _match_image_for_label(label_path, images_by_stem)
        if image_path is None:
            continue

        seg_rows = parse_yolo_seg_rows(label_path)
        if not seg_rows:
            continue

        classes_in_frame = sorted(
            {int(class_id) for class_id, _ in seg_rows if int(class_id) not in floor_labels}
        )
        if not classes_in_frame:
            continue

        should_keep = any(
            class_frame_counts.get(class_id, 0) < max_frames_per_class
            for class_id in classes_in_frame
        )
        if not should_keep:
            continue

        pairs.append((image_path, label_path))
        for class_id in classes_in_frame:
            current = class_frame_counts.get(class_id, 0)
            if current < max_frames_per_class:
                class_frame_counts[class_id] = current + 1

        if class_frame_counts and all(
            count >= max_frames_per_class for count in class_frame_counts.values()
        ):
            print(
                "Reached per-class frame cap "
                f"({max_frames_per_class}) for discovered classes; stopping early."
            )
            break
    return pairs


def find_plain_images(input_dir: Path) -> list[Path]:
    files: list[Path] = []
    for ext in IMAGE_EXTENSIONS:
        files.extend(input_dir.rglob(f"*{ext}"))
        files.extend(input_dir.rglob(f"*{ext.upper()}"))
    return sorted(set(files))


def rasterize_polygon_mask(
    width: int, height: int, polygon_norm: list[tuple[float, float]]
) -> np.ndarray:
    canvas = Image.new("L", (width, height), 0)
    draw = ImageDraw.Draw(canvas)
    points = [
        (
            int(round(np.clip(x, 0.0, 1.0) * (width - 1))),
            int(round(np.clip(y, 0.0, 1.0) * (height - 1))),
        )
        for x, y in polygon_norm
    ]
    if len(points) >= 3:
        draw.polygon(points, fill=255)
    return (np.array(canvas) > 0).astype(np.uint8)


def _mask_bbox(mask: np.ndarray) -> tuple[int, int, int, int] | None:
    ys, xs = np.where(mask > 0)
    if len(xs) == 0 or len(ys) == 0:
        return None
    x0, x1 = int(xs.min()), int(xs.max()) + 1
    y0, y1 = int(ys.min()), int(ys.max()) + 1
    return (x0, y0, x1, y1)


def discover_samples(
    input_dir: Path,
    *,
    min_mask_area: int,
    floor_labels: set[int],
    max_frames_per_class: int | None,
    max_images: int | None,
    max_instances_per_image: int | None,
) -> list[InstanceSample]:
    samples: list[InstanceSample] = []
    yolo_pairs = find_yolo_seg_pairs_sampled(
        input_dir,
        floor_labels=floor_labels,
        max_frames_per_class=max_frames_per_class,
    )
    if yolo_pairs:
        if max_images is not None:
            yolo_pairs = yolo_pairs[:max_images]
        for image_path, label_path in progress_iter(
            yolo_pairs, total=len(yolo_pairs), desc="Discovering YOLO instances"
        ):
            w, h = Image.open(image_path).convert("RGB").size
            seg_rows = parse_yolo_seg_rows(label_path)
            kept = 0
            for row_idx, (class_id, polygon_norm) in enumerate(seg_rows):
                if class_id in floor_labels:
                    continue
                inst_mask = rasterize_polygon_mask(w, h, polygon_norm)
                area = int(inst_mask.sum())
                if area < min_mask_area:
                    continue
                bbox = _mask_bbox(inst_mask)
                if bbox is None:
                    continue
                sample_id = f"{image_path.stem}_cls{class_id}_seg{row_idx:02d}"
                samples.append(
                    InstanceSample(
                        sample_id=sample_id,
                        image_path=image_path,
                        annotation_path=label_path,
                        annotation_format="yolo_seg",
                        label_id=int(class_id),
                        bbox_xyxy=bbox,
                        pixel_area=area,
                        polygon_norm=tuple((float(x), float(y)) for x, y in polygon_norm),
                    )
                )
                kept += 1
                if (
                    max_instances_per_image is not None
                    and kept >= max_instances_per_image
                ):
                    break
        return samples

    pairs = find_image_mask_pairs(input_dir)
    if pairs:
        if max_images is not None:
            pairs = pairs[:max_images]
        for image_path, mask_path in progress_iter(
            pairs, total=len(pairs), desc="Discovering segmask instances"
        ):
            mask = np.array(Image.open(mask_path))
            if mask.ndim == 3:
                mask = mask[:, :, 0]
            labels = np.unique(mask)
            labels = [int(v) for v in labels if int(v) > 0 and int(v) not in floor_labels]
            labels.sort()
            if max_instances_per_image is not None:
                labels = labels[:max_instances_per_image]
            for label in labels:
                inst_mask = mask == label
                area = int(inst_mask.sum())
                if area < min_mask_area:
                    continue
                ys, xs = np.where(inst_mask)
                x0, x1 = int(xs.min()), int(xs.max()) + 1
                y0, y1 = int(ys.min()), int(ys.max()) + 1
                sample_id = f"{image_path.stem}_lbl{label}"
                samples.append(
                    InstanceSample(
                        sample_id=sample_id,
                        image_path=image_path,
                        annotation_path=mask_path,
                        annotation_format="segmask_png",
                        label_id=label,
                        bbox_xyxy=(x0, y0, x1, y1),
                        pixel_area=area,
                    )
                )
        return samples

    # Fallback mode: plain photo directory (whole image becomes one sample).
    images = find_plain_images(input_dir)
    if max_images is not None:
        images = images[:max_images]
    for image_path in images:
        img = np.array(Image.open(image_path).convert("RGB"))
        h, w = img.shape[:2]
        samples.append(
            InstanceSample(
                sample_id=image_path.stem,
                image_path=image_path,
                annotation_path=image_path,
                annotation_format="full_image",
                label_id=1,
                bbox_xyxy=(0, 0, w, h),
                pixel_area=int(h * w),
            )
        )
    return samples


def extract_rgba_crop(
    sample: InstanceSample,
    *,
    target_size: int,
    crop_padding: float,
    jitter_scale: float = 1.0,
    jitter_shift: tuple[float, float] = (0.0, 0.0),
) -> np.ndarray:
    rgb = np.array(Image.open(sample.image_path).convert("RGB"))
    h, w = rgb.shape[:2]

    if sample.annotation_format == "yolo_seg" and sample.polygon_norm is not None:
        inst = rasterize_polygon_mask(
            w, h, [(float(x), float(y)) for x, y in sample.polygon_norm]
        )
    elif (
        sample.annotation_format == "segmask_png"
        and sample.annotation_path.suffix.lower() == ".png"
        and sample.annotation_path.name.endswith("_mask.png")
    ):
        mask = np.array(Image.open(sample.annotation_path))
        if mask.ndim == 3:
            mask = mask[:, :, 0]
        inst = (mask == sample.label_id).astype(np.uint8)
    else:
        inst = np.ones((h, w), dtype=np.uint8)

    x0, y0, x1, y1 = sample.bbox_xyxy
    bw = max(1, x1 - x0)
    bh = max(1, y1 - y0)
    cx = (x0 + x1) / 2.0 + jitter_shift[0] * bw
    cy = (y0 + y1) / 2.0 + jitter_shift[1] * bh
    side = max(bw, bh) * (1.0 + 2.0 * crop_padding) * jitter_scale
    side = max(16.0, side)

    x0f = int(round(cx - side / 2.0))
    y0f = int(round(cy - side / 2.0))
    x1f = int(round(cx + side / 2.0))
    y1f = int(round(cy + side / 2.0))

    pad_left = max(0, -x0f)
    pad_top = max(0, -y0f)
    pad_right = max(0, x1f - w)
    pad_bottom = max(0, y1f - h)

    x0c = max(0, x0f)
    y0c = max(0, y0f)
    x1c = min(w, x1f)
    y1c = min(h, y1f)

    crop_rgb = rgb[y0c:y1c, x0c:x1c]
    crop_mask = inst[y0c:y1c, x0c:x1c]
    if crop_rgb.size == 0 or crop_mask.size == 0:
        crop_rgb = np.zeros((target_size, target_size, 3), dtype=np.uint8)
        crop_mask = np.zeros((target_size, target_size), dtype=np.uint8)
    else:
        crop_rgb = np.pad(
            crop_rgb,
            ((pad_top, pad_bottom), (pad_left, pad_right), (0, 0)),
            mode="constant",
            constant_values=0,
        )
        crop_mask = np.pad(
            crop_mask,
            ((pad_top, pad_bottom), (pad_left, pad_right)),
            mode="constant",
            constant_values=0,
        )
        crop_rgb = np.array(
            Image.fromarray(crop_rgb, mode="RGB").resize(
                (target_size, target_size), Image.Resampling.LANCZOS
            )
        )
        crop_mask = np.array(
            Image.fromarray((crop_mask > 0).astype(np.uint8) * 255, mode="L").resize(
                (target_size, target_size), Image.Resampling.NEAREST
            )
        )

    alpha = (crop_mask > 0).astype(np.uint8) * 255
    rgba = np.dstack((crop_rgb, alpha))
    return rgba


def run_command_template(command_template: str, *, input_rgba: Path, output_mesh: Path) -> tuple[bool, str]:
    cmd = command_template.format(
        input=str(input_rgba),
        output=str(output_mesh),
        output_dir=str(output_mesh.parent),
        output_stem=output_mesh.stem,
    )
    parsed = shlex.split(cmd)
    if not parsed:
        return False, "empty backend command"

    start = time.monotonic()
    next_heartbeat_s = 10
    print(f"    [backend] starting: {parsed[0]}")
    proc = subprocess.Popen(
        parsed,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )

    stdout_lines: list[str] = []
    stderr_lines: list[str] = []

    def _forward(pipe: subprocess.PIPE, sink: list[str], label: str) -> None:
        if pipe is None:
            return
        for line in iter(pipe.readline, ""):
            sink.append(line)
            print(f"    [backend {label}] {line.rstrip()}")
        pipe.close()

    t_out = threading.Thread(
        target=_forward, args=(proc.stdout, stdout_lines, "stdout"), daemon=True
    )
    t_err = threading.Thread(
        target=_forward, args=(proc.stderr, stderr_lines, "stderr"), daemon=True
    )
    t_out.start()
    t_err.start()

    while proc.poll() is None:
        elapsed = int(time.monotonic() - start)
        if elapsed >= next_heartbeat_s:
            print(f"    [backend] still running... {elapsed}s elapsed")
            next_heartbeat_s += 10
        time.sleep(1.0)

    ret = int(proc.wait())
    t_out.join(timeout=5.0)
    t_err.join(timeout=5.0)

    elapsed_total = int(time.monotonic() - start)
    print(f"    [backend] finished in {elapsed_total}s (exit={ret})")

    output_parts: list[str] = []
    if stdout_lines:
        output_parts.append("[stdout]\n" + "".join(stdout_lines).strip())
    if stderr_lines:
        output_parts.append("[stderr]\n" + "".join(stderr_lines).strip())
    output = "\n\n".join(part for part in output_parts if part.strip())
    return ret == 0, output


def generate_silhouette_mesh(rgba: np.ndarray, output_mesh: Path) -> tuple[bool, str]:
    import trimesh

    alpha = rgba[:, :, 3] > 0
    if alpha.sum() < 64:
        return False, "mask too small for silhouette extrusion"

    depth_layers = int(np.clip(max(rgba.shape[0], rgba.shape[1]) // 8, 12, 64))
    base = Image.fromarray(alpha.astype(np.uint8) * 255, mode="L")
    volume = np.zeros((depth_layers, rgba.shape[0], rgba.shape[1]), dtype=bool)
    for z in range(depth_layers):
        zt = z / max(depth_layers - 1, 1)
        center_weight = 1.0 - abs(2.0 * zt - 1.0)
        shrink = 0.6 + 0.4 * center_weight
        new_w = max(4, int(round(base.width * shrink)))
        new_h = max(4, int(round(base.height * shrink)))
        layer = base.resize((new_w, new_h), Image.Resampling.NEAREST)
        canvas = Image.new("L", base.size, 0)
        ox = (base.width - new_w) // 2
        oy = (base.height - new_h) // 2
        canvas.paste(layer, (ox, oy))
        volume[z] = np.array(canvas) > 0

    try:
        mesh = trimesh.voxel.ops.matrix_to_marching_cubes(volume, pitch=1.0)
    except Exception as exc:  # pragma: no cover - defensive path
        return False, f"marching cubes failed: {exc}"

    if mesh.is_empty or len(mesh.faces) == 0:
        return False, "empty mesh from silhouette volume"

    apply_vertex_colors_from_rgba(mesh, rgba)
    mesh.export(str(output_mesh))
    return True, "ok"


def apply_vertex_colors_from_rgba(mesh: trimesh.Trimesh, rgba: np.ndarray) -> None:
    if len(mesh.vertices) == 0:
        return
    vertices = mesh.vertices
    mins = vertices.min(axis=0)
    maxs = vertices.max(axis=0)
    extents = np.maximum(maxs - mins, 1e-6)
    axes = np.argsort(extents)[::-1]
    ax_u, ax_v = int(axes[0]), int(axes[1])
    u = (vertices[:, ax_u] - mins[ax_u]) / extents[ax_u]
    v = (vertices[:, ax_v] - mins[ax_v]) / extents[ax_v]
    px = np.clip((u * (rgba.shape[1] - 1)).astype(int), 0, rgba.shape[1] - 1)
    py = np.clip(((1.0 - v) * (rgba.shape[0] - 1)).astype(int), 0, rgba.shape[0] - 1)
    colors = rgba[py, px].copy()
    colors[:, 3] = 255
    mesh.visual.vertex_colors = colors


def load_mesh(path: Path) -> trimesh.Trimesh | None:
    import trimesh

    try:
        loaded = trimesh.load(str(path), force="mesh")
    except Exception:
        return None
    if isinstance(loaded, trimesh.Trimesh):
        return loaded
    return None


def postprocess_mesh(
    mesh: trimesh.Trimesh,
    *,
    target_max_faces: int,
    target_native_size: float,
    rgba_for_color: np.ndarray | None,
) -> trimesh.Trimesh:
    pieces = mesh.split(only_watertight=False)
    if len(pieces) > 1:
        mesh = max(pieces, key=lambda p: float(p.area))

    mesh.remove_unreferenced_vertices()
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_infinite_values()

    if len(mesh.faces) > target_max_faces:
        try:
            mesh = mesh.simplify_quadric_decimation(target_max_faces)
        except Exception:
            pass

    if mesh.visual.kind != "vertex" and rgba_for_color is not None:
        apply_vertex_colors_from_rgba(mesh, rgba_for_color)

    bounds = mesh.bounds
    extents = bounds[1] - bounds[0]
    longest = float(np.max(extents))
    if longest > 1e-6:
        mesh.apply_translation(-mesh.bounding_box.centroid)
        mesh.apply_scale(target_native_size / longest)

    mesh.fix_normals()
    return mesh


def score_mesh(mesh: trimesh.Trimesh) -> tuple[float, dict[str, float | int | bool]]:
    faces = int(len(mesh.faces))
    vertices = int(len(mesh.vertices))
    components = len(mesh.split(only_watertight=False))
    bounds = mesh.bounds
    extents = bounds[1] - bounds[0]
    longest = float(np.max(extents))
    shortest = float(np.max([np.min(extents), 1e-6]))
    aspect = longest / shortest

    face_score = np.clip((faces - 500.0) / 8000.0, 0.0, 1.0)
    component_penalty = np.clip((components - 1) / 8.0, 0.0, 1.0)
    aspect_penalty = np.clip((aspect - 3.0) / 10.0, 0.0, 1.0)
    watertight_bonus = 1.0 if bool(mesh.is_watertight) else 0.0
    score = 0.55 * face_score + 0.25 * watertight_bonus + 0.2 * (1.0 - component_penalty)
    score *= 1.0 - 0.4 * aspect_penalty
    score = float(np.clip(score, 0.0, 1.0))

    metrics: dict[str, float | int | bool] = {
        "faces": faces,
        "vertices": vertices,
        "components": components,
        "aspect_ratio": float(aspect),
        "is_watertight": bool(mesh.is_watertight),
    }
    return score, metrics


def should_reject(metrics: dict[str, float | int | bool], *, min_faces: int) -> str | None:
    faces = int(metrics["faces"])
    components = int(metrics["components"])
    aspect = float(metrics["aspect_ratio"])
    if faces < min_faces:
        return "too_few_faces"
    if components > 16:
        return "too_many_components"
    if aspect > 20.0:
        return "extreme_aspect_ratio"
    return None


def choose_backend(
    backend: str,
    *,
    backend_cmd: str,
    rgba_path: Path,
    output_mesh: Path,
) -> tuple[bool, str]:
    if backend == "triposr":
        return run_command_template(backend_cmd, input_rgba=rgba_path, output_mesh=output_mesh)
    if backend == "silhouette-extrude":
        rgba = np.array(Image.open(rgba_path).convert("RGBA"))
        return generate_silhouette_mesh(rgba, output_mesh)
    raise ValueError(f"Unknown backend: {backend}")


def process_sample(
    sample: InstanceSample,
    *,
    output_dir: Path,
    num_candidates: int,
    target_size: int,
    crop_padding: float,
    jitter_shift: float,
    jitter_scale: float,
    backend: str,
    backend_cmd: str,
    fallback_backend: str | None,
    target_max_faces: int,
    target_native_size: float,
    quality_threshold: float,
    min_faces: int,
    keep_rejected: bool,
    rng: random.Random,
) -> tuple[MeshCandidate | None, dict]:
    best: MeshCandidate | None = None
    sample_meta: dict = {
        "source_image": str(sample.image_path),
        "source_mask_path": str(sample.annotation_path),
        "annotation_format": sample.annotation_format,
        "label_id": sample.label_id,
        "pixel_area": sample.pixel_area,
        "candidates": [],
        "backend": backend,
    }

    with tempfile.TemporaryDirectory(prefix="ref3d_", dir=str(output_dir)) as tmpdir:
        tmp = Path(tmpdir)
        for idx in progress_iter(
            range(num_candidates),
            total=num_candidates,
            desc=f"{sample.sample_id} candidates",
        ):
            dx = rng.uniform(-jitter_shift, jitter_shift)
            dy = rng.uniform(-jitter_shift, jitter_shift)
            scale = rng.uniform(1.0 - jitter_scale, 1.0 + jitter_scale)
            rgba = extract_rgba_crop(
                sample,
                target_size=target_size,
                crop_padding=crop_padding,
                jitter_scale=scale,
                jitter_shift=(dx, dy),
            )
            rgba_path = tmp / f"{sample.sample_id}_cand{idx:02d}.png"
            if sample.annotation_format == "full_image":
                # For plain photo folders with no segmentation, keep RGB input
                # so backends like TripoSR can run their own rembg step.
                Image.fromarray(rgba[:, :, :3], mode="RGB").save(rgba_path)
            else:
                Image.fromarray(rgba, mode="RGBA").save(rgba_path)

            raw_mesh_path = tmp / f"{sample.sample_id}_cand{idx:02d}.glb"
            ok, backend_log = choose_backend(
                backend,
                backend_cmd=backend_cmd,
                rgba_path=rgba_path,
                output_mesh=raw_mesh_path,
            )
            used_backend = backend
            if not ok and fallback_backend:
                ok, backend_log = choose_backend(
                    fallback_backend,
                    backend_cmd=backend_cmd,
                    rgba_path=rgba_path,
                    output_mesh=raw_mesh_path,
                )
                used_backend = fallback_backend

            if not ok or not raw_mesh_path.exists():
                sample_meta["candidates"].append(
                    {
                        "index": idx,
                        "status": "generation_failed",
                        "backend": used_backend,
                        "backend_log": backend_log[-1200:],
                    }
                )
                continue

            mesh = load_mesh(raw_mesh_path)
            if mesh is None:
                sample_meta["candidates"].append(
                    {
                        "index": idx,
                        "status": "load_failed",
                        "backend": used_backend,
                    }
                )
                continue

            mesh = postprocess_mesh(
                mesh,
                target_max_faces=target_max_faces,
                target_native_size=target_native_size,
                rgba_for_color=rgba,
            )
            score, metrics = score_mesh(mesh)
            reject_reason = should_reject(metrics, min_faces=min_faces)
            sample_meta["candidates"].append(
                {
                    "index": idx,
                    "status": "ok" if reject_reason is None else "rejected",
                    "backend": used_backend,
                    "score": score,
                    "reject_reason": reject_reason,
                    **metrics,
                }
            )
            if reject_reason is not None:
                continue

            candidate_path = tmp / f"{sample.sample_id}_cand{idx:02d}_processed.glb"
            mesh.export(str(candidate_path))
            candidate = MeshCandidate(
                mesh_path=candidate_path,
                quality_score=score,
                diagnostics={**metrics, "backend": used_backend},
            )
            if best is None or candidate.quality_score > best.quality_score:
                best = candidate

        if best is None:
            sample_meta["status"] = "rejected_no_valid_candidate"
            return None, sample_meta

        if best.quality_score < quality_threshold:
            sample_meta["status"] = "rejected_low_quality"
            sample_meta["quality_score"] = best.quality_score
            if not keep_rejected:
                return None, sample_meta

        out_name = f"{sample.sample_id}.glb"
        out_path = output_dir / out_name
        best.mesh_path.replace(out_path)
        best.mesh_path = out_path
        sample_meta["status"] = "ok"
        sample_meta["mesh_file"] = out_name
        sample_meta["quality_score"] = best.quality_score
        sample_meta.update(best.diagnostics)
        return best, sample_meta


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("input_dir", type=Path, help="Dataset or photo directory")
    p.add_argument("output_dir", type=Path, help="Output directory for generated meshes")
    p.add_argument(
        "--backend",
        choices=["triposr", "silhouette-extrude"],
        default="triposr",
        help="Primary generation backend",
    )
    p.add_argument(
        "--backend-cmd",
        type=str,
        default='triposr --input "{input}" --output "{output}"',
        help=(
            "Command template for --backend triposr. "
            "Supported placeholders: {input}, {output}, {output_dir}, {output_stem}"
        ),
    )
    p.add_argument(
        "--fallback-backend",
        choices=["silhouette-extrude"],
        default="silhouette-extrude",
        help="Fallback backend used when primary generation fails",
    )
    p.add_argument("--max-images", type=int, default=None, help="Limit source images")
    p.add_argument(
        "--max-frames-per-class",
        type=int,
        default=30,
        help=(
            "Maximum source frames to sample per class for YOLO-seg datasets. "
            "Set <= 0 to disable and scan all frames."
        ),
    )
    p.add_argument(
        "--max-instances-per-image",
        type=int,
        default=4,
        help="Max labeled instances to extract per source image",
    )
    p.add_argument(
        "--floor-labels",
        type=int,
        nargs="*",
        default=[1],
        help="Label IDs treated as floor/background distractors to skip",
    )
    p.add_argument("--min-mask-area", type=int, default=1200, help="Minimum instance pixels")
    p.add_argument("--target-size", type=int, default=512, help="Crop resolution for backend")
    p.add_argument("--crop-padding", type=float, default=0.16, help="Relative crop padding")
    p.add_argument("--num-candidates", type=int, default=3, help="Candidates per instance")
    p.add_argument(
        "--full-image-num-candidates",
        type=int,
        default=1,
        help=(
            "Candidates per image when no segmentation is found (plain image folder mode). "
            "Set >1 to evaluate multiple jittered candidates per image."
        ),
    )
    p.add_argument("--jitter-shift", type=float, default=0.04, help="Crop center jitter ratio")
    p.add_argument("--jitter-scale", type=float, default=0.08, help="Crop scale jitter ratio")
    p.add_argument("--target-max-faces", type=int, default=18000, help="Face budget after cleanup")
    p.add_argument("--target-native-size", type=float, default=0.22, help="Longest-side size (m)")
    p.add_argument("--min-faces", type=int, default=700, help="Minimum faces to keep mesh")
    p.add_argument(
        "--quality-threshold",
        type=float,
        default=0.32,
        help="Reject meshes scoring below threshold",
    )
    p.add_argument(
        "--keep-rejected",
        action="store_true",
        help="Keep low-quality best candidate instead of dropping it",
    )
    p.add_argument("--seed", type=int, default=42, help="RNG seed for jitter sampling")
    return p


def main() -> None:
    args = build_arg_parser().parse_args()
    rng = random.Random(args.seed)

    input_dir = args.input_dir.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    floor_labels = set(int(v) for v in args.floor_labels)
    max_frames_per_class = (
        None if int(args.max_frames_per_class) <= 0 else int(args.max_frames_per_class)
    )
    samples = discover_samples(
        input_dir,
        min_mask_area=args.min_mask_area,
        floor_labels=floor_labels,
        max_frames_per_class=max_frames_per_class,
        max_images=args.max_images,
        max_instances_per_image=args.max_instances_per_image,
    )
    if not samples:
        print(f"No valid samples found in {input_dir}")
        return

    print(f"Discovered {len(samples)} samples from {input_dir}")
    manifest: dict[str, dict] = {}
    success = 0

    sample_iter = progress_iter(
        enumerate(samples, start=1),
        total=len(samples),
        desc="Generating reference meshes",
    )
    for i, sample in sample_iter:
        print(f"[{i}/{len(samples)}] {sample.sample_id}")
        sample_num_candidates = (
            int(args.full_image_num_candidates)
            if sample.annotation_format == "full_image"
            else int(args.num_candidates)
        )
        best, meta = process_sample(
            sample,
            output_dir=output_dir,
            num_candidates=sample_num_candidates,
            target_size=args.target_size,
            crop_padding=args.crop_padding,
            jitter_shift=args.jitter_shift,
            jitter_scale=args.jitter_scale,
            backend=args.backend,
            backend_cmd=args.backend_cmd,
            fallback_backend=args.fallback_backend,
            target_max_faces=args.target_max_faces,
            target_native_size=args.target_native_size,
            quality_threshold=args.quality_threshold,
            min_faces=args.min_faces,
            keep_rejected=args.keep_rejected,
            rng=rng,
        )
        manifest[sample.sample_id] = meta
        if best is not None and meta.get("status") == "ok":
            success += 1
            print(
                f"  -> {meta['mesh_file']} (score={meta['quality_score']:.3f}, "
                f"backend={meta['backend']})"
            )
        else:
            print(f"  -> rejected ({meta.get('status')})")

    manifest_path = output_dir / "manifest.json"
    with manifest_path.open("w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)

    print(f"\nGenerated {success}/{len(samples)} meshes")
    print(f"Manifest written to {manifest_path}")


if __name__ == "__main__":
    main()
