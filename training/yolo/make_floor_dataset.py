#!/usr/bin/env python3
"""
Build a floor-only YOLO-seg dataset from one or more segmentation datasets.

For each image:
- Keep polygons belonging to the floor class
- Subtract all non-floor polygons from the floor geometry
- Write resulting polygons as class 0

Output is a flat dataset:
- <output>/images/*.jpg|png|...
- <output>/labels/*.txt
- <output>/data.yaml
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml
from shapely.geometry import GeometryCollection, MultiPolygon, Polygon

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a floor-only flat YOLO-seg dataset"
    )
    parser.add_argument(
        "--input-dir",
        required=True,
        type=Path,
        help="Input search root containing one or more YOLO datasets",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        type=Path,
        help="Output directory for flat dataset",
    )
    parser.add_argument(
        "--floor-class-name",
        default="floor",
        help="Class name to treat as floor (case-insensitive)",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Delete output directory contents before writing",
    )
    parser.add_argument(
        "--copy-empty",
        action="store_true",
        help="Also copy images with empty post-subtraction floor labels",
    )
    parser.add_argument(
        "--min-area",
        type=float,
        default=1e-5,
        help="Drop polygon fragments with normalized area below this threshold",
    )
    parser.add_argument(
        "--max-polygons",
        type=int,
        default=5,
        help="Maximum number of output polygons per image (largest by area)",
    )
    parser.add_argument(
        "--mask-size",
        type=int,
        default=1024,
        help="Minimum raster long-side for subtraction (uses image size if larger)",
    )
    return parser.parse_args()


def load_dataset_yaml(dataset_root: Path) -> dict:
    for yaml_name in ("data.yaml", "data.yml"):
        yaml_path = dataset_root / yaml_name
        if yaml_path.exists():
            with yaml_path.open("r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
    raise FileNotFoundError(f"No data.yaml/data.yml found in {dataset_root}")


def find_yolo_datasets(search_dir: Path) -> list[Path]:
    """Recursively find YOLO dataset roots under search_dir."""
    seen: set[Path] = set()
    datasets: list[Path] = []
    for pattern in ("data.yaml", "data.yml"):
        for yaml_path in sorted(search_dir.rglob(pattern)):
            root = yaml_path.parent
            if root in seen:
                continue
            if (root / "images").exists() and (root / "labels").exists():
                seen.add(root)
                datasets.append(root)
    return sorted(datasets)


def normalize_names(names_value: object) -> list[str]:
    if isinstance(names_value, list):
        return [str(n) for n in names_value]

    if isinstance(names_value, dict):
        resolved: dict[int, str] = {}
        for key, value in names_value.items():
            try:
                idx = int(key)
            except (TypeError, ValueError):
                continue
            resolved[idx] = str(value)
        if not resolved:
            return []
        max_idx = max(resolved)
        return [resolved.get(i, f"class_{i}") for i in range(max_idx + 1)]

    return []


def find_floor_class_id(dataset_yaml: dict, floor_class_name: str) -> int:
    names = normalize_names(dataset_yaml.get("names", []))
    if not names:
        raise ValueError("Dataset YAML does not contain a usable 'names' field")

    target = floor_class_name.strip().lower()
    for class_id, class_name in enumerate(names):
        if class_name.strip().lower() == target:
            return class_id

    raise ValueError(
        f"Floor class '{floor_class_name}' not found in dataset names: {names}"
    )


def iter_image_label_pairs(dataset_root: Path) -> list[tuple[Path, Path]]:
    images_dir = dataset_root / "images"
    labels_dir = dataset_root / "labels"
    if not images_dir.exists() or not labels_dir.exists():
        raise FileNotFoundError(f"Expected {images_dir} and {labels_dir} to both exist")

    pairs: list[tuple[Path, Path]] = []
    for img_path in sorted(images_dir.rglob("*")):
        if img_path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue
        rel = img_path.relative_to(images_dir)
        label_path = (labels_dir / rel).with_suffix(".txt")
        if label_path.exists():
            pairs.append((img_path, label_path))
    return pairs


def parse_yolo_seg_rows(
    label_path: Path,
) -> list[tuple[int, list[tuple[float, float]]]]:
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

        # YOLO-seg: class x1 y1 ... xn yn
        if len(values) < 6 or len(values) % 2 != 0:
            continue

        polygon = [(values[i], values[i + 1]) for i in range(0, len(values), 2)]
        if len(polygon) < 3:
            continue
        rows.append((class_id, polygon))

    return rows


def sanitize_polygon_points(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    sanitized: list[tuple[float, float]] = []
    for point in points:
        if len(point) != 2:
            continue
        x, y = point
        try:
            xf = float(x)
            yf = float(y)
        except (TypeError, ValueError):
            continue
        sanitized.append((max(0.0, min(1.0, xf)), max(0.0, min(1.0, yf))))
    if len(sanitized) < 3:
        return []
    return sanitized


def normalized_polygon_parts(
    points: list[tuple[float, float]],
) -> list[list[tuple[float, float]]]:
    """
    Normalize an input polygon into one or more valid simple polygons.

    This keeps the raster pipeline but reuses the prior shapely-based repair that
    fixed dropped robot masks when labels contained self-intersections or other
    invalid geometries.
    """
    sanitized = sanitize_polygon_points(points)
    if len(sanitized) < 3:
        return []

    try:
        geom: object = Polygon(sanitized)
    except Exception:
        return []

    # The key fix: repair invalid inputs and keep all resulting polygon parts.
    if not geom.is_valid:
        try:
            geom = geom.buffer(0)
        except Exception:
            return []

    parts: list[list[tuple[float, float]]] = []
    candidates: list[Polygon] = []
    if isinstance(geom, Polygon):
        candidates = [geom]
    elif isinstance(geom, MultiPolygon):
        candidates = [p for p in geom.geoms if not p.is_empty]
    elif isinstance(geom, GeometryCollection):
        for sub in geom.geoms:
            if isinstance(sub, Polygon) and not sub.is_empty:
                candidates.append(sub)
            elif isinstance(sub, MultiPolygon):
                candidates.extend([p for p in sub.geoms if not p.is_empty])

    for poly in candidates:
        if poly.is_empty:
            continue
        coords = list(poly.exterior.coords)
        if len(coords) < 4:
            continue
        # Drop repeated closure point.
        ring = [(float(x), float(y)) for x, y in coords[:-1]]
        ring = sanitize_polygon_points(ring)
        if len(ring) >= 3:
            parts.append(ring)

    return parts


def gather_polygons(
    rows: list[tuple[int, list[tuple[float, float]]]],
    floor_class_id: int,
) -> tuple[list[list[tuple[float, float]]], list[list[tuple[float, float]]]]:
    floor_polys: list[list[tuple[float, float]]] = []
    obstacle_polys: list[list[tuple[float, float]]] = []

    for class_id, points in rows:
        parts = normalized_polygon_parts(points)
        if not parts:
            continue
        if class_id == floor_class_id:
            floor_polys.extend(parts)
        else:
            obstacle_polys.extend(parts)

    return floor_polys, obstacle_polys


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def polygon_to_cv_points(
    points: list[tuple[float, float]], mask_width: int, mask_height: int
) -> np.ndarray | None:
    if len(points) < 3:
        return None
    max_x = mask_width - 1
    max_y = mask_height - 1
    arr = []
    for x, y in points:
        px = int(round(clamp01(x) * max_x))
        py = int(round(clamp01(y) * max_y))
        arr.append([px, py])
    if len(arr) < 3:
        return None
    return np.asarray(arr, dtype=np.int32).reshape(-1, 1, 2)


def contour_to_normalized_points(
    contour: np.ndarray, mask_width: int, mask_height: int
) -> list[tuple[float, float]]:
    points = contour.reshape(-1, 2)
    if len(points) < 3:
        return []
    max_x = float(mask_width - 1)
    max_y = float(mask_height - 1)
    return [(float(x) / max_x, float(y) / max_y) for x, y in points]


def resolve_mask_dims(
    image_width: int,
    image_height: int,
    min_long_side: int,
) -> tuple[int, int]:
    """Use image resolution by default, optionally upscaling to min long side."""
    width = max(8, int(image_width))
    height = max(8, int(image_height))
    if min_long_side <= 0:
        return width, height

    long_side = max(width, height)
    if long_side >= min_long_side:
        return width, height

    scale = float(min_long_side) / float(long_side)
    scaled_w = max(8, int(round(width * scale)))
    scaled_h = max(8, int(round(height * scale)))
    return scaled_w, scaled_h


def to_yolo_seg_line(class_id: int, points: list[tuple[float, float]]) -> str | None:
    if len(points) < 3:
        return None

    values: list[str] = []
    for x, y in points:
        values.append(f"{clamp01(float(x)):.6f}")
        values.append(f"{clamp01(float(y)):.6f}")

    return " ".join([str(class_id), *values])


def build_floor_rows(
    rows: list[tuple[int, list[tuple[float, float]]]],
    floor_class_id: int,
    min_area: float,
    max_polygons: int,
    image_width: int,
    image_height: int,
    min_mask_long_side: int,
) -> list[str]:
    if max_polygons <= 0:
        return []
    if image_width < 1 or image_height < 1:
        return []

    floor_polys, obstacle_polys = gather_polygons(rows, floor_class_id=floor_class_id)
    if not floor_polys:
        return []

    mask_width, mask_height = resolve_mask_dims(
        image_width=image_width,
        image_height=image_height,
        min_long_side=min_mask_long_side,
    )
    mask = np.zeros((mask_height, mask_width), dtype=np.uint8)

    floor_cv_polys = [
        cv_poly
        for poly in floor_polys
        if (cv_poly := polygon_to_cv_points(poly, mask_width, mask_height)) is not None
    ]
    if not floor_cv_polys:
        return []
    cv2.fillPoly(mask, floor_cv_polys, 255)

    obstacle_cv_polys = [
        cv_poly
        for poly in obstacle_polys
        if (cv_poly := polygon_to_cv_points(poly, mask_width, mask_height)) is not None
    ]
    if obstacle_cv_polys:
        cv2.fillPoly(mask, obstacle_cv_polys, 0)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []

    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    min_area_px = float(min_area) * float(mask_width * mask_height)

    out_rows: list[str] = []
    for contour in contours[:max_polygons]:
        contour_area = cv2.contourArea(contour)
        if contour_area < min_area_px:
            continue
        norm_points = contour_to_normalized_points(contour, mask_width, mask_height)
        line = to_yolo_seg_line(0, norm_points)
        if line is not None:
            out_rows.append(line)

    return out_rows


def flat_stem(dataset_root: Path, images_dir: Path, img_path: Path) -> str:
    rel = img_path.relative_to(images_dir).with_suffix("")
    return "__".join((dataset_root.name, *rel.parts))


def prepare_output(output_dir: Path, overwrite: bool) -> tuple[Path, Path]:
    if output_dir.exists():
        if overwrite:
            shutil.rmtree(output_dir)
        elif any(output_dir.iterdir()):
            raise FileExistsError(
                f"Output directory {output_dir} is not empty. Use --overwrite."
            )

    out_images = output_dir / "images"
    out_labels = output_dir / "labels"
    out_images.mkdir(parents=True, exist_ok=True)
    out_labels.mkdir(parents=True, exist_ok=True)
    return out_images, out_labels


def write_output_yaml(output_dir: Path) -> None:
    yaml_lines = [
        f"path: {output_dir.resolve()}",
        "train: images",
        "val: images",
        "nc: 1",
        "names: ['floor']",
    ]
    (output_dir / "data.yaml").write_text(
        "\n".join(yaml_lines) + "\n", encoding="utf-8"
    )


def main() -> None:
    args = parse_args()
    out_images, out_labels = prepare_output(args.output_dir, args.overwrite)
    dataset_roots = find_yolo_datasets(args.input_dir)
    if not dataset_roots:
        raise SystemExit(
            f"No YOLO datasets found under {args.input_dir}. "
            "Expected dataset roots with data.yaml + images/ + labels/."
        )
    print(f"Found {len(dataset_roots)} dataset(s) under: {args.input_dir}")

    collision_count = 0
    datasets_used = 0
    datasets_skipped = 0
    total_pairs = 0
    scanned_images = 0
    parse_skips = 0
    wrote_labels = 0
    empty_results = 0
    copied_images = 0

    for dataset_root in dataset_roots:
        try:
            dataset_yaml = load_dataset_yaml(dataset_root)
            floor_class_id = find_floor_class_id(dataset_yaml, args.floor_class_name)
        except (FileNotFoundError, ValueError) as exc:
            print(f"[SKIP] {dataset_root}: {exc}")
            datasets_skipped += 1
            continue

        pairs = iter_image_label_pairs(dataset_root)
        if not pairs:
            print(f"[SKIP] {dataset_root}: no image/label pairs")
            datasets_skipped += 1
            continue

        datasets_used += 1
        total_pairs += len(pairs)
        print(
            f"[DATASET] {dataset_root} "
            f"(pairs={len(pairs)}, floor_id={floor_class_id})"
        )

        images_dir = dataset_root / "images"
        for img_path, label_path in pairs:
            scanned_images += 1

            img = cv2.imread(str(img_path), cv2.IMREAD_UNCHANGED)
            if img is None:
                parse_skips += 1
                continue
            image_height, image_width = img.shape[:2]

            rows = parse_yolo_seg_rows(label_path)
            if not rows:
                parse_skips += 1
                continue

            floor_rows = build_floor_rows(
                rows,
                floor_class_id=floor_class_id,
                min_area=args.min_area,
                max_polygons=args.max_polygons,
                image_width=image_width,
                image_height=image_height,
                min_mask_long_side=args.mask_size,
            )
            if not floor_rows:
                empty_results += 1
                if not args.copy_empty:
                    continue

            stem = flat_stem(dataset_root, images_dir, img_path)
            out_img = out_images / f"{stem}{img_path.suffix.lower()}"
            out_lbl = out_labels / f"{stem}.txt"
            if out_img.exists() or out_lbl.exists():
                collision_count += 1
                stem = f"{stem}_{collision_count}"
                out_img = out_images / f"{stem}{img_path.suffix.lower()}"
                out_lbl = out_labels / f"{stem}.txt"

            shutil.copy2(img_path, out_img)
            copied_images += 1

            if floor_rows:
                out_lbl.write_text("\n".join(floor_rows) + "\n", encoding="utf-8")
                wrote_labels += 1
            else:
                out_lbl.write_text("", encoding="utf-8")

    if datasets_used == 0:
        raise SystemExit(
            "No usable datasets were processed. "
            "Verify nested datasets contain floor class names and image/label pairs."
        )

    write_output_yaml(args.output_dir)

    print(f"Input search dir:   {args.input_dir}")
    print(f"Output dataset:     {args.output_dir}")
    print(f"Floor class name:   '{args.floor_class_name}'")
    print(f"Datasets used:      {datasets_used}")
    print(f"Datasets skipped:   {datasets_skipped}")
    print(f"Image/label pairs:  {total_pairs}")
    print(f"Scanned images:     {scanned_images}")
    print(f"Parse skips:        {parse_skips}")
    print(f"Empty floor result: {empty_results}")
    print(f"Copied images:      {copied_images}")
    print(f"Wrote label files:  {wrote_labels}")
    if collision_count:
        print(f"Resolved collisions: {collision_count}")
    print(f"Wrote data.yaml -> {args.output_dir / 'data.yaml'}")
    print("Done.")


if __name__ == "__main__":
    main()
