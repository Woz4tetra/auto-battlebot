#!/usr/bin/env python3
"""
Build a flat YOLO-seg dataset from one or more segmentation datasets.

For each image:
- floor_only mode: keep floor polygons minus all non-floor polygons
- merged_classes mode: remap source class IDs into configured class groups

Output is a flat dataset:
- <output>/images/*.jpg|png|...
- <output>/labels/*.txt
- <output>/data.yaml
"""

from __future__ import annotations

import argparse
import shutil
from collections import defaultdict
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml
from shapely.errors import GEOSException
from shapely.geometry import GeometryCollection, LineString, MultiPolygon, Polygon
from shapely.ops import nearest_points, unary_union

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a flat YOLO-seg dataset using configurable transform modes."
    )
    parser.add_argument(
        "--config",
        type=Path,
        help="YAML config file defining mode and default options",
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        help="Input search root containing one or more YOLO datasets (overrides config)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Output directory for flat dataset (overrides config)",
    )
    parser.add_argument(
        "--mode",
        choices=("floor_only", "merged_classes"),
        help="Processing mode (overrides config)",
    )
    parser.add_argument(
        "--floor-class-name",
        help="Class name to treat as floor (case-insensitive, overrides config)",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        default=None,
        help="Delete output directory contents before writing",
    )
    parser.add_argument(
        "--copy-empty",
        action="store_true",
        default=None,
        help="Also copy images with empty post-subtraction floor labels",
    )
    parser.add_argument(
        "--min-area",
        type=float,
        default=None,
        help="Drop polygon fragments with normalized area below this threshold",
    )
    parser.add_argument(
        "--max-polygons",
        type=int,
        default=None,
        help="Keep only this many largest floor polygons per image",
    )
    parser.add_argument(
        "--mask-size",
        type=int,
        default=None,
        help="Raster size used for floor/robot mask subtraction",
    )
    return parser.parse_args()


def load_dataset_yaml(dataset_root: Path) -> dict:
    for yaml_name in ("data.yaml", "data.yml"):
        yaml_path = dataset_root / yaml_name
        if yaml_path.exists():
            with yaml_path.open("r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
    raise FileNotFoundError(f"No data.yaml/data.yml found in {dataset_root}")


def load_runtime_config(config_path: Path | None) -> dict[str, Any]:
    if config_path is None:
        return {}
    with config_path.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    if not isinstance(config, dict):
        raise ValueError("Config root must be a YAML mapping/object")
    return config


def config_get(config: dict[str, Any], key: str, default: Any) -> Any:
    value = config.get(key, default)
    return default if value is None else value


def choose_arg_or_config(cli_value: Any, cfg_value: Any, default: Any) -> Any:
    if cli_value is not None:
        return cli_value
    if cfg_value is not None:
        return cfg_value
    return default


def normalize_label(label: str) -> str:
    return label.strip().lower().replace("-", " ").replace("_", " ")


def parse_merged_class_rules(
    mode_cfg: dict[str, Any],
) -> tuple[list[str], dict[str, str], str | None]:
    output_names = mode_cfg.get("output_names")
    if not isinstance(output_names, list) or not output_names:
        raise ValueError(
            "merged_classes config must define non-empty 'output_names' list"
        )

    groups = mode_cfg.get("groups", {})
    if not isinstance(groups, dict) or not groups:
        raise ValueError("merged_classes config must define non-empty 'groups' mapping")

    output_names_str = [str(name) for name in output_names]
    output_set = set(output_names_str)
    alias_to_output: dict[str, str] = {}

    for output_name, aliases in groups.items():
        output_name_str = str(output_name)
        if output_name_str not in output_set:
            raise ValueError(
                f"Group '{output_name_str}' is not present in output_names {output_names_str}"
            )
        if not isinstance(aliases, list) or not aliases:
            raise ValueError(
                f"Group '{output_name_str}' must contain at least one alias class name"
            )
        for alias in aliases:
            norm_alias = normalize_label(str(alias))
            if norm_alias in alias_to_output and alias_to_output[norm_alias] != output_name_str:
                raise ValueError(
                    f"Alias '{alias}' maps to multiple outputs: "
                    f"{alias_to_output[norm_alias]} and {output_name_str}"
                )
            alias_to_output[norm_alias] = output_name_str

    remaining_to_group = mode_cfg.get("remaining_to_group")
    if remaining_to_group is not None:
        remaining_to_group = str(remaining_to_group)
        if remaining_to_group not in output_set:
            raise ValueError(
                f"remaining_to_group '{remaining_to_group}' is not present in output_names {output_names_str}"
            )

    return output_names_str, alias_to_output, remaining_to_group


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


def as_valid_polygons(points: list[tuple[float, float]]) -> list[Polygon]:
    try:
        polygon = Polygon(points)
    except Exception:
        return []

    if polygon.is_empty:
        return []

    if not polygon.is_valid:
        polygon = polygon.buffer(0)

    if polygon.is_empty:
        return []

    if isinstance(polygon, Polygon):
        return [polygon]
    if isinstance(polygon, MultiPolygon):
        return [p for p in polygon.geoms if not p.is_empty]
    if isinstance(polygon, GeometryCollection):
        polys: list[Polygon] = []
        for geom in polygon.geoms:
            if isinstance(geom, Polygon) and not geom.is_empty:
                polys.append(geom)
            elif isinstance(geom, MultiPolygon):
                polys.extend([p for p in geom.geoms if not p.is_empty])
        return polys

    return []


def repair_geometry(geometry: object) -> object:
    if geometry is None:
        return None
    try:
        if getattr(geometry, "is_empty", True):
            return geometry
        if not getattr(geometry, "is_valid", True):
            return geometry.buffer(0)
        return geometry
    except GEOSException:
        return None


def robust_unary_union(polygons: list[Polygon]) -> object:
    if not polygons:
        return GeometryCollection()
    try:
        return unary_union(polygons)
    except GEOSException:
        repaired: list[Polygon] = []
        for poly in polygons:
            fixed = repair_geometry(poly)
            if isinstance(fixed, Polygon) and not fixed.is_empty:
                repaired.append(fixed)
            elif isinstance(fixed, MultiPolygon):
                repaired.extend([p for p in fixed.geoms if not p.is_empty])
        if not repaired:
            return GeometryCollection()
        return unary_union(repaired)


def robust_difference(floor_geom: object, obstacle_geom: object) -> object:
    try:
        return floor_geom.difference(obstacle_geom)
    except GEOSException:
        pass

    fixed_floor = repair_geometry(floor_geom)
    fixed_obstacles = repair_geometry(obstacle_geom)
    if fixed_floor is None or fixed_obstacles is None:
        return GeometryCollection()

    try:
        return fixed_floor.difference(fixed_obstacles)
    except GEOSException:
        # Last-resort: subtract obstacle polygons one-by-one from each floor polygon.
        obstacle_polys = explode_to_polygons(fixed_obstacles)
        result_pieces: list[Polygon] = []
        for floor_poly in explode_to_polygons(fixed_floor):
            piece: object = floor_poly
            for obs_poly in obstacle_polys:
                try:
                    piece = piece.difference(obs_poly)
                except GEOSException:
                    continue
            result_pieces.extend(explode_to_polygons(piece))
        return robust_unary_union(result_pieces)


def decompose_polygon_without_holes(
    polygon: Polygon,
    min_area: float,
) -> list[Polygon]:
    """Convert polygons-with-holes into hole-free pieces using narrow slit cuts."""
    if polygon.is_empty:
        return []
    if not polygon.interiors:
        return [polygon] if polygon.area >= min_area else []

    # Build tiny cuts from each hole to the exterior boundary. Removing those
    # slits turns holey polygons into regular polygons without fan triangulation.
    cutters = []
    epsilon = 1e-4
    for ring in polygon.interiors:
        try:
            hole_poly = Polygon(ring)
        except Exception:
            continue
        if hole_poly.is_empty:
            continue
        try:
            shell_pt, hole_pt = nearest_points(polygon.exterior, hole_poly.exterior)
            cut_line = LineString([shell_pt, hole_pt])
            if cut_line.length <= 0.0:
                continue
            cutters.append(cut_line.buffer(epsilon, cap_style=2))
        except GEOSException:
            continue

    if not cutters:
        return [polygon] if polygon.area >= min_area else []

    try:
        split_geom = polygon.difference(unary_union(cutters))
    except GEOSException:
        return []

    pieces: list[Polygon] = []
    for piece in explode_to_polygons(split_geom):
        if piece.is_empty:
            continue
        if not piece.is_valid:
            piece = piece.buffer(0)
        if piece.is_empty or not isinstance(piece, Polygon):
            continue
        if piece.area < min_area:
            continue
        pieces.append(piece)
    return pieces


def normalize_result_polygons(result: object, min_area: float) -> list[Polygon]:
    normalized: list[Polygon] = []
    for poly in explode_to_polygons(result):
        if poly.is_empty:
            continue
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty or not isinstance(poly, Polygon):
            continue
        if poly.area < min_area:
            continue
        normalized.extend(decompose_polygon_without_holes(poly, min_area=min_area))
    return normalized


def gather_polygons(
    rows: list[tuple[int, list[tuple[float, float]]]],
    floor_class_id: int,
) -> tuple[list[Polygon], list[Polygon]]:
    floor_polys: list[Polygon] = []
    obstacle_polys: list[Polygon] = []

    for class_id, points in rows:
        polys = as_valid_polygons(points)
        if not polys:
            continue
        if class_id == floor_class_id:
            floor_polys.extend(polys)
        else:
            obstacle_polys.extend(polys)

    return floor_polys, obstacle_polys


def explode_to_polygons(geometry: object) -> list[Polygon]:
    if geometry is None:
        return []
    if isinstance(geometry, Polygon):
        return [geometry]
    if isinstance(geometry, MultiPolygon):
        return list(geometry.geoms)
    if isinstance(geometry, GeometryCollection):
        polys: list[Polygon] = []
        for geom in geometry.geoms:
            polys.extend(explode_to_polygons(geom))
        return polys
    return []


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def to_yolo_seg_line(class_id: int, polygon: Polygon) -> str | None:
    coords = list(polygon.exterior.coords)
    if len(coords) < 4:
        return None
    # Last coord repeats first; YOLO polygon rows should not include duplicate closure.
    coords = coords[:-1]
    if len(coords) < 3:
        return None

    values: list[str] = []
    for x, y in coords:
        values.append(f"{clamp01(float(x)):.6f}")
        values.append(f"{clamp01(float(y)):.6f}")

    return " ".join([str(class_id), *values])


def row_points_to_yolo_line(class_id: int, points: list[tuple[float, float]]) -> str | None:
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
    mask_size: int,
) -> list[str]:
    floor_polys, obstacle_polys = gather_polygons(rows, floor_class_id=floor_class_id)
    if not floor_polys or mask_size < 8 or max_polygons <= 0:
        return []

    max_idx = mask_size - 1
    floor_mask = np.zeros((mask_size, mask_size), dtype=np.uint8)
    obstacle_mask = np.zeros((mask_size, mask_size), dtype=np.uint8)

    def to_cv_poly(poly: Polygon) -> np.ndarray | None:
        coords = list(poly.exterior.coords)
        if len(coords) < 4:
            return None
        pts: list[list[int]] = []
        for x, y in coords[:-1]:
            px = int(round(clamp01(float(x)) * max_idx))
            py = int(round(clamp01(float(y)) * max_idx))
            pts.append([px, py])
        if len(pts) < 3:
            return None
        return np.asarray(pts, dtype=np.int32).reshape(-1, 1, 2)

    def to_cv_poly_points(points: list[tuple[float, float]]) -> np.ndarray | None:
        if len(points) < 3:
            return None
        pts: list[list[int]] = []
        for x, y in points:
            px = int(round(clamp01(float(x)) * max_idx))
            py = int(round(clamp01(float(y)) * max_idx))
            pts.append([px, py])
        if len(pts) < 3:
            return None
        return np.asarray(pts, dtype=np.int32).reshape(-1, 1, 2)

    floor_cv = [arr for p in floor_polys if (arr := to_cv_poly(p)) is not None]
    if not floor_cv:
        return []
    cv2.fillPoly(floor_mask, floor_cv, 255)

    # Subtract all non-floor rows as obstacles. We include both repaired shapely
    # polygons and raw rows to avoid dropping invalid robot polygons.
    obstacle_cv = [arr for p in obstacle_polys if (arr := to_cv_poly(p)) is not None]
    for class_id, points in rows:
        if class_id == floor_class_id:
            continue
        arr = to_cv_poly_points(points)
        if arr is not None:
            obstacle_cv.append(arr)
    if obstacle_cv:
        cv2.fillPoly(obstacle_mask, obstacle_cv, 255)

    mask = cv2.bitwise_and(floor_mask, cv2.bitwise_not(obstacle_mask))

    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours or hierarchy is None:
        return []

    # Build polygons with hole hierarchy first, then decompose holes into
    # hole-free polygons so robot cutouts are not filled back in.
    pieces: list[Polygon] = []
    hierarchy_data = hierarchy[0]
    for idx, contour in enumerate(contours):
        # Parent == -1 means outer contour.
        if hierarchy_data[idx][3] != -1:
            continue
        if len(contour) < 3:
            continue

        exterior = [
            (float(pt[0][0]) / float(max_idx), float(pt[0][1]) / float(max_idx))
            for pt in contour
        ]
        if len(exterior) < 3:
            continue

        holes: list[list[tuple[float, float]]] = []
        child_idx = hierarchy_data[idx][2]
        while child_idx != -1:
            child = contours[child_idx]
            if len(child) >= 3:
                hole = [
                    (float(pt[0][0]) / float(max_idx), float(pt[0][1]) / float(max_idx))
                    for pt in child
                ]
                if len(hole) >= 3:
                    holes.append(hole)
            child_idx = hierarchy_data[child_idx][0]

        try:
            outer_poly = Polygon(exterior, holes)
        except Exception:
            continue
        if not outer_poly.is_valid:
            outer_poly = outer_poly.buffer(0)
        if outer_poly.is_empty:
            continue

        for poly in explode_to_polygons(outer_poly):
            if poly.is_empty:
                continue
            if poly.area < min_area:
                continue
            pieces.extend(decompose_polygon_without_holes(poly, min_area=min_area))

    if not pieces:
        return []

    # Keep only largest polygons to control output size.
    pieces = sorted(pieces, key=lambda p: p.area, reverse=True)[:max_polygons]

    out_rows: list[str] = []
    for poly in pieces:
        line = to_yolo_seg_line(0, poly)
        if line is not None:
            out_rows.append(line)

    return out_rows


def build_class_id_remap(
    dataset_names: list[str],
    output_name_to_id: dict[str, int],
    alias_to_output_name: dict[str, str],
    remaining_to_group: str | None = None,
) -> tuple[dict[int, int], dict[str, list[str]]]:
    id_remap: dict[int, int] = {}
    unmapped_names: list[str] = []

    for old_id, class_name in enumerate(dataset_names):
        norm_name = normalize_label(class_name)
        output_name = alias_to_output_name.get(norm_name)
        if output_name is None:
            if remaining_to_group is not None:
                output_name = remaining_to_group
            else:
                unmapped_names.append(class_name)
                continue
        id_remap[old_id] = output_name_to_id[output_name]

    diagnostics = {"unmapped_names": unmapped_names}
    return id_remap, diagnostics


def build_merged_rows(
    rows: list[tuple[int, list[tuple[float, float]]]],
    id_remap: dict[int, int],
) -> tuple[list[str], int]:
    out_rows: list[str] = []
    dropped_unmapped = 0
    for old_class_id, points in rows:
        new_class_id = id_remap.get(old_class_id)
        if new_class_id is None:
            dropped_unmapped += 1
            continue
        line = row_points_to_yolo_line(new_class_id, points)
        if line is not None:
            out_rows.append(line)
    return out_rows, dropped_unmapped


def flat_stem(dataset_root: Path, images_dir: Path, img_path: Path) -> str:
    rel = img_path.relative_to(images_dir).with_suffix("")
    return "__".join((dataset_root.name, *rel.parts))


def prepare_output(output_dir: Path, overwrite: bool) -> tuple[Path, Path]:
    if output_dir.exists():
        if overwrite:
            shutil.rmtree(output_dir)

    out_images = output_dir / "images"
    out_labels = output_dir / "labels"
    out_images.mkdir(parents=True, exist_ok=True)
    out_labels.mkdir(parents=True, exist_ok=True)
    return out_images, out_labels


def write_output_yaml(output_dir: Path, names: list[str]) -> None:
    yaml_lines = [
        f"path: {output_dir.resolve()}",
        "train: images",
        "val: images",
        f"nc: {len(names)}",
        f"names: {names}",
    ]
    (output_dir / "data.yaml").write_text(
        "\n".join(yaml_lines) + "\n", encoding="utf-8"
    )


def main() -> None:
    args = parse_args()
    try:
        config = load_runtime_config(args.config)
    except (OSError, ValueError) as exc:
        raise SystemExit(f"Failed to load config: {exc}") from exc
    mode = choose_arg_or_config(args.mode, config.get("mode"), "floor_only")
    if mode not in {"floor_only", "merged_classes"}:
        raise SystemExit(f"Unsupported mode '{mode}'. Expected floor_only or merged_classes")

    input_dir_raw = choose_arg_or_config(args.input_dir, config.get("input_dir"), None)
    output_dir_raw = choose_arg_or_config(args.output_dir, config.get("output_dir"), None)
    if input_dir_raw is None or output_dir_raw is None:
        raise SystemExit("Both input_dir and output_dir must be set via CLI or config file")

    input_dir = Path(input_dir_raw)
    output_dir = Path(output_dir_raw)
    overwrite = bool(choose_arg_or_config(args.overwrite, config.get("overwrite"), False))
    copy_empty = bool(choose_arg_or_config(args.copy_empty, config.get("copy_empty"), False))
    min_area = float(choose_arg_or_config(args.min_area, config.get("min_area"), 1e-5))
    max_polygons = int(
        choose_arg_or_config(args.max_polygons, config.get("max_polygons"), 5)
    )
    mask_size = int(choose_arg_or_config(args.mask_size, config.get("mask_size"), 2048))
    output_names: list[str]
    alias_to_output_name: dict[str, str] = {}
    output_name_to_id: dict[str, int] = {}
    remaining_to_group: str | None = None
    modes_cfg = config_get(config, "modes", {})
    if not isinstance(modes_cfg, dict):
        raise SystemExit("Config field 'modes' must be a mapping/object")

    if mode == "floor_only":
        floor_cfg = modes_cfg.get("floor_only", {})
        if not isinstance(floor_cfg, dict):
            raise SystemExit("Config field 'modes.floor_only' must be a mapping/object")
        floor_class_name = choose_arg_or_config(
            args.floor_class_name,
            floor_cfg.get("floor_class_name", config.get("floor_class_name")),
            "floor",
        )
        output_names = [str(floor_cfg.get("output_class_name", "floor"))]
    else:
        merged_cfg = modes_cfg.get("merged_classes", {})
        if not isinstance(merged_cfg, dict):
            raise SystemExit("Config field 'modes.merged_classes' must be a mapping/object")
        output_names, alias_to_output_name, remaining_to_group = parse_merged_class_rules(
            merged_cfg
        )
        output_name_to_id = {name: idx for idx, name in enumerate(output_names)}

    out_images, out_labels = prepare_output(output_dir, overwrite)
    dataset_roots = find_yolo_datasets(input_dir)
    if not dataset_roots:
        raise SystemExit(
            f"No YOLO datasets found under {input_dir}. "
            "Expected dataset roots with data.yaml + images/ + labels/."
        )
    print(f"Found {len(dataset_roots)} dataset(s) under: {input_dir}")
    print(f"Mode: {mode}")

    collision_count = 0
    datasets_used = 0
    datasets_skipped = 0
    total_pairs = 0
    scanned_images = 0
    parse_skips = 0
    wrote_labels = 0
    empty_results = 0
    copied_images = 0
    skipped_existing = 0
    dropped_unmapped_total = 0
    per_class_rows_written: dict[str, int] = defaultdict(int)

    for dataset_root in dataset_roots:
        try:
            dataset_yaml = load_dataset_yaml(dataset_root)
            dataset_names = normalize_names(dataset_yaml.get("names", []))
            floor_class_id = -1
            id_remap: dict[int, int] = {}
            if mode == "floor_only":
                floor_class_id = find_floor_class_id(dataset_yaml, floor_class_name)
            else:
                if not dataset_names:
                    raise ValueError("Dataset YAML does not contain a usable 'names' field")
                id_remap, diagnostics = build_class_id_remap(
                    dataset_names=dataset_names,
                    output_name_to_id=output_name_to_id,
                    alias_to_output_name=alias_to_output_name,
                    remaining_to_group=remaining_to_group,
                )
                unmapped = diagnostics["unmapped_names"]
                if unmapped:
                    print(
                        f"[WARN] {dataset_root}: unmapped dataset classes skipped: {sorted(set(unmapped))}"
                    )
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
        if mode == "floor_only":
            print(
                f"[DATASET] {dataset_root} (pairs={len(pairs)}, floor_id={floor_class_id})"
            )
        else:
            print(f"[DATASET] {dataset_root} (pairs={len(pairs)})")

        images_dir = dataset_root / "images"
        for img_path, label_path in pairs:
            scanned_images += 1

            stem = flat_stem(dataset_root, images_dir, img_path)
            out_img = out_images / f"{stem}{img_path.suffix.lower()}"
            out_lbl = out_labels / f"{stem}.txt"
            if not overwrite and (out_img.exists() or out_lbl.exists()):
                skipped_existing += 1
                continue

            rows = parse_yolo_seg_rows(label_path)
            if not rows:
                parse_skips += 1
                continue

            out_rows: list[str] = []
            if mode == "floor_only":
                out_rows = build_floor_rows(
                    rows,
                    floor_class_id=floor_class_id,
                    min_area=min_area,
                    max_polygons=max_polygons,
                    mask_size=mask_size,
                )
            else:
                out_rows, dropped_unmapped = build_merged_rows(rows, id_remap=id_remap)
                dropped_unmapped_total += dropped_unmapped

            if not out_rows:
                empty_results += 1
                if not copy_empty:
                    continue

            if out_img.exists() or out_lbl.exists():
                collision_count += 1
                stem = f"{stem}_{collision_count}"
                out_img = out_images / f"{stem}{img_path.suffix.lower()}"
                out_lbl = out_labels / f"{stem}.txt"

            shutil.copy2(img_path, out_img)
            copied_images += 1

            if out_rows:
                out_lbl.write_text("\n".join(out_rows) + "\n", encoding="utf-8")
                wrote_labels += 1
                for line in out_rows:
                    parts = line.split(maxsplit=1)
                    if not parts:
                        continue
                    try:
                        class_id = int(parts[0])
                    except ValueError:
                        continue
                    if 0 <= class_id < len(output_names):
                        per_class_rows_written[output_names[class_id]] += 1
            else:
                out_lbl.write_text("", encoding="utf-8")

    if datasets_used == 0:
        raise SystemExit(
            "No usable datasets were processed. "
            "Verify nested datasets contain floor class names and image/label pairs."
        )

    write_output_yaml(output_dir, output_names)

    print(f"Input search dir:   {input_dir}")
    print(f"Output dataset:     {output_dir}")
    print(f"Output classes:     {output_names}")
    if mode == "floor_only":
        print(f"Floor class name:   '{floor_class_name}'")
    print(f"Datasets used:      {datasets_used}")
    print(f"Datasets skipped:   {datasets_skipped}")
    print(f"Image/label pairs:  {total_pairs}")
    print(f"Scanned images:     {scanned_images}")
    print(f"Parse skips:        {parse_skips}")
    print(f"Skipped existing:   {skipped_existing}")
    print(f"Empty output result: {empty_results}")
    print(f"Copied images:      {copied_images}")
    print(f"Wrote label files:  {wrote_labels}")
    if mode == "merged_classes":
        print(f"Dropped unmapped rows: {dropped_unmapped_total}")
    if per_class_rows_written:
        print("Rows written per class:")
        for class_name in output_names:
            print(f"  {class_name}: {per_class_rows_written[class_name]}")
    if collision_count:
        print(f"Resolved collisions: {collision_count}")
    print(f"Wrote data.yaml -> {output_dir / 'data.yaml'}")
    print("Done.")


if __name__ == "__main__":
    main()
