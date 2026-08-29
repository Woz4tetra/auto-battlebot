#!/usr/bin/env python3
"""Detect robots by field background subtraction and write predictions for score.py.

This grades background subtraction as if it were a detector. It is not a model: it has no
weights and no classes, it just finds what is on the arena floor that was not there before.
The point of scoring it is to see how far pure geometry plus a difference image gets, and
where it breaks, next to the trained detectors on the same frames.

How a prediction is made, per sub dataset:

1. Read the source recording named in data.yaml and rebuild `field -> camera` for every
   processed frame, by composing `field -> camera_world` (/tf_static, one per field init)
   with `camera_world -> camera` (/tf). This reproduces the poses
   `export_camera_transforms.py` exported for the GT frames exactly, so the background and
   the frames it is subtracted from live in the same geometry.
2. Warp a spread of recording frames onto the arena floor, into a metric top-down raster, and
   take the per-pixel median. Robots move over a fight, so the median is an empty floor. The
   GT frames alone are too few and too clustered for this: robots dwell near the centre and
   end up baked into the background.
3. For each GT frame, warp that floor back into the image through the frame's own pose,
   difference it, and turn the blobs into boxes.

Everything found is emitted as one class, because background subtraction cannot tell a robot
from a house bot from a dropped screw. Only score.py's `agnostic` level says anything real
about it; `archetype` and `instance` measure a classifier that is not there.

Usage:
    python training/model_eval/background_subtraction_predict.py \
        training/data/nhrl_keypoints_eval_test \
        --recordings data/saved_recordings \
        -o training/data/eval_results/bgsub_predictions.json
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml
from camera_geometry import NOMINAL_FIELD_SIZE_M, field_to_pixels, load_frame_geometry

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from auto_battlebot.background_subtraction import (  # noqa: E402
    SubtractionParams,
    build_median_background,
    find_blobs,
    subtract,
    warp_forward,
)
from auto_battlebot.mcap_io import (  # noqa: E402
    decode_compressed_image,
    decode_image_stamp_ns,
    decode_tf_message,
    iter_messages,
)

CAMERA_IMAGE_TOPIC = "/camera/image"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
FIELD_FROM_CAMERA_WORLD = ("field", "camera_world")
CAMERA_WORLD_FROM_CAMERA = ("camera_world", "camera")

# Top-down floor raster resolution. 400 px/m puts the 8 ft arena in a 975 px square, finer
# than the image ever resolves the far edge, so the raster is not the limiting factor.
RASTER_PX_PER_M = 400.0

# Trim the floor square before comparing. The arena wall is not on the floor plane so it never
# warps correctly, and the projected square lands within a few pixels of the wall base.
FLOOR_MARGIN_M = 0.06

# A blob's difference intensity stands in for confidence: mAP needs predictions ranked, and a
# bright difference is the closest thing to a detection score this method has.
SCORE_REFERENCE_DIFF = 110.0


@dataclass
class Recording:
    """Frame stamps and the pose chain from one source recording."""

    path: Path
    image_stamps: np.ndarray  # int64, indexed by SVO frame index
    dynamic: list[tuple[int, np.ndarray]]  # (stamp_ns, camera_world_from_camera)
    static: list[tuple[int, np.ndarray]]  # (stamp_ns, field_from_camera_world)

    def field_from_camera(self, index: int) -> np.ndarray:
        stamp, camera_world_from_camera = self.dynamic[index]
        return self.static_at(stamp) @ camera_world_from_camera

    def static_at(self, stamp_ns: int) -> np.ndarray:
        """The field init in force at a stamp: the last one published at or before it."""
        matrix = self.static[0][1]
        for static_stamp, static_matrix in self.static:
            if static_stamp <= stamp_ns:
                matrix = static_matrix
        return matrix

    def frame_index(self, stamp_ns: int) -> int:
        """The image frame whose interval contains a pipeline stamp.

        Image stamps sit just before the pipeline stamps derived from them, so this joins on
        the frame interval rather than picking the nearest stamp.
        """
        return int(np.searchsorted(self.image_stamps, stamp_ns, "right")) - 1


def read_recording(path: Path) -> Recording:
    image_stamps = np.array(
        [
            decode_image_stamp_ns(data)
            for _topic, _ts, data in iter_messages(path, [CAMERA_IMAGE_TOPIC])
        ],
        dtype=np.int64,
    )
    dynamic: list[tuple[int, np.ndarray]] = []
    static: list[tuple[int, np.ndarray]] = []
    for topic in (TF_TOPIC, TF_STATIC_TOPIC):
        for _topic, _ts, data in iter_messages(path, [topic]):
            for transform in decode_tf_message(data):
                if transform.key == CAMERA_WORLD_FROM_CAMERA:
                    dynamic.append((transform.stamp_ns, transform.matrix))
                elif transform.key == FIELD_FROM_CAMERA_WORLD:
                    static.append((transform.stamp_ns, transform.matrix))
    dynamic.sort(key=lambda item: item[0])
    static.sort(key=lambda item: item[0])
    if not dynamic or not static:
        raise SystemExit(
            f"{path} has no field pose chain (dynamic={len(dynamic)} static={len(static)})"
        )
    return Recording(path=path, image_stamps=image_stamps, dynamic=dynamic, static=static)


def find_recording(source_mcap: str, roots: list[Path]) -> Path:
    """data.yaml records the path on the machine that built the dataset; match by name."""
    name = Path(source_mcap).name
    for root in roots:
        if root.is_file() and root.name == name:
            return root
        matches = sorted(root.rglob(name)) if root.is_dir() else []
        if matches:
            return matches[0]
    raise SystemExit(f"Could not find {name} under {', '.join(str(r) for r in roots)}")


class FloorRaster:
    """A metric top-down view of the arena floor, and the warps in and out of it."""

    def __init__(self, size_m: float, px_per_m: float, margin_m: float) -> None:
        self.size_m = size_m
        self.pixels = int(round(size_m * px_per_m))
        half = size_m / 2.0
        self._corners_field = np.array(
            [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]]
        )
        self._corners_raster = np.array(
            [[0, 0], [self.pixels, 0], [self.pixels, self.pixels], [0, self.pixels]], np.float32
        )
        inset = int(round(margin_m * px_per_m))
        self.floor_mask = np.zeros((self.pixels, self.pixels), np.uint8)
        self.floor_mask[inset : self.pixels - inset, inset : self.pixels - inset] = 255

    @property
    def size(self) -> tuple[int, int]:
        return (self.pixels, self.pixels)

    def image_from_raster(self, geometry: object) -> np.ndarray | None:
        """Homography taking raster pixels to image pixels, or None if the floor is not visible."""
        corners_px = field_to_pixels(self._corners_field, geometry)  # type: ignore[arg-type]
        if not np.all(np.isfinite(corners_px)):
            return None
        return cv2.getPerspectiveTransform(self._corners_raster, corners_px.astype(np.float32))


def sample_frame_indices(recording: Recording, count: int) -> list[int]:
    """Evenly spaced processed frames across the whole recording."""
    total = len(recording.dynamic)
    if total <= count:
        return list(range(total))
    return [int(round(i)) for i in np.linspace(0, total - 1, count)]


def build_floor_background(
    recording: Recording,
    raster: FloorRaster,
    sample_count: int,
    intrinsics: tuple[float, ...],
) -> tuple[np.ndarray, np.ndarray]:
    """Median floor over a spread of recording frames, in the top-down raster."""
    wanted: dict[int, np.ndarray] = {}
    for pose_index in sample_frame_indices(recording, sample_count):
        stamp, _ = recording.dynamic[pose_index]
        frame_index = recording.frame_index(stamp)
        if frame_index < 0 or frame_index >= len(recording.image_stamps):
            continue
        homography = raster.image_from_raster(
            _Geometry(recording.field_from_camera(pose_index), intrinsics)
        )
        if homography is not None:
            wanted[frame_index] = homography

    samples: list[np.ndarray] = []
    warps: list[np.ndarray] = []
    for index, (_topic, _ts, data) in enumerate(
        iter_messages(recording.path, [CAMERA_IMAGE_TOPIC])
    ):
        if index in wanted:
            samples.append(decode_compressed_image(data).image)
            warps.append(wanted[index])
    if not samples:
        raise SystemExit(f"{recording.path}: no usable background samples")
    return build_median_background(samples, warps, raster.size)


class _Geometry:
    """Adapter giving field_to_pixels the attributes it wants, from a bare pose matrix."""

    def __init__(self, tf_field_from_camera: np.ndarray, intrinsics: tuple[float, ...]) -> None:
        self.tf_field_from_camera = tf_field_from_camera
        self.fx, self.fy, self.cx, self.cy = intrinsics

    @property
    def camera_position(self) -> np.ndarray:
        return self.tf_field_from_camera[:3, 3]


GT_BGR = (0, 200, 0)
PRED_BGR = (60, 60, 235)
INK = (245, 245, 245)
SHADOW = (20, 20, 20)
FONT = cv2.FONT_HERSHEY_SIMPLEX


def read_gt_boxes(
    subdataset: Path, stem: str, names: list[str], size: tuple[int, int]
) -> list[tuple]:
    """(x1, y1, x2, y2, label) from a YOLO label file, in pixels."""
    path = subdataset / "labels" / f"{stem}.txt"
    if not path.exists():
        return []
    width, height = size
    rows = []
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        class_id = int(float(parts[0]))
        cx, cy, box_w, box_h = (float(v) for v in parts[1:5])
        rows.append(
            (
                (cx - box_w / 2) * width,
                (cy - box_h / 2) * height,
                (cx + box_w / 2) * width,
                (cy + box_h / 2) * height,
                names[class_id] if class_id < len(names) else f"class_{class_id}",
            )
        )
    return rows


def label_text(
    image: np.ndarray, text: str, origin: tuple[int, int], color: tuple, scale: float = 0.5
) -> None:
    cv2.putText(image, text, origin, FONT, scale, SHADOW, 3, cv2.LINE_AA)
    cv2.putText(image, text, origin, FONT, scale, color, 1, cv2.LINE_AA)


def annotate(
    frame: np.ndarray,
    difference: np.ndarray,
    predictions: list[dict],
    gt_boxes: list[tuple],
    caption: str,
) -> np.ndarray:
    """The frame with the difference laid over it, GT in green and detections in red."""
    canvas = cv2.addWeighted(frame, 1.0, cv2.cvtColor(difference, cv2.COLOR_GRAY2BGR), 0.9, 0.0)

    for x1, y1, x2, y2, name in gt_boxes:
        cv2.rectangle(canvas, (int(x1), int(y1)), (int(x2), int(y2)), GT_BGR, 2)
        label_text(canvas, name, (int(x1), max(12, int(y1) - 5)), GT_BGR)

    for row in predictions:
        x1, y1, x2, y2 = (int(v) for v in row["xyxy"])
        cv2.rectangle(canvas, (x1, y1), (x2, y2), PRED_BGR, 2)
        label_text(
            canvas,
            f"{row['score']:.2f} {row['area']}px",
            (x1, min(canvas.shape[0] - 4, y2 + 14)),
            PRED_BGR,
        )

    label_text(canvas, caption, (10, 22), INK, 0.55)
    label_text(canvas, "green: ground truth", (10, 44), GT_BGR, 0.55)
    label_text(canvas, "red: background subtraction", (10, 64), PRED_BGR, 0.55)
    return canvas


def write_contact_sheet(tiles: list[np.ndarray], path: Path, columns: int = 5) -> None:
    """Grid of downscaled frames, so a whole recording can be scanned at once."""
    if not tiles:
        return
    width, height = 384, 216
    small = [cv2.resize(tile, (width, height)) for tile in tiles]
    rows = []
    for start in range(0, len(small), columns):
        row = small[start : start + columns]
        while len(row) < columns:
            row.append(np.zeros((height, width, 3), np.uint8))
        rows.append(np.hstack(row))
    cv2.imwrite(str(path), np.vstack(rows))


def predict_subdataset(
    subdataset: Path,
    roots: list[Path],
    params: SubtractionParams,
    args: argparse.Namespace,
    reviewed_stems: set[str],
) -> dict[str, list[dict]]:
    data = yaml.safe_load((subdataset / "data.yaml").read_text())
    recording = read_recording(find_recording(str(data["source_mcap"]), roots))

    images = sorted((subdataset / "images").glob("*.png"))
    geometries = {path: load_frame_geometry(path) for path in images}
    usable = [path for path in images if geometries[path] is not None]
    if not usable:
        print(f"  {subdataset.name}: no usable poses, skipped")
        return {}

    first = geometries[usable[0]]
    intrinsics = (first.fx, first.fy, first.cx, first.cy)  # type: ignore[union-attr]
    sample = cv2.imread(str(usable[0]))
    image_size = (sample.shape[1], sample.shape[0])

    raster = FloorRaster(NOMINAL_FIELD_SIZE_M, args.raster_px_per_m, args.floor_margin_m)
    background, coverage = build_floor_background(
        recording, raster, args.background_samples, intrinsics
    )
    compare_mask = cv2.bitwise_and(raster.floor_mask, coverage)

    names = list(data.get("names", []))
    annotate_dir = None
    if args.annotate_dir is not None:
        annotate_dir = Path(args.annotate_dir) / subdataset.name
        annotate_dir.mkdir(parents=True, exist_ok=True)
    tiles: list[np.ndarray] = []

    predictions: dict[str, list[dict]] = {}
    for path in usable:
        geometry = geometries[path]
        homography = raster.image_from_raster(geometry)
        frame = cv2.imread(str(path))
        if homography is None or frame is None:
            predictions[path.stem] = []
            continue

        warped_background = warp_forward(background, homography, image_size)
        valid = warp_forward(compare_mask, homography, image_size, nearest=True)
        valid = cv2.erode(valid, np.ones((11, 11), np.uint8))
        difference, foreground = subtract(frame, warped_background, valid, params)

        rows = []
        for x, y, width, height, area in find_blobs(foreground, params.min_area):
            window = difference[y : y + height, x : x + width]
            spot = foreground[y : y + height, x : x + width] > 0
            strength = float(window[spot].mean()) if spot.any() else 0.0
            rows.append(
                {
                    "xyxy": [float(x), float(y), float(x + width), float(y + height)],
                    "score": round(min(1.0, strength / SCORE_REFERENCE_DIFF), 4),
                    "class_id": 0,
                    "area": area,
                }
            )
        predictions[path.stem] = rows

        if annotate_dir is not None:
            gt_boxes = read_gt_boxes(subdataset, path.stem, names, image_size)
            reviewed = "reviewed" if path.stem in reviewed_stems else "GT not reviewed"
            caption = (
                f"{subdataset.name}  {path.stem}  {len(rows)} det  {len(gt_boxes)} GT  ({reviewed})"
            )
            canvas = annotate(frame, difference, rows, gt_boxes, caption)
            cv2.imwrite(
                str(annotate_dir / f"{path.stem}.jpg"), canvas, [cv2.IMWRITE_JPEG_QUALITY, 92]
            )
            tiles.append(canvas)

    if annotate_dir is not None:
        write_contact_sheet(tiles, annotate_dir.parent / f"contact_{subdataset.name}.jpg")

    found = sum(len(rows) for rows in predictions.values())
    print(f"  {subdataset.name}: {len(usable)}/{len(images)} frames posed, {found} detections")
    return predictions


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("gt", type=Path, help="eval dataset root or a single sub dataset")
    parser.add_argument(
        "--recordings",
        type=Path,
        nargs="+",
        default=[Path("data/saved_recordings")],
        help="where to look for the source MCAPs named in each data.yaml",
    )
    parser.add_argument(
        "-o", "--output", type=Path, required=True, help="predictions JSON to write"
    )
    parser.add_argument(
        "--background-samples", type=int, default=160, help="recording frames in the floor median"
    )
    parser.add_argument("--raster-px-per-m", type=float, default=RASTER_PX_PER_M)
    parser.add_argument("--floor-margin-m", type=float, default=FLOOR_MARGIN_M)
    parser.add_argument("--threshold", type=int, default=35)
    parser.add_argument("--edge-tolerance", type=float, default=0.5)
    parser.add_argument("--min-area", type=int, default=400)
    parser.add_argument("--illumination", choices=("local", "global", "none"), default="local")
    parser.add_argument("--label", default="opponent", help="label to stamp on every detection")
    parser.add_argument(
        "--annotate-dir",
        type=Path,
        default=None,
        help="also write annotated frames (GT vs detections) and per-recording contact sheets here",
    )
    args = parser.parse_args()

    params = SubtractionParams(
        threshold=args.threshold,
        illumination=args.illumination,
        edge_tolerance=args.edge_tolerance,
        min_area=args.min_area,
    )

    root = args.gt
    subdatasets = (
        [root]
        if (root / "data.yaml").exists()
        else sorted(d for d in root.iterdir() if d.is_dir() and (d / "data.yaml").exists())
    )
    if not subdatasets:
        raise SystemExit(f"No data.yaml found under {root}")

    # edit_labels.py marks which frames have been checked. Only those are scored, so the
    # annotations say which ones they are rather than implying every GT box is trustworthy.
    state_path = root / ".edit_state.json"
    reviewed: set[str] = set()
    if state_path.exists():
        reviewed = {
            Path(rel).stem for rel in json.loads(state_path.read_text()).get("reviewed", [])
        }

    frames: dict[str, list[dict]] = {}
    for subdataset in subdatasets:
        frames.update(predict_subdataset(subdataset, list(args.recordings), params, args, reviewed))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(
            {
                "source": "background_subtraction_predict.py",
                "labels": [args.label],
                "params": {
                    "threshold": args.threshold,
                    "edge_tolerance": args.edge_tolerance,
                    "min_area": args.min_area,
                    "illumination": args.illumination,
                    "background_samples": args.background_samples,
                },
                "frames": frames,
            },
            indent=1,
        )
    )
    total = sum(len(rows) for rows in frames.values())
    print(f"Wrote {total} detections over {len(frames)} frames to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
