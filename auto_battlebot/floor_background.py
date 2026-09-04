"""Top-down arena floor reconstruction from a recording's pose chain.

Rebuilds `field -> camera` for every processed frame of a recording by composing
`field -> camera_world` (`/tf_static`, one per field init) with `camera_world -> camera`
(`/tf`), warps a spread of frames onto the arena floor into a metric top-down raster, and
takes the per-pixel median. That median is the empty floor: robots move over a fight, so
they average out. A handful of frames is not enough, because robots dwell near the centre
and get baked in.

`background_subtraction_predict.py` uses this to grade background subtraction as if it were
a detector; the depth-gating experiments in `playground/` reuse the same floor.

The poses this reproduces are the ones `export_camera_transforms.py` exported for GT frames,
so background frames are subtracted from live frames in the same geometry.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from auto_battlebot.background_subtraction import build_median_background
from auto_battlebot.camera_geometry import CameraGeometry
from auto_battlebot.mcap_io import (
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
        return np.asarray(self.static_at(stamp) @ camera_world_from_camera)

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


class PoseGeometry:
    """Adapter giving the CameraGeometry attributes from a bare pose matrix and intrinsics."""

    def __init__(self, tf_field_from_camera: np.ndarray, intrinsics: tuple[float, ...]) -> None:
        self.tf_field_from_camera = tf_field_from_camera
        self.fx, self.fy, self.cx, self.cy = intrinsics

    @property
    def camera_position(self) -> np.ndarray:
        return self.tf_field_from_camera[:3, 3]


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
        inset = int(round(margin_m * px_per_m))
        self.floor_mask = np.zeros((self.pixels, self.pixels), np.uint8)
        self.floor_mask[inset : self.pixels - inset, inset : self.pixels - inset] = 255

    @property
    def size(self) -> tuple[int, int]:
        return (self.pixels, self.pixels)

    def image_from_raster(self, geometry: CameraGeometry) -> np.ndarray:
        """Homography taking raster pixels to image pixels.

        Built from the intrinsics and the pose rather than from the projected floor corners.
        Fitting it to four corners needs all four in front of the camera, and when the camera
        leans in over the wall one of them falls behind, which threw the whole frame away.
        Roughly one scored frame in ten was lost that way.

        For a plane point (x, y, 0) the projection is K [r1 r2 t] (x, y, 1), and the raster is
        an affine relabelling of that plane, so the two compose into one homography.
        """
        half = self.size_m / 2.0
        metres_per_px = self.size_m / self.pixels
        raster_to_field = np.array(
            [[metres_per_px, 0.0, -half], [0.0, -metres_per_px, half], [0.0, 0.0, 1.0]]
        )
        camera_from_field = np.linalg.inv(geometry.tf_field_from_camera)
        rotation = camera_from_field[:3, :3]
        translation = camera_from_field[:3, 3]
        intrinsics = np.array(
            [
                [geometry.fx, 0.0, geometry.cx],
                [0.0, geometry.fy, geometry.cy],
                [0.0, 0.0, 1.0],
            ]
        )
        field_to_image = intrinsics @ np.column_stack([rotation[:, 0], rotation[:, 1], translation])
        homography = field_to_image @ raster_to_field

        # Fix the overall sign so a positive homogeneous scale means "in front of the camera",
        # using the field origin, which the camera is always pointed at.
        centre = homography @ np.array([self.pixels / 2.0, self.pixels / 2.0, 1.0])
        return np.asarray(-homography if centre[2] < 0 else homography)

    @staticmethod
    def in_front_mask(homography: np.ndarray, size: tuple[int, int]) -> np.ndarray:
        """Image pixels whose floor point lies in front of the camera, not behind it.

        The floor plane extends past the horizon, and warpPerspective happily samples the part
        behind the camera and paints a mirrored copy of the arena into the sky. Backprojecting
        a pixel gives a homogeneous scale whose sign says which side it came from, and that is
        linear in the pixel, so the test is one half-plane.
        """
        width, height = size
        row = np.linalg.inv(homography)[2]
        xs = np.arange(width, dtype=np.float32)
        ys = np.arange(height, dtype=np.float32)
        scale = row[0] * xs[None, :] + row[1] * ys[:, None] + row[2]
        return np.where(scale > 0, 255, 0).astype(np.uint8)


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
        wanted[frame_index] = raster.image_from_raster(
            PoseGeometry(recording.field_from_camera(pose_index), intrinsics)
        )

    samples: list[np.ndarray] = []
    warps: list[np.ndarray] = []
    masks: list[np.ndarray] = []
    for index, (_topic, _ts, data) in enumerate(
        iter_messages(recording.path, [CAMERA_IMAGE_TOPIC])
    ):
        if index in wanted:
            image = decode_compressed_image(data).image
            homography = wanted[index]
            samples.append(image)
            warps.append(homography)
            # Never fold the mirrored floor from beyond the horizon into the median.
            masks.append(raster.in_front_mask(homography, (image.shape[1], image.shape[0])))
    if not samples:
        raise SystemExit(f"{recording.path}: no usable background samples")
    return build_median_background(samples, warps, raster.size, masks)
