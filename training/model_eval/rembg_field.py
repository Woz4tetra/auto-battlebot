"""Shared pieces of the rembg floor-saliency experiment.

`rembg` is a salient-object matting network: one image in, one foreground matte out, no
temporal state and no camera model. The experiment asks how much of a robot detector that
gives for free on NHRL footage, in three arms that differ only in what the network sees and
which field mask discards its out-of-field responses afterwards:

- `rembg_warped`: the frame warped into the top-down floor raster, so the field mask is
  intrinsic to the input. Needs a pose.
- `rembg_geom`: the untouched frame, with the nominal arena square projected through the
  pose intersected with the matte after inference. Needs a pose.
- `rembg_deeplab`: the same matte, intersected with the convex hull of the DeepLab floor
  mask instead. Needs no pose, which is what the Jetson would actually run.

Masking after inference rather than before is the design decision that shapes everything
here. Filling outside the field with black hands the network a bright trapezoid on a dark
ground, which is exactly the closed high-contrast region a saliency net is built to return.

This module holds the frame loader, the matte cache, the two field masks and the per-arm
box extraction. `rembg_field_predict.py` tunes and writes predictions for score.py;
`rembg_field_render.py` draws the clips and contact sheets.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from score import reviewed_stems

from auto_battlebot.background_subtraction import warp_forward, warp_inverse
from auto_battlebot.camera_geometry import (
    NOMINAL_FIELD_SIZE_M,
    FrameGeometry,
    load_frame_geometry,
)
from auto_battlebot.floor_background import FLOOR_MARGIN_M, RASTER_PX_PER_M, FloorRaster

SALIENCY_MODEL = "isnet-general-use"
# rembg resizes to this square before inference, whatever the frame size. Confirmed against
# rembg 2.0.83: DisSession.predict normalizes to (1024, 1024).
SALIENCY_INPUT_PX = 1024

# Every detection is one class; the label matches background_subtraction_predict.py so
# score.py's unknown-label check stays quiet.
LABEL = "opponent"

# The first frames of each recording, in stamp order, are the tuning set and are held out.
TUNING_FRAMES_PER_RECORDING = 10

DEEPLAB_CHECKPOINT = Path("data/models/field_deeplabv3p_r50_2026-07-29.pth")
FLOOR_CLASS = 1

ARMS = ("rembg_warped", "rembg_geom", "rembg_deeplab")
POST_HOC_ARMS = ("rembg_geom", "rembg_deeplab")

Box = tuple[float, float, float, float]


@dataclass
class FrameRef:
    """One eval image and everything needed to run any arm on it."""

    subdataset: Path
    stem: str
    image_path: Path
    geometry: FrameGeometry | None
    size: tuple[int, int]  # (width, height)
    tuning: bool

    @property
    def stamp_ns(self) -> int:
        return int(self.stem)

    @property
    def recording(self) -> str:
        return self.subdataset.name

    @property
    def posed(self) -> bool:
        return self.geometry is not None


def _image_size(path: Path) -> tuple[int, int]:
    with Image.open(path) as image:
        return image.size


def enumerate_frames(
    root: Path, tuning_per_recording: int = TUNING_FRAMES_PER_RECORDING
) -> list[FrameRef]:
    """Every validated frame of the eval set, in recording then stamp order.

    Resolution varies between recordings (one is 1080p, the rest 720p), so the size is read
    per frame and nothing downstream may assume one. Pose filtering is left to the caller:
    `rembg_deeplab` can run on unposed frames even though the main table excludes them."""
    accepted = reviewed_stems(root)
    frames: list[FrameRef] = []
    for subdataset in sorted(d for d in root.iterdir() if (d / "data.yaml").exists()):
        stems = sorted(
            (p.stem for p in (subdataset / "images").glob("*.png")),
            key=int,
        )
        if accepted is not None:
            stems = [s for s in stems if s in accepted]
        for index, stem in enumerate(stems):
            image_path = subdataset / "images" / f"{stem}.png"
            frames.append(
                FrameRef(
                    subdataset=subdataset,
                    stem=stem,
                    image_path=image_path,
                    geometry=load_frame_geometry(image_path),
                    size=_image_size(image_path),
                    tuning=index < tuning_per_recording,
                )
            )
    return frames


def scored_frames(frames: list[FrameRef]) -> list[FrameRef]:
    """The frames every arm is graded on: posed and not in the tuning set."""
    return [f for f in frames if f.posed and not f.tuning]


def tuning_frames(frames: list[FrameRef]) -> list[FrameRef]:
    return [f for f in frames if f.posed and f.tuning]


class Saliency:
    """One rembg session, with per-input-size timing.

    `import torch` before the session is deliberate: onnxruntime-gpu dlopens libcudnn.so.9
    by soname, and the venv only has it inside torch's bundled nvidia wheels. Once torch has
    loaded it the CUDA provider resolves; without it rembg silently falls back to CPU."""

    def __init__(self, model: str = SALIENCY_MODEL) -> None:
        import torch  # noqa: F401
        from rembg import new_session, remove

        self._remove = remove
        self._session = new_session(model)
        self.providers: list[str] = list(self._session.inner_session.get_providers())
        self.timings: dict[tuple[int, int], list[float]] = {}

    def matte(self, bgr: np.ndarray) -> np.ndarray:
        """uint8 alpha at the frame's own size, straight from the network with no matting."""
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        started = time.perf_counter()
        result = self._remove(Image.fromarray(rgb), session=self._session, only_mask=True)
        elapsed = time.perf_counter() - started
        self.timings.setdefault((bgr.shape[1], bgr.shape[0]), []).append(elapsed)
        return np.asarray(result.convert("L"), dtype=np.uint8)

    def timing_summary(self) -> dict[str, dict[str, float]]:
        """Median and count per input size, dropping the first call (warm-up) of each."""
        summary = {}
        for (width, height), times in self.timings.items():
            steady = times[1:] if len(times) > 1 else times
            summary[f"{width}x{height}"] = {
                "median_ms": float(np.median(steady) * 1000.0),
                "p90_ms": float(np.percentile(steady, 90) * 1000.0),
                "count": len(times),
            }
        return summary


class MatteCache:
    """PNG-backed cache of uint8 masks, keyed by kind, recording and stem.

    Mattes are the expensive part and are exactly reproducible, so both tuning and scoring
    read them from here after one pass. The same store holds the DeepLab masks."""

    def __init__(self, root: Path) -> None:
        self.root = root

    def path(self, kind: str, frame: FrameRef) -> Path:
        return self.root / kind / frame.recording / f"{frame.stem}.png"

    def get(self, kind: str, frame: FrameRef) -> np.ndarray | None:
        path = self.path(kind, frame)
        if not path.exists():
            return None
        return cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)

    def put(self, kind: str, frame: FrameRef, mask: np.ndarray) -> None:
        path = self.path(kind, frame)
        path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(path), mask)


def make_raster() -> FloorRaster:
    return FloorRaster(NOMINAL_FIELD_SIZE_M, RASTER_PX_PER_M, FLOOR_MARGIN_M)


def geometric_field_mask(
    raster: FloorRaster, geometry: FrameGeometry, size: tuple[int, int]
) -> np.ndarray:
    """The nominal arena square, inset by the floor margin, projected into the image.

    Warping the raster's floor mask through the homography handles the case where a corner
    of the square falls behind the camera, which a polygon of projected corners does not."""
    homography = raster.image_from_raster(geometry)
    mask = warp_forward(raster.floor_mask, homography, size, nearest=True)
    return cv2.bitwise_and(mask, raster.in_front_mask(homography, size))


def raster_valid_mask(
    raster: FloorRaster, homography: np.ndarray, size: tuple[int, int]
) -> np.ndarray:
    """Raster pixels that are on the floor square, in front of the camera and inside the image."""
    pixels = raster.pixels
    us, vs = np.meshgrid(np.arange(pixels, dtype=np.float64), np.arange(pixels, dtype=np.float64))
    homogeneous = np.stack([us, vs, np.ones_like(us)], axis=-1) @ homography.T
    scale = homogeneous[..., 2]
    in_front = scale > 1e-9
    with np.errstate(divide="ignore", invalid="ignore"):
        x = homogeneous[..., 0] / scale
        y = homogeneous[..., 1] / scale
    width, height = size
    inside = in_front & (x >= 0) & (x < width) & (y >= 0) & (y < height)
    valid = np.where(inside, 255, 0).astype(np.uint8)
    return cv2.bitwise_and(valid, raster.floor_mask)


def warp_to_raster(
    frame: np.ndarray, raster: FloorRaster, homography: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """The frame seen from above, black outside the floor square. Returns (raster, valid)."""
    valid = raster_valid_mask(raster, homography, (frame.shape[1], frame.shape[0]))
    warped = warp_inverse(frame, homography, raster.size)
    warped[valid == 0] = 0
    return warped, valid


class FieldSegmenter:
    """The DeepLab floor model, at its own trained input size, plus the convex-hull fix.

    A robot on the floor occludes the floor, so the raw mask has a hole exactly where each
    robot stands, and masking a detection against it would delete the detections the
    experiment is counting. The hull fills those holes. The true field region is a square in
    perspective and therefore convex, so the hull can only recover floor the mask lost."""

    def __init__(self, checkpoint: Path = DEEPLAB_CHECKPOINT) -> None:
        import torch
        from load_deeplabv3 import common_transforms, load_model

        self._torch = torch
        self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self._model, self.config = load_model(checkpoint, self._device)
        self._transform = common_transforms(pad_size=self.config.pad_size)
        self.timings: list[float] = []

    def raw_mask(self, bgr: np.ndarray) -> np.ndarray:
        """uint8 0/255 floor mask at the frame's size, nearest-upscaled from the model."""
        started = time.perf_counter()
        size = self.config.image_size
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (size, size), interpolation=cv2.INTER_LINEAR)
        tensor = self._transform(resized).unsqueeze(0).to(self._device)
        with self._torch.no_grad():
            output = self._model(tensor)
        pred = self._torch.argmax(output, dim=1).squeeze(0).cpu().numpy()
        pad = self.config.pad_size
        if pad > 0:
            pred = pred[pad:-pad, pad:-pad]
        floor = np.where(pred == FLOOR_CLASS, 255, 0).astype(np.uint8)
        self.timings.append(time.perf_counter() - started)
        return cv2.resize(floor, (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST)


def largest_component_hull(mask: np.ndarray) -> np.ndarray:
    """Convex hull of the largest connected blob, filled. Empty mask in, empty mask out."""
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return np.zeros_like(mask)
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    blob = np.where(labels == largest, 255, 0).astype(np.uint8)
    contours, _ = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    hull = cv2.convexHull(max(contours, key=cv2.contourArea))
    out = np.zeros_like(mask)
    cv2.fillConvexPoly(out, hull, 255)
    return out


@dataclass
class Component:
    """One connected blob of the thresholded matte, with what every arm needs to filter it."""

    box: Box
    area: int
    mean_alpha: float
    overlap: dict[str, float] = field(default_factory=dict)  # field mask name -> fraction

    def row(self) -> dict:
        return {
            "xyxy": [round(v, 1) for v in self.box],
            "score": round(self.mean_alpha / 255.0, 4),
            "class_id": 0,
            "area": self.area,
        }


def matte_components(
    matte: np.ndarray, threshold: int, field_masks: dict[str, np.ndarray] | None = None
) -> list[Component]:
    """Connected components of `matte >= threshold`, with overlap against each field mask.

    No morphology: the matte is the network's own output and the experiment grades it as
    such. `min_area` and the overlap cut are applied by the caller so one component list
    serves the whole tuning grid."""
    binary = np.where(matte >= threshold, 255, 0).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    components = []
    for index in range(1, count):
        x, y, w, h, area = (int(v) for v in stats[index])
        window = labels[y : y + h, x : x + w] == index
        mean_alpha = float(matte[y : y + h, x : x + w][window].mean())
        overlap = {}
        for name, mask in (field_masks or {}).items():
            inside = mask[y : y + h, x : x + w][window] > 0
            overlap[name] = float(inside.mean())
        components.append(
            Component(
                box=(float(x), float(y), float(x + w), float(y + h)),
                area=area,
                mean_alpha=mean_alpha,
                overlap=overlap,
            )
        )
    return components


def post_hoc_select(
    components: list[Component], mask_name: str, min_area: int, overlap: float
) -> list[Component]:
    """Components that survive a field mask: enough of their area lies inside it.

    The box is the whole component, not its in-field part. A robot near the far wall has
    its top above the floor polygon, and clipping the box there would cut every such robot
    at the wall line."""
    return [
        c for c in components if c.area >= min_area and c.overlap.get(mask_name, 0.0) >= overlap
    ]


def raster_box_to_image(box: Box, homography: np.ndarray, size: tuple[int, int]) -> Box | None:
    """Bounding box in the image of a raster-space box's four corners.

    A raster-axis-aligned box is not axis-aligned in the image, so this inflates the box
    slightly; that cost belongs to the warped arm."""
    x1, y1, x2, y2 = box
    corners = np.array([[x1, y1, 1.0], [x2, y1, 1.0], [x2, y2, 1.0], [x1, y2, 1.0]])
    projected = corners @ homography.T
    if np.any(projected[:, 2] <= 1e-9):
        return None
    xs = projected[:, 0] / projected[:, 2]
    ys = projected[:, 1] / projected[:, 2]
    width, height = size
    out = (
        float(np.clip(xs.min(), 0, width)),
        float(np.clip(ys.min(), 0, height)),
        float(np.clip(xs.max(), 0, width)),
        float(np.clip(ys.max(), 0, height)),
    )
    if out[2] - out[0] < 1.0 or out[3] - out[1] < 1.0:
        return None
    return out


def warped_select(
    components: list[Component], homography: np.ndarray, size: tuple[int, int], min_area: int
) -> list[Component]:
    """Raster components above `min_area`, re-boxed in image pixels."""
    selected = []
    for component in components:
        if component.area < min_area:
            continue
        box = raster_box_to_image(component.box, homography, size)
        if box is None:
            continue
        selected.append(Component(box=box, area=component.area, mean_alpha=component.mean_alpha))
    return selected


@dataclass
class ArmParams:
    threshold: int
    min_area: int
    overlap: float | None = None  # post-hoc arms only

    def to_json(self) -> dict:
        return {"threshold": self.threshold, "min_area": self.min_area, "overlap": self.overlap}

    @classmethod
    def from_json(cls, data: dict) -> ArmParams:
        return cls(int(data["threshold"]), int(data["min_area"]), data.get("overlap"))


def load_params(path: Path) -> dict[str, ArmParams]:
    payload = json.loads(path.read_text())
    return {arm: ArmParams.from_json(payload[arm]["chosen"]) for arm in ARMS}


def write_predictions(path: Path, frames: dict[str, list[dict]], meta: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps({"labels": [LABEL], **meta, "frames": frames}, indent=1),
    )


def parallax_px(distance_m: float, height_m: float, camera_height_m: float) -> float:
    """Radial raster displacement of a point `height_m` above the floor, in raster pixels.

    The floor homography is exact only on the plane. A point at height h seen from a camera
    at height H lands `d * h / (H - h)` further from the nadir than its floor footprint."""
    if camera_height_m <= height_m:
        return float("nan")
    return distance_m * height_m / (camera_height_m - height_m) * RASTER_PX_PER_M
