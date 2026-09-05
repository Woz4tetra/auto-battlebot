"""Render what each candidate input geometry actually hands the detector.

Figure for `input_resolution_plan.md`. The arms differ only in how a 16:9 frame reaches a
fixed-size tensor, and that is hard to argue about in a table: 43.8% padding and "1.5x
smaller robots" are numbers, not pictures. This draws them.

Top row is every arm's input tensor at true pixel scale on one canvas, so a 384x640 tensor
is visibly smaller than a 640x640 one and the grey padding is visible as grey. Bottom row
zooms one annotated robot from each tensor by a common factor with nearest-neighbour
sampling, so the reader sees the pixel budget each geometry spends on the thing being
detected.

    python playground/input_geometry_mosaic.py \
        --frame training/data/nhrl_keypoints_eval_test/<rec>/images/<frame>.png \
        --out docs/experiments/perception_performance/assets/2026-09-05_input_geometry/geo.png
"""

import argparse
from pathlib import Path

import cv2
import numpy as np

from auto_battlebot.trt_yolo import letterbox

PAD_VAL = 114
GAP = 24
LABEL_H = 74
FONT = cv2.FONT_HERSHEY_SIMPLEX
BG = (28, 28, 30)
FG = (236, 236, 236)
DIM = (150, 150, 155)
ACCENT = (90, 200, 250)
BOX_COLOR = (60, 220, 120)

# (label, height, width, mode). `mode` is how the frame is fitted into the tensor.
GEOMETRIES = (
    ("A  640x640 letterbox", 640, 640, "letterbox"),
    ("A2/B  384x640 letterbox", 384, 640, "letterbox"),
    ("C  576x1024 letterbox", 576, 1024, "letterbox"),
    ("D  640x640 stretch", 640, 640, "stretch"),
    ("E  640x640 field crop", 640, 640, "crop"),
)
# Field box measured for the default figure frame by the DeepLab model the arm would use
# (`field_deeplabv3p_r50_2026-07-29`), as normalized (x0, y0, x1, y1). Override with
# --field-box for another frame; without a real box the E panel would flatter the arm.
DEFAULT_FIELD_BOX = (0.0, 0.4913, 0.8314, 1.0)


def read_boxes(label_path: Path) -> list[tuple[float, float, float, float]]:
    """Normalized (cx, cy, w, h) boxes from a YOLO label file, pose columns ignored."""
    if not label_path.exists():
        return []
    boxes = []
    for line in label_path.read_text().splitlines():
        parts = line.split()
        if len(parts) >= 5:
            boxes.append(tuple(float(v) for v in parts[1:5]))  # type: ignore[arg-type]
    return boxes


def to_tensor(
    frame: np.ndarray, height: int, width: int, mode: str, crop: tuple[int, int, int, int] | None
) -> tuple[np.ndarray, np.ndarray]:
    """Return (tensor image, 2x3 affine mapping source pixels into the tensor)."""
    if mode == "stretch":
        out = cv2.resize(frame, (width, height), interpolation=cv2.INTER_LINEAR)
        h, w = frame.shape[:2]
        affine = np.array([[width / w, 0.0, 0.0], [0.0, height / h, 0.0]])
        return out, affine

    source, offset_x, offset_y = frame, 0.0, 0.0
    if mode == "crop" and crop is not None:
        x0, y0, x1, y1 = crop
        source, offset_x, offset_y = frame[y0:y1, x0:x1], float(x0), float(y0)

    out, scale, pad_left, pad_top = letterbox(source, height, width, pad_val=PAD_VAL)
    affine = np.array(
        [[scale, 0.0, pad_left - offset_x * scale], [0.0, scale, pad_top - offset_y * scale]]
    )
    return out, affine


def padding_fraction(source: np.ndarray, tensor: np.ndarray, mode: str) -> float:
    """Share of the tensor filled with grey, given the source the tensor was built from."""
    if mode == "stretch":
        return 0.0
    h, w = source.shape[:2]
    th, tw = tensor.shape[:2]
    scale = min(th / h, tw / w)
    return 1.0 - (round(w * scale) * round(h * scale)) / (th * tw)


def apply_affine(affine: np.ndarray, x: float, y: float) -> tuple[float, float]:
    return (
        affine[0, 0] * x + affine[0, 2],
        affine[1, 1] * y + affine[1, 2],
    )


def draw_label(canvas: np.ndarray, x: int, y: int, lines: list[tuple[str, tuple[int, int, int]]]):
    for i, (text, color) in enumerate(lines):
        scale = 0.62 if i == 0 else 0.5
        thickness = 2 if i == 0 else 1
        cv2.putText(canvas, text, (x, y + 20 + i * 24), FONT, scale, color, thickness, cv2.LINE_AA)


def field_crop_box(
    frame: np.ndarray, field_box: tuple[float, float, float, float], margin: float
) -> tuple[int, int, int, int]:
    """The crop arm E would take: the DeepLab field box grown by `margin` on every side.

    The margin is not decoration. Measured over 250 train and 688 eval frames, a crop to the
    bare field box slices a quarter of the annotated robots; 0.20 is what keeps them whole.
    """
    h, w = frame.shape[:2]
    x0, y0, x1, y1 = field_box
    mx, my = (x1 - x0) * margin, (y1 - y0) * margin
    return (
        int(max(x0 - mx, 0.0) * w),
        int(max(y0 - my, 0.0) * h),
        int(min(x1 + mx, 1.0) * w),
        int(min(y1 + my, 1.0) * h),
    )


def build(
    frame: np.ndarray,
    boxes: list,
    zoom_px: int,
    margin: float,
    field_box: tuple[float, float, float, float],
) -> np.ndarray:
    crop = field_crop_box(frame, field_box, margin)
    h, w = frame.shape[:2]
    panels = [to_tensor(frame, th, tw, mode, crop) for _, th, tw, mode in GEOMETRIES]

    # Biggest annotated robot, used for the zoom row.
    target = max(boxes, key=lambda b: b[2] * b[3]) if boxes else None

    top_h = max(t.shape[0] for t, _ in panels)
    widths = [t.shape[1] for t, _ in panels]
    total_w = sum(widths) + GAP * (len(panels) + 1)
    zoom_row = zoom_px + LABEL_H if target is not None else 0
    canvas = np.full((GAP + LABEL_H + top_h + GAP + zoom_row + GAP, total_w, 3), BG, np.uint8)

    x = GAP
    for (name, _, _, mode), (tensor, affine) in zip(GEOMETRIES, panels):
        source = frame[crop[1] : crop[3], crop[0] : crop[2]] if mode == "crop" else frame
        pad = padding_fraction(source, tensor, mode)
        px = tensor.shape[0] * tensor.shape[1]
        draw_label(
            canvas,
            x,
            GAP,
            [
                (name, FG),
                (f"{tensor.shape[1]}x{tensor.shape[0]} = {px:,} px", DIM),
                (f"{pad * 100:.1f}% padding", ACCENT if pad < 0.1 else DIM),
            ],
        )
        y = GAP + LABEL_H
        canvas[y : y + tensor.shape[0], x : x + tensor.shape[1]] = tensor
        for cx, cy, bw, bh in boxes:
            p0 = apply_affine(affine, (cx - bw / 2) * w, (cy - bh / 2) * h)
            p1 = apply_affine(affine, (cx + bw / 2) * w, (cy + bh / 2) * h)
            cv2.rectangle(
                canvas,
                (x + int(p0[0]), y + int(p0[1])),
                (x + int(p1[0]), y + int(p1[1])),
                BOX_COLOR,
                2,
            )

        if target is not None:
            cx, cy, bw, bh = target
            side = max(bw * w, bh * h) * 1.6
            p = apply_affine(affine, cx * w, cy * h)
            half = side * affine[0, 0] / 2.0
            x0, y0 = int(p[0] - half), int(p[1] - half)
            x1, y1 = int(p[0] + half), int(p[1] + half)
            x0, y0 = max(x0, 0), max(y0, 0)
            x1, y1 = min(x1, tensor.shape[1]), min(y1, tensor.shape[0])
            patch = tensor[y0:y1, x0:x1]
            zy = GAP + LABEL_H + top_h + GAP
            if patch.size:
                shown = cv2.resize(patch, (zoom_px, zoom_px), interpolation=cv2.INTER_NEAREST)
                canvas[zy + LABEL_H : zy + LABEL_H + zoom_px, x : x + zoom_px] = shown
                sqrt_area = float(np.sqrt(bw * w * affine[0, 0] * bh * h * affine[1, 1]))
                draw_label(
                    canvas,
                    x,
                    zy,
                    [
                        (f"robot at {patch.shape[1]}x{patch.shape[0]} real px", FG),
                        (f"sqrt-area {sqrt_area:.0f} px", ACCENT if sqrt_area >= 32 else DIM),
                        ("COCO-small" if sqrt_area < 32 else "", DIM),
                    ],
                )
        x += tensor.shape[1] + GAP
    return canvas


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frame", type=Path, required=True, help="image to render")
    parser.add_argument(
        "--labels", type=Path, default=None, help="defaults to ../labels/<stem>.txt"
    )
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--zoom-px", type=int, default=360)
    parser.add_argument("--crop-margin", type=float, default=0.20)
    parser.add_argument(
        "--field-box",
        type=float,
        nargs=4,
        default=list(DEFAULT_FIELD_BOX),
        metavar=("X0", "Y0", "X1", "Y1"),
        help="normalized DeepLab field box for this frame; the default is measured for the "
        "default figure frame, so pass a fresh one when rendering a different frame",
    )
    args = parser.parse_args()

    frame = cv2.imread(str(args.frame))
    if frame is None:
        raise SystemExit(f"could not read {args.frame}")
    labels = args.labels or args.frame.parent.parent / "labels" / f"{args.frame.stem}.txt"
    canvas = build(
        frame,
        read_boxes(labels),
        args.zoom_px,
        args.crop_margin,
        tuple(args.field_box),  # type: ignore[arg-type]
    )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.out), canvas)
    print(f"wrote {args.out} ({canvas.shape[1]}x{canvas.shape[0]})")


if __name__ == "__main__":
    main()
