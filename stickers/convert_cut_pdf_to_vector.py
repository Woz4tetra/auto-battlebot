#!/usr/bin/env python3
"""Convert image-based cut PDF content into vector path strokes.

This takes a cut PDF exported from GIMP (often raster-in-PDF), traces the cut
shape, and writes a new PDF containing vector path stroke commands only.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import pikepdf
from pikepdf import Operator, parse_content_stream


Matrix = tuple[float, float, float, float, float, float]
IDENTITY: Matrix = (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)


@dataclass
class PlacedImage:
    xobj: pikepdf.Stream
    matrix: Matrix


def _mul(m1: Matrix, m2: Matrix) -> Matrix:
    a1, b1, c1, d1, e1, f1 = m1
    a2, b2, c2, d2, e2, f2 = m2
    return (
        a1 * a2 + b1 * c2,
        a1 * b2 + b1 * d2,
        c1 * a2 + d1 * c2,
        c1 * b2 + d1 * d2,
        a1 * e2 + c1 * f2 + e1,
        b1 * e2 + d1 * f2 + f1,
    )


def _apply(m: Matrix, x: float, y: float) -> tuple[float, float]:
    a, b, c, d, e, f = m
    return a * x + c * y + e, b * x + d * y + f


def _find_placed_images(
    container: pikepdf.Stream | pikepdf.Dictionary,
    inherited_resources: pikepdf.Dictionary | None,
    start_matrix: Matrix,
) -> list[PlacedImage]:
    if isinstance(container, pikepdf.Stream):
        local_resources = container.get("/Resources")
    elif "/Resources" in container:
        local_resources = container["/Resources"]
    else:
        local_resources = None

    resources = local_resources if local_resources is not None else inherited_resources
    if resources is None or "/XObject" not in resources:
        return []

    placed: list[PlacedImage] = []
    stack: list[Matrix] = []
    ctm = start_matrix

    for ins in parse_content_stream(container):
        op = ins.operator
        if op == Operator("q"):
            stack.append(ctm)
        elif op == Operator("Q"):
            if stack:
                ctm = stack.pop()
        elif op == Operator("cm"):
            m = tuple(float(v) for v in ins.operands)  # type: ignore[assignment]
            ctm = _mul(ctm, m)  # type: ignore[arg-type]
        elif op == Operator("Do"):
            name = ins.operands[0]
            xobj = resources["/XObject"].get(name)
            if not isinstance(xobj, pikepdf.Stream):
                continue
            subtype = xobj.get("/Subtype")
            if subtype == pikepdf.Name("/Image"):
                placed.append(PlacedImage(xobj=xobj, matrix=ctm))
            elif subtype == pikepdf.Name("/Form"):
                placed.extend(_find_placed_images(xobj, resources, ctm))

    return placed


def _build_binary_cut_mask(
    rgba: np.ndarray,
    alpha_threshold: int,
    color_tolerance: int,
) -> np.ndarray:
    alpha = rgba[:, :, 3]
    rgb = rgba[:, :, :3].astype(np.int16)

    flat_rgb = rgb.reshape(-1, 3)
    colors, counts = np.unique(flat_rgb, axis=0, return_counts=True)
    background = colors[np.argmax(counts)]

    diff = np.max(np.abs(rgb - background), axis=2)
    cut_pixels = (alpha > alpha_threshold) & (diff > color_tolerance)
    if not np.any(cut_pixels):
        luma = cv2.cvtColor(rgba[:, :, :3], cv2.COLOR_RGB2GRAY)
        cut_pixels = (alpha > alpha_threshold) & (luma > 0)

    return cut_pixels.astype(np.uint8) * 255


def _vector_paths_for_image(
    placed_image: PlacedImage,
    *,
    alpha_threshold: int,
    color_tolerance: int,
    min_area: float,
    epsilon_px: float,
) -> list[list[tuple[float, float]]]:
    pil_image = pikepdf.PdfImage(placed_image.xobj).as_pil_image().convert("RGBA")
    rgba = np.asarray(pil_image)
    mask = _build_binary_cut_mask(
        rgba,
        alpha_threshold=alpha_threshold,
        color_tolerance=color_tolerance,
    )

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    height, width = mask.shape
    vectors: list[list[tuple[float, float]]] = []
    for contour in contours:
        if cv2.contourArea(contour) < min_area:
            continue
        approx = cv2.approxPolyDP(contour, epsilon=epsilon_px, closed=True)
        if len(approx) < 3:
            continue

        points: list[tuple[float, float]] = []
        for p in approx[:, 0, :]:
            px = (float(p[0]) + 0.5) / float(width)
            py = (float(height - p[1]) - 0.5) / float(height)
            x, y = _apply(placed_image.matrix, px, py)
            points.append((x, y))
        vectors.append(points)

    return vectors


def _page_size(page: pikepdf.Page) -> tuple[float, float]:
    x0, y0, x1, y1 = (float(v) for v in page.obj.MediaBox)
    return x1 - x0, y1 - y0


def convert_cut_pdf_to_vector(
    input_pdf: Path,
    output_pdf: Path,
    *,
    stroke_width_pt: float,
    stroke_rgb: tuple[float, float, float],
    alpha_threshold: int,
    color_tolerance: int,
    min_area: float,
    epsilon_px: float,
) -> int:
    path_count = 0

    with pikepdf.Pdf.open(str(input_pdf)) as in_pdf, pikepdf.Pdf.new() as out_pdf:
        for page in in_pdf.pages:
            width, height = _page_size(page)
            out_page = out_pdf.add_blank_page(page_size=(width, height))
            placed_images = _find_placed_images(
                container=page.obj,
                inherited_resources=page.obj.get("/Resources"),
                start_matrix=IDENTITY,
            )

            commands: list[str] = [
                "q",
                f"{stroke_rgb[0]:.4f} {stroke_rgb[1]:.4f} {stroke_rgb[2]:.4f} RG",
                f"{stroke_width_pt:.4f} w",
                "1 J",
                "1 j",
            ]
            for placed in placed_images:
                vectors = _vector_paths_for_image(
                    placed,
                    alpha_threshold=alpha_threshold,
                    color_tolerance=color_tolerance,
                    min_area=min_area,
                    epsilon_px=epsilon_px,
                )
                for pts in vectors:
                    if not pts:
                        continue
                    x0, y0 = pts[0]
                    commands.append(f"{x0:.3f} {y0:.3f} m")
                    for x, y in pts[1:]:
                        commands.append(f"{x:.3f} {y:.3f} l")
                    commands.append("h")
                    commands.append("S")
                    path_count += 1

            commands.append("Q")
            out_page.obj.Contents = out_pdf.make_stream("\n".join(commands).encode("ascii"))

        output_pdf.parent.mkdir(parents=True, exist_ok=True)
        out_pdf.save(str(output_pdf))

    return path_count


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert raster cut content in a PDF to vector stroke paths."
    )
    parser.add_argument(
        "input_pdf",
        nargs="?",
        default="stickers/Stickers - MBMK3 Top Sticker Cut.pdf",
        help="Input cut PDF path.",
    )
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="Output vector PDF path (default: input name + ' vector').",
    )
    parser.add_argument(
        "--stroke-width-pt",
        type=float,
        default=1.0,
        help="Vector stroke width in points.",
    )
    parser.add_argument(
        "--stroke-r",
        type=float,
        default=1.0,
        help="Red channel for stroke color (0.0-1.0).",
    )
    parser.add_argument(
        "--stroke-g",
        type=float,
        default=0.0,
        help="Green channel for stroke color (0.0-1.0).",
    )
    parser.add_argument(
        "--stroke-b",
        type=float,
        default=0.0,
        help="Blue channel for stroke color (0.0-1.0).",
    )
    parser.add_argument(
        "--alpha-threshold",
        type=int,
        default=10,
        help="Pixels with alpha above this are treated as visible (0-255).",
    )
    parser.add_argument(
        "--color-tolerance",
        type=int,
        default=2,
        help="Difference from dominant background color to treat as cut ink.",
    )
    parser.add_argument(
        "--min-area",
        type=float,
        default=20.0,
        help="Ignore contours below this area in pixel units.",
    )
    parser.add_argument(
        "--epsilon-px",
        type=float,
        default=0.75,
        help="Contour simplification epsilon in pixel units.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_pdf = Path(args.input_pdf)
    if args.output is None:
        output_pdf = input_pdf.with_name(f"{input_pdf.stem} vector.pdf")
    else:
        output_pdf = Path(args.output)

    if not input_pdf.exists():
        print(f"Input cut PDF not found: {input_pdf}")
        return 2

    path_count = convert_cut_pdf_to_vector(
        input_pdf=input_pdf,
        output_pdf=output_pdf,
        stroke_width_pt=args.stroke_width_pt,
        stroke_rgb=(args.stroke_r, args.stroke_g, args.stroke_b),
        alpha_threshold=args.alpha_threshold,
        color_tolerance=args.color_tolerance,
        min_area=args.min_area,
        epsilon_px=args.epsilon_px,
    )
    print(f"Wrote vector cut PDF: {output_pdf} ({path_count} path(s))")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
