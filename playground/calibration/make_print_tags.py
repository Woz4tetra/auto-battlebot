"""Generate print-ready AprilTag PDFs for drivetrain calibration: a floor grid and a robot tag.

The PDF export (pixels-per-mm -> DPI, so a 100%-scale print is dimensionally exact) is adapted from
true_battlebot scripts/tags/tag_tools.py. The tags themselves are rendered with cv2.aruco
(DICT_APRILTAG_36h11) so the printed geometry exactly matches the GridBoard model apriltag_track.py uses for
pose. cv2.aruco's ids match the canonical AprilRobotics tag36h11 family (verified), so these are real
AprilTags.

Sizing keeps the ZED resolution in mind. In its fastest mode (VGA 672x376 @ 100 fps) the ZED 2i has coarse
ground sampling, so markers must be physically large to detect from an overhead mount. The script prints the
estimated marker size in pixels at the ZED for the given mount height; aim for >= ~30 px. If your mount is
high and the estimate is small, increase --floor-marker-size / --robot-tag-size, or run the camera at
HD720 @ 60 instead of VGA.

Usage:
    source scripts/activate_python.sh
    python playground/calibration/make_print_tags.py --out-dir playground/calibration/print

Defaults match apriltag_track.py: floor grid 4x3 of 0.16 m markers (ids 10..21), robot tag id 0 at 0.13 m.
Print both at 100% (no "fit to page"), tape the grid flat on the floor and the tag flat on the robot top.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

DICTIONARY = cv2.aruco.DICT_APRILTAG_36h11


def save_pdf(img_gray: np.ndarray, px_per_mm: float, out: Path) -> None:
    """Save a grayscale image as a PDF whose page is dimensionally exact when printed at 100%.

    The tags are pure black/white, so the image is stored bilevel (mode "1"): PDF keeps that lossless
    (no JPEG softening of the marker edges), and the file stays small.
    """
    dpi = px_per_mm * 25.4
    pil = Image.fromarray(img_gray > 127)  # bool array -> mode "1", crisp and lossless
    out.parent.mkdir(parents=True, exist_ok=True)
    pil.save(out, "PDF", resolution=dpi)
    w_mm, h_mm = img_gray.shape[1] / px_per_mm, img_gray.shape[0] / px_per_mm
    print(f"  wrote {out}  ({w_mm:.0f} x {h_mm:.0f} mm, {dpi:.0f} DPI)")


def floor_grid_pdf(
    cols: int, rows: int, marker_m: float, sep_m: float, first_id: int,
    quiet_mm: float, px_per_mm: float, out: Path,
) -> None:
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    ids = np.arange(first_id, first_id + cols * rows)
    board = cv2.aruco.GridBoard((cols, rows), marker_m, sep_m, dictionary, ids)
    board_w_mm = (cols * marker_m + (cols - 1) * sep_m) * 1000.0
    board_h_mm = (rows * marker_m + (rows - 1) * sep_m) * 1000.0
    inner_w = round(board_w_mm * px_per_mm)
    inner_h = round(board_h_mm * px_per_mm)
    margin = round(quiet_mm * px_per_mm)
    img = board.generateImage((inner_w + 2 * margin, inner_h + 2 * margin), marginSize=margin)
    print(f"floor grid: {cols}x{rows} markers, ids {first_id}..{first_id + cols * rows - 1}")
    save_pdf(img, px_per_mm, out)


def robot_tag_pdf(tag_id: int, tag_m: float, quiet_mm: float, px_per_mm: float, out: Path) -> None:
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    side_px = round(tag_m * 1000.0 * px_per_mm)
    marker = dictionary.generateImageMarker(tag_id, side_px)
    margin = round(quiet_mm * px_per_mm)
    img = cv2.copyMakeBorder(marker, margin, margin, margin, margin, cv2.BORDER_CONSTANT, value=255)
    print(f"robot tag: id {tag_id}")
    save_pdf(img, px_per_mm, out)


def zed_px(marker_m: float, mount_m: float, hfov_deg: float, width_px: int) -> float:
    """Estimated marker size in pixels at the camera (ground sampling from an overhead mount)."""
    gsd = (2.0 * mount_m * math.tan(math.radians(hfov_deg) / 2.0)) / width_px  # m / px
    return marker_m / gsd


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out-dir", type=Path, default=Path("playground/calibration/print"))
    p.add_argument("--px-per-mm", type=float, default=8.0, help="print resolution (8 = ~203 DPI)")
    p.add_argument("--quiet-mm", type=float, default=20.0, help="white border around tags/board (mm)")
    # Floor grid (must match apriltag_track.py --floor-* defaults).
    p.add_argument("--floor-cols", type=int, default=4)
    p.add_argument("--floor-rows", type=int, default=3)
    p.add_argument("--floor-marker-size", type=float, default=0.16, help="floor marker edge (m)")
    p.add_argument("--floor-marker-sep", type=float, default=0.05, help="gap between floor markers (m)")
    p.add_argument("--floor-first-id", type=int, default=10)
    # Robot tag.
    p.add_argument("--robot-tag-id", type=int, default=0)
    p.add_argument("--robot-tag-size", type=float, default=0.13, help="robot tag edge (m)")
    # ZED sizing estimate.
    p.add_argument("--mount-height", type=float, default=1.0, help="camera height above the floor (m)")
    p.add_argument("--zed-hfov-deg", type=float, default=110.0, help="ZED 2i wide-lens horizontal FOV")
    p.add_argument("--zed-width", type=int, default=672, help="VGA width in pixels")
    args = p.parse_args()

    floor_grid_pdf(
        args.floor_cols, args.floor_rows, args.floor_marker_size, args.floor_marker_sep,
        args.floor_first_id, args.quiet_mm, args.px_per_mm, args.out_dir / "floor_grid.pdf",
    )
    robot_tag_pdf(
        args.robot_tag_id, args.robot_tag_size, args.quiet_mm, args.px_per_mm,
        args.out_dir / "robot_tag.pdf",
    )

    print(f"\nZED VGA check (mount {args.mount_height:.2f} m, HFOV {args.zed_hfov_deg:.0f} deg, "
          f"{args.zed_width}px):")
    for name, size in (("floor marker", args.floor_marker_size), ("robot tag", args.robot_tag_size)):
        px = zed_px(size, args.mount_height, args.zed_hfov_deg, args.zed_width)
        flag = "ok" if px >= 30 else "LOW - enlarge it, lower the mount, or use HD720@60"
        print(f"  {name} {size * 1000:.0f} mm -> ~{px:.0f} px  [{flag}]")
    print("\nPrint both PDFs at 100% (no fit-to-page). Verify a marker edge measures the stated mm.")


if __name__ == "__main__":
    main()
