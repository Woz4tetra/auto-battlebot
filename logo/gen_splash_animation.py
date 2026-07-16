#!/usr/bin/env python3
"""Regenerate the BW-bots splash animation at high resolution from the logo SVG.

The original 160x128 bwbots_splash.gif was authored at low resolution. This script
rebuilds the same animation from the vector logo (logo/bwbots_logo.svg) at
the robot UI's screen size (1024x600 by default, full-screen on the Jetson LCD):

  frame 0      black
  frames 1-7   the two half-discs slide in from the screen edges and meet
  frames 8-16  the assembled disc spins one full clockwise turn, decelerating
  frames 17-26 the halves fly apart, overshoot, and settle; the BW pill wipes
               open from the center on top of them (frames 17-20)

The per-frame motion table below was measured from the original GIF via
connected-component analysis (positions normalized by the disc radius). Artwork
proportions come from the SVG, which differs slightly from the hand-made GIF:
the GIF's pill is ~8% larger and its final gap ~30% wider than the vector logo.
The SVG is the true mark, so the final composition uses SVG proportions and the
split trajectory is rescaled to land there.

Rendering: the half-discs are plain semicircles drawn supersampled with PIL (the
SVG halves have notches that are always hidden behind the pill). The pill + "BW"
text is rasterized from the SVG via the inkscape CLI (glyphs are path outlines,
no fonts needed). On the black background the black half is drawn as a white
outline and the white half as plain fill, exactly like the original GIF.

The disc size tracks the output height (same proportion as the reference GIF).
When the canvas is wider than the reference's 5:4 aspect, the slide-in stage is
rescaled so the halves still enter from the physical screen edges instead of
materializing mid-screen.

Verification: every rendered frame is center-cropped to the reference aspect,
downscaled to 160x128, and diffed against the original GIF frame. A contact sheet
(original / rendered / diff) and per-frame mean absolute error are written next to
the output. On canvases wider than 5:4 the slide-in frames legitimately differ
(longer runway), so expect elevated MAE there.

Requires: inkscape on PATH. Run inside the project venv (numpy, Pillow).

Usage:
    logo/gen_splash_animation.py                    # 1024x600 (robot LCD)
    logo/gen_splash_animation.py --size 480x384 --fps-mult 1 --no-verify
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

SCRIPT_DIR = Path(__file__).resolve().parent
ASSETS_DIR = SCRIPT_DIR  # assets live next to this script in logo/

# Reference GIF geometry (the motion table is normalized against these).
REF_SIZE = (160, 128)
REF_RADIUS = 30.0
REF_FRAME_MS = 50
# Hold the assembled logo before the GIF ends/loops so playback doesn't cut off
# abruptly (the UI pauses the gif during this window, then fades to the app).
HOLD_LAST_MS = 500

# Disc radius as a fraction of output height (from the reference GIF: disc
# radius 30 on a 128-tall canvas, disc centered).
RADIUS_FRAC = REF_RADIUS / 128.0

# SVG geometry, in path coordinates of bwbots_logo.svg (before the group's
# 0.2645833 scale). Used to size/position the pill and the final layout.
SVG_DISC_DIAMETER = 273.07  # black half-disc vertical extent
SVG_VIEWBOX_W = 93.0
SVG_GROUP_SCALE = 0.2645833
# Final logo half-gap (flat edge of each half-disc to logo center), fraction of r.
# From the SVG: flats at x=203.5 / 277.2, gap 73.7 units on a 136.5 radius.
FINAL_HALF_GAP = 0.270
# The GIF's final half-gap is wider (10.5 px / 30 px); the measured split offsets
# below are rescaled so the same trajectory settles at the SVG's layout.
GIF_FINAL_HALF_GAP = 0.35
SPLIT_RESCALE = FINAL_HALF_GAP / GIF_FINAL_HALF_GAP

# SVG element ids: pill body and "BW" text outlines (kept for the pill layer),
# everything else (canvas rect, half-discs, outline) is dropped.
PILL_KEEP_IDS = {"path20", "path24"}

# Per-frame motion, measured from bwbots_splash_160x128.gif.
#   left_off / right_off: signed offset of each half-disc's flat edge from the
#     disc center, in units of disc radius (left half sits left of its flat edge).
#   angle: clockwise rotation of the assembled disc, degrees (0 = white half right).
#   pill: revealed width fraction of the pill (center-out wipe).
# fmt: off
KEYFRAMES: list[tuple[float, float, float, float]] = [
    # left    right   angle  pill
    (-2.750,  2.720,    0.0, 0.0),   # 0: fully offscreen (black frame)
    (-2.550,  2.483,    0.0, 0.0),   # 1
    (-2.383,  2.283,    0.0, 0.0),   # 2
    (-2.183,  2.117,    0.0, 0.0),   # 3
    (-1.650,  1.583,    0.0, 0.0),   # 4
    (-1.117,  1.083,    0.0, 0.0),   # 5
    (-0.417,  0.350,    0.0, 0.0),   # 6
    ( 0.000,  0.000,    0.0, 0.0),   # 7: halves meet
    ( 0.000,  0.000,   45.0, 0.0),   # 8
    ( 0.000,  0.000,  120.0, 0.0),   # 9
    ( 0.000,  0.000,  195.0, 0.0),   # 10
    ( 0.000,  0.000,  250.0, 0.0),   # 11
    ( 0.000,  0.000,  296.0, 0.0),   # 12
    ( 0.000,  0.000,  315.0, 0.0),   # 13
    ( 0.000,  0.000,  331.0, 0.0),   # 14
    ( 0.000,  0.000,  345.0, 0.0),   # 15
    ( 0.000,  0.000,  360.0, 0.0),   # 16: spin complete
    (-0.083,  0.050,  360.0, 0.096),  # 17: split starts, pill wipe starts
    (-0.350,  0.283,  360.0, 0.385),  # 18
    (-0.583,  0.583,  360.0, 0.692),  # 19
    (-0.883,  0.817,  360.0, 1.0),   # 20: pill fully revealed
    (-1.017,  0.950,  360.0, 1.0),   # 21
    (-1.083,  1.017,  360.0, 1.0),   # 22: overshoot peak
    (-0.583,  0.550,  360.0, 1.0),   # 23
    (-0.417,  0.383,  360.0, 1.0),   # 24
    (-0.350,  0.283,  360.0, 1.0),   # 25: settled (= final layout)
    (-0.350,  0.283,  360.0, 1.0),   # 26
]
# fmt: on

SUPERSAMPLE = 3


def render_pill_layer(svg_path: Path, radius_px: float) -> Image.Image:
    """Rasterize only the pill + BW text from the SVG, scaled so the logo's
    half-discs would have the given radius. Returns an RGBA image cropped to
    the pill's bounding box."""
    tree = ET.parse(svg_path)
    root = tree.getroot()
    parents = {child: parent for parent in root.iter() for child in parent}
    for elem in list(root.iter()):
        elem_id = elem.get("id", "")
        if elem.tag.endswith("}path") and elem_id not in PILL_KEEP_IDS:
            parents[elem].remove(elem)
        elif "clip-path" in elem.attrib:
            del elem.attrib["clip-path"]  # references an empty <defs>, drop it

    disc_diameter_vb = SVG_DISC_DIAMETER * SVG_GROUP_SCALE
    export_w = int(round(SVG_VIEWBOX_W * 2.0 * radius_px / disc_diameter_vb))

    with tempfile.TemporaryDirectory() as tmp:
        pill_svg = Path(tmp) / "pill.svg"
        pill_png = Path(tmp) / "pill.png"
        tree.write(pill_svg)
        subprocess.run(
            [
                "inkscape",
                str(pill_svg),
                "--export-type=png",
                f"--export-filename={pill_png}",
                f"--export-width={export_w}",
            ],
            check=True,
            capture_output=True,
        )
        layer = Image.open(pill_png).convert("RGBA")

    alpha = np.asarray(layer)[:, :, 3]
    ys, xs = np.nonzero(alpha > 8)
    return layer.crop((int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1))


class HalfDiscSprites:
    """Supersampled sprites for the two half-discs and the assembled disc.

    Each sprite is a square RGBA image of side `size` whose center is the disc
    center; the flat edges of both halves pass through that center."""

    def __init__(self, radius_ss: int, stroke_ss: int) -> None:
        self.size = 2 * (radius_ss + stroke_ss) + 1
        c = self.size // 2
        r = radius_ss
        box = (c - r, c - r, c + r, c + r)
        white = (255, 255, 255, 255)

        # Right half: plain white fill (the logo's white half).
        self.filled = Image.new("RGBA", (self.size, self.size), (0, 0, 0, 0))
        ImageDraw.Draw(self.filled).pieslice(box, -90, 90, fill=white)

        # Left half: white outline only (the logo's black half, on black bg).
        self.outline = Image.new("RGBA", (self.size, self.size), (0, 0, 0, 0))
        draw = ImageDraw.Draw(self.outline)
        draw.arc(box, 90, 270, fill=white, width=stroke_ss)
        draw.line((c, c - r, c, c + r), fill=white, width=stroke_ss)

        self.disc = Image.alpha_composite(self.outline, self.filled)


def render_frame(
    left_off: float,
    right_off: float,
    angle: float,
    pill_frac: float,
    canvas_ss: tuple[int, int],
    radius_ss: int,
    sprites: HalfDiscSprites,
    pill_ss: Image.Image,
    background: tuple[int, int, int, int] = (0, 0, 0, 255),
) -> Image.Image:
    frame = Image.new("RGBA", canvas_ss, background)
    cx, cy = canvas_ss[0] // 2, canvas_ss[1] // 2
    half = sprites.size // 2

    if left_off == 0.0 and right_off == 0.0 and 0.0 < angle < 360.0:
        disc = sprites.disc.rotate(-angle, resample=Image.Resampling.BICUBIC)
        frame.alpha_composite(disc, (cx - half, cy - half))
    else:
        lx = cx + int(round(left_off * radius_ss))
        rx = cx + int(round(right_off * radius_ss))
        frame.alpha_composite(sprites.outline, (lx - half, cy - half))
        frame.alpha_composite(sprites.filled, (rx - half, cy - half))

    if pill_frac > 0.0:
        pw, ph = pill_ss.size
        vis = max(1, int(round(pill_frac * pw)))
        x0 = (pw - vis) // 2
        window = pill_ss.crop((x0, 0, x0 + vis, ph))
        frame.alpha_composite(window, (cx - vis // 2, cy - ph // 2))

    return frame


def build_keyframes(canvas_half_width_r: float) -> list[tuple[float, float, float, float]]:
    """Adapt the measured table to the output canvas.

    Slide-in offsets (frames 0-6) are scaled so the halves enter from the actual
    canvas edge (the reference table assumed a 5:4 canvas, 2.667 radii wide). The
    first visible frame keeps the reference's look: the flat edge 0.12 r inside the
    screen edge. Split offsets (frames 17+) are scaled to settle at the SVG layout.
    """
    slide_k = max(1.0, (canvas_half_width_r - 0.12) / abs(KEYFRAMES[1][0]))
    adapted = []
    for idx, (left, right, angle, pill) in enumerate(KEYFRAMES):
        if idx <= 6:
            adapted.append((left * slide_k, right * slide_k, angle, pill))
        elif idx >= 17:
            adapted.append((left * SPLIT_RESCALE, right * SPLIT_RESCALE, angle, pill))
        else:
            adapted.append((left, right, angle, pill))
    return adapted


def interpolate(
    keyframes: list[tuple[float, float, float, float]], t: float
) -> tuple[float, float, float, float]:
    i = min(int(t), len(keyframes) - 1)
    j = min(i + 1, len(keyframes) - 1)
    f = t - i
    a, b = keyframes[i], keyframes[j]
    return tuple(av + (bv - av) * f for av, bv in zip(a, b))  # type: ignore[return-value]


def verify(rendered: list[Image.Image], fps_mult: int, ref_gif: Path, preview_path: Path) -> float:
    ref = Image.open(ref_gif)
    n_ref = getattr(ref, "n_frames", 1)
    cols, pad = 9, 4
    rw, rh = REF_SIZE
    rows = (n_ref + cols - 1) // cols
    sheet = Image.new("L", (cols * (rw + pad), rows * 3 * (rh + pad)), 96)
    errors: list[float] = []
    ref_aspect = REF_SIZE[0] / REF_SIZE[1]
    for i in range(n_ref):
        ref.seek(i)
        orig = np.asarray(ref.convert("L"), dtype=np.int16)
        frame = rendered[i * fps_mult]
        # Center-crop to the reference aspect so wider canvases compare sanely.
        crop_w = min(frame.width, int(round(frame.height * ref_aspect)))
        crop_h = min(frame.height, int(round(frame.width / ref_aspect)))
        x0, y0 = (frame.width - crop_w) // 2, (frame.height - crop_h) // 2
        cropped = frame.crop((x0, y0, x0 + crop_w, y0 + crop_h))
        ours_img = cropped.resize(REF_SIZE, Image.Resampling.LANCZOS)
        ours = np.asarray(ours_img, dtype=np.int16)
        diff = np.abs(orig - ours)
        errors.append(float(diff.mean()))
        x = (i % cols) * (rw + pad)
        y = (i // cols) * 3 * (rh + pad)
        sheet.paste(Image.fromarray(orig.astype(np.uint8)), (x, y))
        sheet.paste(ours_img, (x, y + rh + pad))
        sheet.paste(Image.fromarray(diff.astype(np.uint8)), (x, y + 2 * (rh + pad)))
    sheet.save(preview_path)

    print("frame :", " ".join(f"{i:5d}" for i in range(n_ref)))
    print("MAE   :", " ".join(f"{e:5.1f}" for e in errors))
    worst = max(errors)
    print(
        f"mean MAE {sum(errors) / len(errors):.2f}, worst {worst:.2f} "
        f"(0-255 grayscale; rows in {preview_path.name}: original / rendered / |diff|)"
    )
    return worst


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--size", default="1024x600", help="output WxH px (default 1024x600, the robot LCD)"
    )
    parser.add_argument(
        "--fps-mult",
        type=int,
        default=2,
        help="frames interpolated per original frame (default 2 -> 25 ms/frame)",
    )
    parser.add_argument("--svg", type=Path, default=ASSETS_DIR / "bwbots_logo.svg")
    parser.add_argument("--ref-gif", type=Path, default=ASSETS_DIR / "bwbots_splash_160x128.gif")
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="output gif (default assets/bwbots_splash_<W>x<H>.gif)",
    )
    parser.add_argument("--no-verify", action="store_true", help="skip diff against the reference")
    args = parser.parse_args()

    try:
        width, height = (int(v) for v in args.size.lower().split("x"))
    except ValueError:
        parser.error(f"--size must be WxH, got '{args.size}'")
    out_path: Path = args.out or ASSETS_DIR / f"bwbots_splash_{width}x{height}.gif"

    radius = RADIUS_FRAC * height
    radius_ss = int(round(radius * SUPERSAMPLE))
    stroke_ss = max(1, int(round(radius_ss / REF_RADIUS)))  # ~1 px at reference scale
    canvas_ss = (width * SUPERSAMPLE, height * SUPERSAMPLE)

    sprites = HalfDiscSprites(radius_ss, stroke_ss)
    pill_ss = render_pill_layer(args.svg, radius * SUPERSAMPLE)
    print(
        f"canvas {width}x{height}, disc r={radius:.1f}px, "
        f"pill {pill_ss.width // SUPERSAMPLE}x{pill_ss.height // SUPERSAMPLE}px"
    )

    keyframes = build_keyframes(width / (2.0 * radius))
    n_out = (len(keyframes) - 1) * args.fps_mult + 1
    frames: list[Image.Image] = []
    for j in range(n_out):
        t = j / args.fps_mult
        params = interpolate(keyframes, t)
        frame_ss = render_frame(*params, canvas_ss, radius_ss, sprites, pill_ss)
        frames.append(frame_ss.convert("L").resize((width, height), Image.Resampling.LANCZOS))

    duration = REF_FRAME_MS // args.fps_mult
    durations = [duration] * len(frames)
    durations[-1] += HOLD_LAST_MS
    frames[0].save(
        out_path,
        save_all=True,
        append_images=frames[1:],
        duration=durations,
        loop=0,
        optimize=True,
    )
    size_kb = out_path.stat().st_size / 1024
    print(
        f"wrote {out_path} ({len(frames)} frames @ {duration} ms, "
        f"{HOLD_LAST_MS} ms final hold, {size_kb:.0f} KB)"
    )

    if not args.no_verify:
        preview = out_path.with_name(out_path.stem + "_verify.png")
        worst = verify(frames, args.fps_mult, args.ref_gif, preview)
        # Expected: MAE 2-5 on spin frames, up to ~22 on split frames (intentional
        # SVG-true pill artwork / gap vs the hand-made GIF) and elevated on slide
        # frames when the canvas is wider than 5:4 (longer slide-in runway).
        if worst > 30.0:
            print("warning: worst-frame error is high; inspect the verify sheet", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
