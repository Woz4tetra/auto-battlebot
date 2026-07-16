#!/usr/bin/env python3
"""Render the BW-bots logo SVG to a small transparent PNG for the UI top bar.

The PNG is embedded into the app at build time (cmake/bin2c.cmake) and decoded
by LVGL's lodepng decoder. The logo is black/white, so the UI places it on a
white chip; the PNG itself keeps the original colors on a transparent background.

Requires: inkscape on PATH.

Usage:
    scripts/gen_logo_icon.py                # 24 px tall -> assets/bwbots_logo_h24.png
    scripts/gen_logo_icon.py --height 48
"""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ASSETS_DIR = SCRIPT_DIR / "assets"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--height", type=int, default=24, help="output height px (default 24)")
    parser.add_argument("--svg", type=Path, default=ASSETS_DIR / "bwbots_logo.svg")
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    out_path: Path = args.out or ASSETS_DIR / f"bwbots_logo_h{args.height}.png"
    subprocess.run(
        [
            "inkscape",
            str(args.svg),
            "--export-type=png",
            f"--export-filename={out_path}",
            f"--export-height={args.height}",
        ],
        check=True,
        capture_output=True,
    )
    size_kb = out_path.stat().st_size / 1024
    print(f"wrote {out_path} ({size_kb:.1f} KB)")
    return 0


if __name__ == "__main__":
    import sys

    sys.exit(main())
