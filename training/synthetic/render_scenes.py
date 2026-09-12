import blenderproc as bproc  # noqa: F401  # isort: skip  # must be first import for blenderproc run

"""Render synthetic YOLO training scenes with BlenderProc.

Usage (the docker wrapper training/synthetic/docker/run_synthetic.sh starts in
training/synthetic, so the paths below are relative to it):
    blenderproc run render_scenes.py -- config.toml [--num-images N]
        [--images-per-scene N] [--out DIR] [--render-samples N] [--start-index N]
        [--seed N] [-v | -q]

Half the scenes are rendered inside the NHRL cage when ``[cage].enabled`` is set; see
that section of config.toml.

All the actual work lives in the ``synthgen`` package next to this script; this
entry point only parses arguments and hands off to ``synthgen.pipeline.run``.
"""


import argparse
import random
import sys
from pathlib import Path

import numpy as np

# Blender's embedded Python ignores PYTHONPATH (BlenderProc clears it before launching
# Blender), so the two first-party roots this script imports from go on sys.path here, the
# way render_cage_samples.py does it: training/synthetic for `synthgen`, and the repo root for
# `auto_battlebot.perception`, which the cage half reads the camera calibration through.
_SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPT_DIR))
sys.path.insert(0, str(_SCRIPT_DIR.parents[1]))

from synthgen import logsetup  # noqa: E402


def _parse_render_args() -> argparse.Namespace:
    """Parse CLI arguments, supporting the BlenderProc ``--`` argv separator."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path, help="Path to config.toml")
    parser.add_argument("--num-images", type=int, default=None)
    parser.add_argument("--render-samples", type=int, default=64)
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help=(
            "Dataset directory, overriding [output].image_dir/label_dir with"
            " <out>/images and <out>/labels."
        ),
    )
    parser.add_argument(
        "--images-per-scene",
        type=int,
        default=None,
        help="Override [output].images_per_scene (camera viewpoints per arrangement).",
    )
    parser.add_argument(
        "--start-index",
        type=int,
        default=None,
        help=(
            "Starting frame index (for resuming). Defaults to auto-detecting"
            " the next index from existing output files."
        ),
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Seed for Python and numpy RNGs (for reproducible debugging runs).",
    )
    verbosity = parser.add_mutually_exclusive_group()
    verbosity.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Debug logging (per-robot skip detail, asset decisions).",
    )
    verbosity.add_argument(
        "-q",
        "--quiet",
        action="store_true",
        help="Warnings and the run summary only.",
    )
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else sys.argv[1:]
    return parser.parse_args(argv)


def main() -> None:
    """Entry point: configure logging/RNG and run the pipeline."""
    args = _parse_render_args()
    logsetup.configure(verbosity=1 if args.verbose else (-1 if args.quiet else 0))
    if args.seed is not None:
        random.seed(args.seed)
        np.random.seed(args.seed)

    # Deferred import: synthgen.pipeline imports bpy, which only exists inside
    # Blender's Python after the blenderproc import above.
    from synthgen.pipeline import run

    run(args)


if __name__ == "__main__":
    main()
