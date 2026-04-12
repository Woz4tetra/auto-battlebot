#!/usr/bin/env python3
"""Run SF3D and normalize output mesh path for generate_reference_models.py.

This wrapper keeps the backend contract stable:
  - input image path from {input}
  - output mesh path to {output}

The script runs SF3D's run.py and then picks the best matching generated GLB
from SF3D's output directory, copying it to the exact output path requested by
the caller.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


def _resolve_sf3d_root(explicit: str | None) -> Path:
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit))
    env_root = os.environ.get("SF3D_ROOT")
    if env_root:
        candidates.append(Path(env_root))
    candidates.extend([Path("/opt/sf3d"), Path.home() / "stable-fast-3d"])

    for candidate in candidates:
        run_py = candidate / "run.py"
        if run_py.exists():
            return candidate
    raise FileNotFoundError(
        "Could not find SF3D installation. Set --sf3d-root or SF3D_ROOT."
    )


def _pick_output_glb(output_dir: Path, input_image: Path) -> Path:
    glbs = list(output_dir.rglob("*.glb"))
    if not glbs:
        raise FileNotFoundError(f"No GLB files produced by SF3D in {output_dir}")

    stem = input_image.stem.lower()
    exact = [p for p in glbs if p.stem.lower() == stem]
    if exact:
        return sorted(exact, key=lambda p: p.stat().st_mtime, reverse=True)[0]

    prefix = [p for p in glbs if p.stem.lower().startswith(stem)]
    if prefix:
        return sorted(prefix, key=lambda p: p.stat().st_mtime, reverse=True)[0]

    return sorted(glbs, key=lambda p: p.stat().st_mtime, reverse=True)[0]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="RGBA input image")
    parser.add_argument("--output", required=True, type=Path, help="Target mesh path")
    parser.add_argument(
        "--sf3d-root",
        type=str,
        default=None,
        help="Path to stable-fast-3d checkout (default: auto-detect)",
    )
    parser.add_argument(
        "--texture-resolution",
        type=int,
        default=1024,
        help="Texture resolution passed to SF3D run.py",
    )
    parser.add_argument(
        "--remesh-option",
        type=str,
        default="triangle",
        choices=["none", "triangle", "quad"],
        help="Remesher option passed to SF3D run.py",
    )
    parser.add_argument(
        "--foreground-ratio",
        type=float,
        default=0.9,
        help="Foreground ratio passed to SF3D run.py",
    )
    args = parser.parse_args()

    input_image = args.input.resolve()
    output_path = args.output.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    sf3d_root = _resolve_sf3d_root(args.sf3d_root)
    run_script = sf3d_root / "run.py"

    with tempfile.TemporaryDirectory(prefix="sf3d_wrapper_") as tmpdir:
        tmp_output = Path(tmpdir) / "sf3d_out"
        tmp_output.mkdir(parents=True, exist_ok=True)

        cmd = [
            sys.executable,
            str(run_script),
            str(input_image),
            "--output-dir",
            str(tmp_output),
            "--texture-resolution",
            str(args.texture_resolution),
            "--remesh_option",
            args.remesh_option,
            "--foreground-ratio",
            str(args.foreground_ratio),
        ]

        proc = subprocess.run(
            cmd,
            cwd=str(sf3d_root),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
        if proc.returncode != 0:
            sys.stderr.write(proc.stdout or "")
            return proc.returncode

        produced = _pick_output_glb(tmp_output, input_image)
        shutil.copy2(produced, output_path)
        print(f"SF3D wrapper: wrote {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
