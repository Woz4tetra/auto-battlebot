"""Printed-part masses from PrusaSlicer instead of solid volume x a guessed fill factor.

    ~/.local/share/build123d-part/venv/bin/python print_mass.py [part ...]

Slices each part's STL (out/<part>/<part>.stl, written by check.py) with petg_p1s.ini
through the PrusaSlicer flatpak and reads "filament used [g]" from the G-code. Run
check.py on a part first so its STL is current. Settings that change the result live in
petg_p1s.ini: 4 perimeters, 5 top and 4 bottom layers, 25 % gyroid.
"""

import re
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROFILE = HERE / "petg_p1s.ini"
PARTS = ["enclosure", "enclosure_lid", "battery_clamp", "camera_shield"]
SLICER = ["flatpak", "run", "--command=prusa-slicer", "com.prusa3d.PrusaSlicer"]


def slice_grams(stl: Path) -> float:
    # The flatpak can write in $HOME only, not /tmp.
    with tempfile.TemporaryDirectory(dir=Path.home() / ".cache") as tmp:
        gcode = Path(tmp) / "part.gcode"
        cmd = [*SLICER, "--datadir", tmp, "--load", str(PROFILE), "--export-gcode"]
        cmd += ["-o", str(gcode), str(stl)]
        subprocess.run(cmd, check=True, capture_output=True, timeout=600)
        m = re.search(r"filament used \[g\] = ([\d.]+)", gcode.read_text())
    if not m:
        raise RuntimeError(f"no filament mass in the G-code for {stl}")
    return float(m.group(1))


def main(parts: list[str]) -> int:
    total = 0.0
    for name in parts:
        stl = HERE / "out" / name / f"{name}.stl"
        g = slice_grams(stl)
        total += g
        print(f"{name:16s} {g:7.1f} g")
    print(f"{'total':16s} {total:7.1f} g")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:] or PARTS))
