"""Export the Mr Stabs Mk2 assembly parts to STL for the silkscreen line render (render_robot.py).

~/.local/share/build123d-part/venv/bin/python art/export_robot.py
"""

from pathlib import Path

from build123d import export_stl, import_step

D = Path.home() / "Downloads" / "Mr Stabs Mk2"
OUT = Path(__file__).parent / "robot_stl"
OUT.mkdir(exist_ok=True)
for f in sorted(D.glob("Mr Stabs Mk2 - *.step")):
    name = f.stem.removeprefix("Mr Stabs Mk2 - ")
    if any(
        k in name
        for k in (
            "Screw",
            "Plastite",
            "washer",
            "Washer",
            "Apriltag",
            "Battery",
            "ESC",
            "Clamp Hub",
        )
    ):
        continue  # fasteners and internals: noise at silkscreen scale, render_robot skips them
    shape = import_step(f)
    bb = shape.bounding_box()
    lo, hi = (tuple(round(v, 1) for v in c) for c in (bb.min, bb.max))
    print(f"{name:70s} min {lo} max {hi}")
    export_stl(shape, str(OUT / f"{name}.stl"), tolerance=0.05, angular_tolerance=0.2)
