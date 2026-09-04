"""Generate a multi-material 3D-print file for the robot's AprilTag (Bambu Lab AMS).

This is the 3D-print counterpart of make_print_tags.py for the tag that mounts on the robot top.
Instead of a paper tag you tape down, it emits a printable plate: a white base with the black
AprilTag cells raised on top as a second material. Two watertight objects share one coordinate
frame, so in Bambu Studio you assign white to the base and black to the tag and slice.

The black geometry comes straight from the cv2.aruco DICT_APRILTAG_36h11 bit pattern, so the printed
tag is geometrically identical to what apriltag_track.py detects (id and orientation verified by re-
detecting a top-down raster before writing). A white margin around the tag gives the quiet zone the
detector needs.

The 3MF is written directly (a zip of XML) so it needs no lxml: two named, pre-colored objects that
import into Bambu Studio already split into a white base and a black tag. STL pairs are written too.

Outputs, written to --out-dir:
    robot_tag.3mf         two objects (white base + black tag), pre-colored
    robot_tag_white.stl   base only
    robot_tag_black.stl   tag only

Usage:
    source scripts/activate_python.sh
    python playground/calibration/make_robot_tag_3d.py --out-dir playground/calibration/print_3d

Default: id 0, 130 mm tag, 2 mm white base, 0.6 mm raised black, 16 mm white margin (a ~1-cell quiet
zone). The plate is 162 x 162 mm, well within a 256 mm Bambu bed. Bambu Studio: import
robot_tag.3mf, confirm the tag object is black filament and the base is white, then slice. Use
0.1-0.2 mm layers and a base thickness that is a clean multiple of the layer height. Note its
physical edge (--tag-size) and heading offset for apriltag_track.py --tag-size / --yaw-offset-deg.
"""

from __future__ import annotations

import argparse
import zipfile
from pathlib import Path

import cv2
import numpy as np
import trimesh

DICTIONARY = cv2.aruco.DICT_APRILTAG_36h11
BORDER_BITS = 1  # AprilTag 36h11: 6 data bits + 1 black border ring per side -> 8 bits across.
WHITE = (235, 235, 235, 255)
BLACK = (20, 20, 20, 255)
CONTENT_TYPES = (
    '<?xml version="1.0" encoding="UTF-8"?>'
    '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
    '<Default Extension="rels" '
    'ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
    '<Default Extension="model" '
    'ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>'
)
RELS = (
    '<?xml version="1.0" encoding="UTF-8"?>'
    '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
    '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
    'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>'
)


def bit_grid(dictionary: cv2.aruco.Dictionary, tag_id: int, side_bits: int) -> np.ndarray:
    """Return a side_bits x side_bits bool array, True where the tag cell is black."""
    img = dictionary.generateImageMarker(tag_id, side_bits, borderBits=BORDER_BITS)
    return img < 128  # row r, col c -> black?


def black_cell_boxes(
    origin: np.ndarray,
    tag_mm: float,
    grid: np.ndarray,
    base_h: float,
    tag_h: float,
) -> list[trimesh.Trimesh]:
    """Boxes for the black cells, raised on top of the base. origin is the tag's (x0, y0) corner in
    mm.

    Consecutive black cells in a row are merged into one box to keep the mesh small and free of
    internal coplanar faces.
    """
    side = grid.shape[0]
    cell = tag_mm / side
    boxes: list[trimesh.Trimesh] = []
    for r in range(side):
        c = 0
        while c < side:
            if not grid[r, c]:
                c += 1
                continue
            run_start = c
            while c < side and grid[r, c]:
                c += 1
            run = c - run_start
            cx = origin[0] + cell * (run_start + run / 2.0)
            # Image row r counts from the top, world +y points up: row 0 must land at the high-y
            # edge so a top- down camera sees the tag, not its vertical mirror. (A reflected 36h11
            # tag never decodes.)
            cy = origin[1] + cell * (side - 1 - r + 0.5)
            box = trimesh.creation.box(extents=[cell * run, cell, tag_h])
            box.apply_translation([cx, cy, base_h + tag_h / 2.0])
            box.visual.face_colors = BLACK
            boxes.append(box)
    return boxes


def base_box(width: float, height: float, base_h: float) -> trimesh.Trimesh:
    box = trimesh.creation.box(extents=[width, height, base_h])
    box.apply_translation([width / 2.0, height / 2.0, base_h / 2.0])
    box.visual.face_colors = WHITE
    return box


def _object_xml(oid: int, mesh: trimesh.Trimesh, pid: int, pindex: int) -> str:
    verts = "".join(f'<vertex x="{x:.4f}" y="{y:.4f}" z="{z:.4f}"/>' for x, y, z in mesh.vertices)
    tris = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in mesh.faces)
    return (
        f'<object id="{oid}" type="model" pid="{pid}" pindex="{pindex}">'
        f"<mesh><vertices>{verts}</vertices><triangles>{tris}</triangles></mesh></object>"
    )


def write_3mf(path: Path, white: trimesh.Trimesh, black: trimesh.Trimesh) -> None:
    """Write a two-object, pre-colored 3MF without lxml (a plain zip of XML)."""
    model = (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<model unit="millimeter" '
        'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
        "<resources>"
        '<basematerials id="1">'
        '<base name="white" displaycolor="#EBEBEBFF"/>'
        '<base name="black" displaycolor="#141414FF"/>'
        "</basematerials>"
        f"{_object_xml(2, white, 1, 0)}"
        f"{_object_xml(3, black, 1, 1)}"
        "</resources>"
        '<build><item objectid="2"/><item objectid="3"/></build>'
        "</model>"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", CONTENT_TYPES)
        z.writestr("_rels/.rels", RELS)
        z.writestr("3D/3dmodel.model", model)


def verify(grid: np.ndarray, tag_id: int, tag_mm: float, ppm: float) -> None:
    """Rasterize the black cells top-down with a quiet zone and re-detect, confirming id and
    orientation.
    """
    side = grid.shape[0]
    cell = tag_mm / side
    pad = tag_mm * 0.25
    n = int(round((tag_mm + 2 * pad) * ppm))
    img = np.full((n, n), 255, np.uint8)
    cp = int(np.ceil(cell * ppm)) + 1
    for r in range(side):
        for c in range(side):
            if not grid[r, c]:
                continue
            px = int(round((pad + c * cell) * ppm))
            py = int(round((pad + r * cell) * ppm))
            img[py : py + cp, px : px + cp] = 0
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICTIONARY))
    _, ids, _ = det.detectMarkers(img)
    found = [int(i) for i in ids.flatten()] if ids is not None else []
    ok = found == [tag_id]
    print(f"verify: detected {found}, expected [{tag_id}], match = {ok}")
    if not ok:
        raise SystemExit("detection mismatch; tag geometry would not track")


def main() -> None:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument(
        "--out-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "print_3d",
        help="output directory for robot_tag.3mf and the STLs (default: alongside this script)",
    )
    p.add_argument("--tag-id", type=int, default=0)
    p.add_argument("--tag-size", type=float, default=0.1, help="AprilTag edge length (m)")
    p.add_argument("--base-height", type=float, default=2.0, help="white base thickness (mm)")
    p.add_argument("--tag-height", type=float, default=0.6, help="black raise above the base (mm)")
    p.add_argument(
        "--margin", type=float, default=10.0, help="white quiet-zone border around the tag (mm)"
    )
    args = p.parse_args()

    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    side_bits = dictionary.markerSize + 2 * BORDER_BITS
    grid = bit_grid(dictionary, args.tag_id, side_bits)
    tag_mm = args.tag_size * 1000.0

    verify(grid, args.tag_id, tag_mm, ppm=8.0)

    plate = tag_mm + 2 * args.margin
    white = base_box(plate, plate, args.base_height)
    origin = np.array([args.margin, args.margin])
    black = trimesh.util.concatenate(
        black_cell_boxes(origin, tag_mm, grid, args.base_height, args.tag_height)
    )
    black.merge_vertices()

    print(
        f"robot tag: id {args.tag_id}, {tag_mm:.0f} mm tag on a {plate:.0f} x {plate:.0f} mm "
        f"plate, "
        f"base {args.base_height:.1f} mm + {args.tag_height:.1f} mm raised black"
    )

    args.out_dir.mkdir(parents=True, exist_ok=True)
    write_3mf(args.out_dir / "robot_tag.3mf", white, black)
    white.export(args.out_dir / "robot_tag_white.stl")
    black.export(args.out_dir / "robot_tag_black.stl")
    print(
        f"\nWrote to {args.out_dir}. Import robot_tag.3mf into Bambu Studio (it imports as a white "
        f"base "
        f"and a black tag), confirm filaments, and slice. Mount it flat on the robot top; pass "
        f"--tag-size {args.tag_size} to apriltag_track.py."
    )


if __name__ == "__main__":
    main()
