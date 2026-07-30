#!/usr/bin/env python3
"""Render a grid of annotated ground-truth frames from a score.py eval dataset.

Draws the hand-corrected GT boxes and keypoints that `score.py` grades against, one
tile per frame, for use as a figure. Frames are picked per subdataset: only reviewed
frames with at least two labeled robots are eligible, ranked by how large the robots
are on screen, and kept far apart in time so the grid does not repeat one moment.

Usage:
    python training/model_eval/make_dataset_mosaic.py training/data/nhrl_keypoints_eval_test \
        -o docs/experiments/perception_performance/assets/eval_dataset_mosaic.png
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

TILE_W, TILE_H = 640, 360
PAD = 6
LEGEND_H = 46
MIN_SEPARATION_NS = 10_000_000_000  # keep chosen frames >=10 s apart within a recording
FONT = cv2.FONT_HERSHEY_SIMPLEX

# Recording -> opponent. The GT labels opponents as a generic class, so the matchup
# is only knowable per recording. Recordings absent here get no caption.
OPPONENTS = {
    "main_2026-05-01_17-42-20__2026-05-01T17-42-24": "unlabeled opponent",
    "main_2026-05-02_10-06-02_repaired__2026-05-02T10-06-06": "clyde",
    "main_2026-05-02_11-45-05_repaired__2026-05-02T11-45-08": "sphinx",
    "main_2026-05-02_14-12-25_repaired__2026-05-02T14-12-27": "wreckcreation",
    "main_2026-05-02_15-35-00_repaired__2026-05-02T15-35-04": "ironwarrior",
}


def hex_to_bgr(value: str) -> tuple[int, int, int]:
    value = value.lstrip("#")
    r, g, b = (int(value[i : i + 2], 16) for i in (0, 2, 4))
    return (b, g, r)


def parse_label(path: Path, width: float, height: float) -> list[dict]:
    """Parse YOLO detect (5 values) or pose (5 + 3k values) rows into pixel coords."""
    rows = []
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) < 5 or (len(parts) - 5) % 3 != 0:
            continue
        values = [float(x) for x in parts[1:]]
        cx, cy, bw, bh = values[:4]
        rows.append(
            {
                "cid": int(float(parts[0])),
                "box": (
                    (cx - bw / 2) * width,
                    (cy - bh / 2) * height,
                    (cx + bw / 2) * width,
                    (cy + bh / 2) * height,
                ),
                "kps": [
                    (values[i] * width, values[i + 1] * height, values[i + 2])
                    for i in range(4, len(values), 3)
                ],
            }
        )
    return rows


def label_path_for(root: Path, rel: str) -> Path:
    return (root / rel.replace("images", "labels")).with_suffix(".txt")


def draw_tile(
    image_path: Path, label_path: Path, names: list[str], colors: list[str], caption: str
) -> np.ndarray:
    image = cv2.imread(str(image_path))
    if image is None:
        raise SystemExit(f"Could not read {image_path}")
    height, width = image.shape[:2]
    rows = parse_label(label_path, width, height)

    sx, sy = TILE_W / width, TILE_H / height
    tile = cv2.resize(image, (TILE_W, TILE_H), interpolation=cv2.INTER_AREA)

    placed: list[tuple[int, int, int, int]] = []

    def is_free(rect: tuple[int, int, int, int]) -> bool:
        ax1, ay1, ax2, ay2 = rect
        if ay1 < 0 or ay2 > TILE_H or ax2 > TILE_W:
            return False
        return not any(
            ax1 < bx2 and bx1 < ax2 and ay1 < by2 and by1 < ay2 for bx1, by1, bx2, by2 in placed
        )

    # Big boxes first so their labels claim space before small ones crowd in.
    rows.sort(key=lambda r: -(r["box"][2] - r["box"][0]) * (r["box"][3] - r["box"][1]))

    for row in rows:
        color = hex_to_bgr(colors[row["cid"]]) if row["cid"] < len(colors) else (200, 200, 200)
        x1, y1, x2, y2 = (
            int(row["box"][0] * sx),
            int(row["box"][1] * sy),
            int(row["box"][2] * sx),
            int(row["box"][3] * sy),
        )
        cv2.rectangle(tile, (x1, y1), (x2, y2), color, 2)

        # Keypoints: front -> back vector, front filled, back hollow.
        visible = [(int(kx * sx), int(ky * sy)) for kx, ky, v in row["kps"] if v > 0]
        if len(visible) == 2:
            cv2.line(tile, visible[0], visible[1], (255, 255, 255), 2, cv2.LINE_AA)
            cv2.line(tile, visible[0], visible[1], color, 1, cv2.LINE_AA)
            cv2.circle(tile, visible[0], 4, (255, 255, 255), -1, cv2.LINE_AA)
            cv2.circle(tile, visible[0], 3, color, -1, cv2.LINE_AA)
            cv2.circle(tile, visible[1], 4, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.circle(tile, visible[1], 3, color, 1, cv2.LINE_AA)

        name = names[row["cid"]] if row["cid"] < len(names) else f"class_{row['cid']}"
        (tw, th), _ = cv2.getTextSize(name, FONT, 0.38, 1)
        # Above the box, then below, then either shifted to the box's right edge.
        anchors = [
            (x1, y1 - 4),
            (x1, y2 + th + 4),
            (max(0, x2 - tw - 4), y1 - 4),
            (max(0, x2 - tw - 4), y2 + th + 4),
        ]
        lx, ly = anchors[0]
        for ax, ay in anchors:
            if is_free((ax, ay - th - 3, ax + tw + 4, ay + 2)):
                lx, ly = ax, ay
                break
        placed.append((lx, ly - th - 3, lx + tw + 4, ly + 2))
        cv2.rectangle(tile, (lx, ly - th - 3), (lx + tw + 4, ly + 2), color, -1)
        cv2.putText(tile, name, (lx + 2, ly), FONT, 0.38, (255, 255, 255), 1, cv2.LINE_AA)

    if caption:
        (tw, th), _ = cv2.getTextSize(caption, FONT, 0.45, 1)
        cv2.rectangle(tile, (0, TILE_H - th - 10), (tw + 12, TILE_H), (24, 24, 24), -1)
        cv2.putText(tile, caption, (6, TILE_H - 6), FONT, 0.45, (235, 235, 235), 1, cv2.LINE_AA)
    return tile


def reviewed_frames(root: Path) -> list[str]:
    state = root / ".edit_state.json"
    if not state.exists():
        raise SystemExit(f"No .edit_state.json in {root}; nothing marked reviewed")
    return list(json.loads(state.read_text())["reviewed"])


def frame_rank(root: Path, rel: str) -> tuple[int, float]:
    """(#boxes, mean box area as a fraction of the frame) for one reviewed frame."""
    rows = parse_label(label_path_for(root, rel), 1.0, 1.0)  # normalized: area is a fraction
    if not rows:
        return (0, 0.0)
    areas = [(b[2] - b[0]) * (b[3] - b[1]) for b in (r["box"] for r in rows)]
    return (len(rows), float(np.mean(areas)))


def rank_by_recording(
    root: Path, reviewed: list[str]
) -> dict[str, list[tuple[float, int, int, str]]]:
    """Eligible frames per recording, best first: (mean area, #boxes, stamp_ns, rel)."""
    by_recording: dict[str, list[str]] = {}
    for rel in reviewed:
        by_recording.setdefault(rel.split("/")[0], []).append(rel)

    ranked: dict[str, list[tuple[float, int, int, str]]] = {}
    for recording, rels in by_recording.items():
        scored = []
        for rel in rels:
            n_boxes, mean_area = frame_rank(root, rel)
            if n_boxes < 2:
                continue
            scored.append((mean_area, n_boxes, int(Path(rel).stem), rel))
        if scored:
            ranked[recording] = sorted(scored, reverse=True)
    return ranked


def pick_frames(root: Path, reviewed: list[str], count: int) -> list[tuple[str, str]]:
    """Spread `count` frames over the recordings, favoring close-in, multi-robot frames."""
    ranked = rank_by_recording(root, reviewed)
    if not ranked:
        raise SystemExit("No reviewed frames with two or more labeled robots")

    # Round-robin over recordings so every matchup appears before any repeats.
    chosen: list[tuple[str, str]] = []
    taken: dict[str, list[int]] = {r: [] for r in ranked}
    cursor = dict.fromkeys(ranked, 0)
    while len(chosen) < count:
        progressed = False
        for recording, scored in ranked.items():
            if len(chosen) == count:
                break
            while cursor[recording] < len(scored):
                _area, _n, stamp, rel = scored[cursor[recording]]
                cursor[recording] += 1
                if any(abs(stamp - t) < MIN_SEPARATION_NS for t in taken[recording]):
                    continue
                taken[recording].append(stamp)
                chosen.append((recording, rel))
                progressed = True
                break
        if not progressed:
            print(f"Warning: only {len(chosen)} distinct frames available, wanted {count}")
            break
    return chosen


def build_mosaic(root: Path, chosen: list[tuple[str, str]], cols: int, rows_n: int) -> np.ndarray:
    all_names: list[str] = []
    all_colors: list[str] = []
    tiles = []
    for recording, rel in chosen:
        data = yaml.safe_load((root / recording / "data.yaml").read_text())
        names, colors = list(data["names"]), list(data["colors"])
        if len(names) > len(all_names):
            all_names, all_colors = names, colors
        print(f"  {rel}")
        tiles.append(
            draw_tile(
                root / rel,
                label_path_for(root, rel),
                names,
                colors,
                OPPONENTS.get(recording, ""),
            )
        )

    grid_w = cols * TILE_W + (cols + 1) * PAD
    grid_h = rows_n * TILE_H + (rows_n + 1) * PAD + LEGEND_H
    canvas = np.full((grid_h, grid_w, 3), 18, dtype=np.uint8)
    for i, tile in enumerate(tiles):
        r, c = divmod(i, cols)
        y = PAD + r * (TILE_H + PAD)
        x = PAD + c * (TILE_W + PAD)
        canvas[y : y + TILE_H, x : x + TILE_W] = tile

    # Legend strip along the bottom: class colors, then the keypoint convention.
    baseline = grid_h - LEGEND_H // 2 + 1
    lx = PAD + 4
    drawn = {
        row["cid"] for _rec, rel in chosen for row in parse_label(label_path_for(root, rel), 1, 1)
    }
    for cid, (name, color) in enumerate(zip(all_names, all_colors)):
        if cid not in drawn:
            continue
        cv2.rectangle(canvas, (lx, baseline - 12), (lx + 18, baseline + 1), hex_to_bgr(color), -1)
        lx += 24
        cv2.putText(canvas, name, (lx, baseline), FONT, 0.48, (230, 230, 230), 1, cv2.LINE_AA)
        lx += cv2.getTextSize(name, FONT, 0.48, 1)[0][0] + 26

    note = "filled dot = front keypoint, hollow = back"
    nw = cv2.getTextSize(note, FONT, 0.45, 1)[0][0]
    cv2.putText(
        canvas, note, (grid_w - nw - PAD - 4, baseline), FONT, 0.45, (170, 170, 170), 1, cv2.LINE_AA
    )
    return canvas


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("gt", type=Path, help="eval dataset root (the dir score.py is given)")
    parser.add_argument("-o", "--output", type=Path, required=True, help="output PNG path")
    parser.add_argument("--cols", type=int, default=3, help="grid columns (default 3)")
    parser.add_argument("--rows", type=int, default=3, help="grid rows (default 3)")
    args = parser.parse_args()

    chosen = pick_frames(args.gt, reviewed_frames(args.gt), args.cols * args.rows)
    print(f"{len(chosen)} tiles")
    canvas = build_mosaic(args.gt, chosen, args.cols, args.rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), canvas)
    print(f"wrote {args.output} ({canvas.shape[1]}x{canvas.shape[0]})")


if __name__ == "__main__":
    main()
