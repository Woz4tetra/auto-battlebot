"""Plot robot_filter inputs recorded by record_filter_inputs.cpp.

Reads the .jsonl format written by src/record_filter_inputs.cpp (one JSON object per line: tick,
svo_frame_index, keypoints, robot_blob_keypoints, camera_info, field_description -- see
include/robot_filter/filter_input_record.hpp for the schema) and plots keypoint/blob detection
positions over time, one line per (source, label, keypoint_label) combination.

Usage:
    python playground/filter_inputs/plot_filter_inputs.py filter_inputs.jsonl
    python playground/filter_inputs/plot_filter_inputs.py filter_inputs.jsonl --out xy.png
    python playground/filter_inputs/plot_filter_inputs.py filter_inputs.jsonl --show

    # Step through frames with 'n' (next) / 'p' (previous) / 'q' (quit) on a blank
    # canvas -- there's no RGB image in the recording, but the recorded pixel coordinates still
    # let you see detections move frame to frame, same interaction as scratch.cpp's live viewer
    # (which only goes forward; this one can go backward too since it's replaying from a file
    # rather than a live camera feed).
    python playground/filter_inputs/plot_filter_inputs.py filter_inputs.jsonl --interactive
"""

import argparse
import json
import zlib
from pathlib import Path

import matplotlib

import pandas as pd


def load_records(path: Path) -> list[dict]:
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def records_to_dataframe(records: list[dict]) -> pd.DataFrame:
    """Flattens keypoints + robot_blob_keypoints across all records into one long DataFrame:
    one row per detected point, columns [tick, stamp, source, label, keypoint_label, x, y,
    confidence]."""
    rows = []
    for record in records:
        tick = record["tick"]
        for source in ("keypoints", "robot_blob_keypoints"):
            stamped = record[source]
            stamp = stamped["header"]["stamp"]
            for point in stamped["points"]:
                rows.append(
                    {
                        "tick": tick,
                        "stamp": stamp,
                        "source": source,
                        "label": point["label"],
                        "keypoint_label": point["keypoint_label"],
                        "x": point["x"],
                        "y": point["y"],
                        "confidence": point["confidence"],
                    }
                )
    return pd.DataFrame(rows)


def plot_xy_over_time(df: pd.DataFrame, out_path: Path | None, show: bool) -> None:
    import matplotlib.pyplot as plt

    fig, (ax_x, ax_y) = plt.subplots(2, 1, sharex=True, figsize=(12, 7))

    groups = df.groupby(["source", "label", "keypoint_label"])
    for (source, label, keypoint_label), group in groups:
        group = group.sort_values("tick")
        series_label = f"{source}:{label}/{keypoint_label}"
        ax_x.plot(group["tick"], group["x"], marker=".", markersize=3, label=series_label)
        ax_y.plot(group["tick"], group["y"], marker=".", markersize=3, label=series_label)

    ax_x.set_ylabel("x (pixels)")
    ax_y.set_ylabel("y (pixels)")
    ax_y.set_xlabel("tick")
    ax_x.set_title("Keypoint / blob detection position over time")
    ax_x.legend(fontsize="small", loc="upper right", ncol=2)
    fig.tight_layout()

    if out_path is not None:
        fig.savefig(out_path, dpi=150)
        print(f"Wrote {out_path}")
    if show:
        plt.show()


def label_color_bgr(label: str) -> tuple[int, int, int]:
    """Deterministic BGR color for a label string (golden-angle-ish HSV spread via a stable hash,
    since plain hash() is randomized per-process). Doesn't need to match
    include/label_utils.hpp's get_color_for_index pixel-for-pixel -- just distinct and stable
    across runs so the same label always looks the same."""
    hue = (zlib.crc32(label.encode()) % 360) / 360.0
    import colorsys

    r, g, b = colorsys.hsv_to_rgb(hue, 0.75, 0.95)
    return (int(b * 255), int(g * 255), int(r * 255))


def draw_point(img, x: float, y: float, color: tuple[int, int, int], text: str) -> None:
    import cv2

    pt = (int(round(x)), int(round(y)))
    cv2.circle(img, pt, 7, (255, 255, 255), -1, cv2.LINE_AA)
    cv2.circle(img, pt, 5, color, -1, cv2.LINE_AA)
    cv2.putText(img, text, (pt[0] + 9, pt[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2,
                cv2.LINE_AA)
    cv2.putText(img, text, (pt[0] + 8, pt[1] + 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)


def run_interactive_viewer(records: list[dict]) -> None:
    """Steps through records on a blank (no RGB stored) canvas sized to each record's own
    camera_info, drawing keypoints and robot_blob_keypoints same as scratch.cpp's live overlay.
    Blocks on cv2.waitKey(0): 'n' advances, 'p' goes back, 'q'/Esc quits, anything else is
    ignored so a stray keypress can't skip a frame. Index is clamped to [0, len(records)); 'p' at
    the first record is a no-op, 'n' past the last prints a message and exits."""
    import cv2
    import numpy as np

    if not records:
        print("No records to view.")
        return

    window = "plot_filter_inputs: keypoints (blank canvas)"
    idx = 0
    while True:
        idx = max(0, idx)
        if idx >= len(records):
            print("Reached the end of the recording.")
            break

        record = records[idx]
        camera_info = record["camera_info"]
        width = camera_info["width"] if camera_info["width"] > 0 else 1280
        height = camera_info["height"] if camera_info["height"] > 0 else 720
        canvas = np.full((height, width, 3), 30, dtype=np.uint8)

        for point in record["robot_blob_keypoints"]["points"]:
            text = f"blob:{point['label']}"
            draw_point(canvas, point["x"], point["y"], label_color_bgr(point["label"]), text)
        for point in record["keypoints"]["points"]:
            draw_point(canvas, point["x"], point["y"], label_color_bgr(point["label"]),
                       point["keypoint_label"])

        cv2.putText(
            canvas,
            f"tick {record['tick']}  svo_frame {record['svo_frame_index']}  "
            f"[n]ext  [p]rev  [q]uit",
            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA,
        )
        cv2.imshow(window, canvas)

        delta = 0
        while True:
            key = cv2.waitKey(0) & 0xFF
            if key in (ord("q"), 27):
                cv2.destroyAllWindows()
                return
            if key == ord("n"):
                delta = 1
                break
            if key == ord("p"):
                delta = -1
                break
            # Anything else (window manager events, modifier keys, ...) -- keep waiting.
        idx += delta

    cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Path to a .jsonl file from record_filter_inputs")
    parser.add_argument(
        "--out", type=Path, default=None, help="Output PNG path (default: <input>_xy.png)"
    )
    parser.add_argument("--show", action="store_true", help="Also display the plot interactively")
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Step through frames with 'n'/'p'/'q' on a blank canvas (cv2.imshow) instead of "
        "producing the static plot; needs a real DISPLAY",
    )
    args = parser.parse_args()

    records = load_records(args.input)
    print(f"Loaded {len(records)} records from {args.input}")

    if args.interactive:
        run_interactive_viewer(records)
        return

    if not args.show:
        # Headless by default: this runs the same inside the container (no DISPLAY) as on a
        # desktop. --show switches to an interactive backend before pyplot is imported.
        matplotlib.use("Agg")

    out_path = args.out if args.out is not None else args.input.with_name(args.input.stem + "_xy.png")

    df = records_to_dataframe(records)
    print(f"{len(df)} detected points across {df['tick'].nunique()} ticks")

    plot_xy_over_time(df, out_path, args.show)


if __name__ == "__main__":
    main()
