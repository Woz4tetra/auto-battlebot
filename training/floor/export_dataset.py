#!/usr/bin/env python3
"""Export sampled frame/mask pairs as YOLO-Seg dataset."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import yaml

from common import (
    LOGGER,
    append_jsonl,
    configure_logging,
    json_dump,
    load_pipeline_config,
    mkdir,
)


def load_jsonl(path: Path) -> list[dict]:
    rows: list[dict] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def get_frame(video_path: Path, frame_idx: int) -> tuple[bool, any]:
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        return False, None
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
    ok, frame = cap.read()
    cap.release()
    return ok, frame


def contour_to_yolo_polygon(mask_u8, epsilon_ratio: float, min_polygon_points: int) -> list[float]:
    contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []
    contour = max(contours, key=cv2.contourArea)
    perimeter = cv2.arcLength(contour, True)
    epsilon = epsilon_ratio * perimeter
    approx = cv2.approxPolyDP(contour, epsilon, True)
    if len(approx) < min_polygon_points:
        approx = contour
    h, w = mask_u8.shape[:2]
    flat: list[float] = []
    for p in approx[:, 0, :]:
        x = float(p[0]) / w
        y = float(p[1]) / h
        flat.extend([max(0.0, min(1.0, x)), max(0.0, min(1.0, y))])
    return flat


def main() -> None:
    parser = argparse.ArgumentParser(description="Export YOLO-Seg dataset from filtered masks")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).parent / "config" / "pipeline.toml",
    )
    parser.add_argument("--stride", type=int, default=None)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    configure_logging(args.verbose)
    cfg = load_pipeline_config(args.config)
    stride = args.stride if args.stride is not None else cfg.export.stride

    pass_manifest = cfg.paths.metadata_dir / "quality_pass_manifest.jsonl"
    if not pass_manifest.exists():
        raise RuntimeError("No quality pass manifest found. Run quality_filter.py first.")

    rows = load_jsonl(pass_manifest)
    rows = sorted(rows, key=lambda r: (r["video_path"], int(r["frame_idx"])))
    sampled = [r for r in rows if int(r["frame_idx"]) % stride == 0]

    images_dir = mkdir(cfg.paths.export_dir / "images")
    labels_dir = mkdir(cfg.paths.export_dir / "labels")
    masks_dir = mkdir(cfg.paths.export_dir / "masks")

    exported_rows: list[dict] = []
    for idx, row in enumerate(sampled):
        video_path = Path(row["video_path"])
        frame_idx = int(row["frame_idx"])
        ok, frame = get_frame(video_path, frame_idx)
        if not ok:
            LOGGER.warning("Failed reading frame %d from %s", frame_idx, video_path)
            continue

        mask_path = Path(row["filtered_mask_path"])
        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if mask is None:
            continue
        mask_u8 = (mask > 0).astype("uint8") * 255
        polygon = contour_to_yolo_polygon(
            mask_u8,
            epsilon_ratio=cfg.export.polygon_epsilon_ratio,
            min_polygon_points=cfg.export.min_polygon_points,
        )
        if not polygon:
            continue

        stem = f"{video_path.stem}_f{frame_idx:08d}"
        image_path = images_dir / f"{stem}{cfg.export.image_ext}"
        label_path = labels_dir / f"{stem}.txt"
        out_mask_path = masks_dir / f"{stem}_mask.png"
        cv2.imwrite(str(image_path), frame)
        cv2.imwrite(str(out_mask_path), mask_u8)

        with label_path.open("w", encoding="utf-8") as f:
            # class_id + polygon coordinates
            values = " ".join(f"{v:.6f}" for v in polygon)
            f.write(f"0 {values}\n")

        exported_rows.append(
            {
                "video_path": str(video_path),
                "frame_idx": frame_idx,
                "image_path": str(image_path),
                "label_path": str(label_path),
                "mask_path": str(out_mask_path),
                "confidence": row.get("confidence", 0.0),
            }
        )

    dataset_yaml = {
        "path": str(cfg.paths.export_dir),
        "train": "images",
        "val": "images",
        "test": "images",
        "names": ["floor"],
        "colors": ["gray"],
        "nc": 1,
    }
    with (cfg.paths.export_dir / "data.yaml").open("w", encoding="utf-8") as f:
        yaml.safe_dump(dataset_yaml, f, sort_keys=False)

    manifest_path = cfg.paths.metadata_dir / "export_manifest.jsonl"
    append_jsonl(manifest_path, exported_rows)
    json_dump(
        cfg.paths.metadata_dir / "export_summary.json",
        {
            "stride": stride,
            "exported_frames": len(exported_rows),
            "manifest": str(manifest_path),
            "dataset_root": str(cfg.paths.export_dir),
        },
    )
    LOGGER.info("Done. Exported %d dataset frames.", len(exported_rows))


if __name__ == "__main__":
    main()
