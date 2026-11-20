"""Run DeepLab preprocessing arms on recorded Jetson frames; compare with the recorded masks."""

from __future__ import annotations

import argparse
import itertools
from glob import glob
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import torch
from tqdm import tqdm

from auto_battlebot.recording.mcap_io import decode_compressed_image, iter_messages
from auto_battlebot.segmentation.load_deeplabv3 import load_model
from auto_battlebot.segmentation.model_config import ModelConfig
from auto_battlebot.segmentation.preprocess import (
    DEPLOYED_UNTIL_2026_11,
    TRAINING,
    ChannelOrder,
    Geometry,
    Normalization,
    Preprocess,
    Resolution,
    model_input,
    output_labels,
)

FIELD_CHANNEL = 1
RECOVER_THRESHOLD = 64
STAMP_MATCH_NS = 1_000_000
OUT = Path(__file__).parent / "out"


def all_arms() -> list[Preprocess]:
    return [
        Preprocess(c, n, g) for c, n, g in itertools.product(ChannelOrder, Normalization, Geometry)
    ]


def recover_deployed_mask(overlay: np.ndarray, frame: np.ndarray) -> np.ndarray:
    """Floor pixels of a /field_mask overlay: 0.5 * frame + 0.5 * green where the mask is floor."""
    green = overlay[:, :, 1].astype(np.float32) - 0.5 * frame[:, :, 1].astype(np.float32)
    return (green > RECOVER_THRESHOLD).astype(np.uint8)


def iou(a: np.ndarray, b: np.ndarray) -> float:
    union = np.logical_or(a, b).sum()
    return float(np.logical_and(a, b).sum() / union) if union else 1.0


def frames_from(path: Path, every: int) -> list[tuple[str, int, np.ndarray, np.ndarray | None]]:
    """(kind, stamp_ns, frame, recorded_mask) for each field-mask event and every Nth frame."""
    pending_masks: dict[int, np.ndarray] = {}
    frames: dict[int, np.ndarray] = {}
    sampled: list[tuple[str, int, np.ndarray, np.ndarray | None]] = []
    count = 0
    for topic, _, data in iter_messages(path, ["/field_mask", "/camera/image"]):
        image = decode_compressed_image(data)
        if topic == "/field_mask":
            pending_masks[image.stamp_ns] = image.image
            continue
        frames[image.stamp_ns] = image.image
        if count % every == 0:
            sampled.append(("sample", image.stamp_ns, image.image, None))
        count += 1
        if len(frames) > 120:
            frames.pop(next(iter(frames)))
        for stamp in list(pending_masks):
            match = min(frames, key=lambda s: abs(s - stamp))
            if abs(match - stamp) <= STAMP_MATCH_NS:
                sampled.append(("mask_event", stamp, frames[match], pending_masks.pop(stamp)))
    return sampled


def run_arms(
    model: torch.nn.Module,
    config: ModelConfig,
    arms: list[Preprocess],
    frame: np.ndarray,
    device: torch.device,
) -> dict[str, np.ndarray]:
    inputs = torch.stack([model_input(frame, config, arm) for arm in arms]).to(device)
    with torch.no_grad():
        labels = torch.argmax(model(inputs), dim=1).to(torch.uint8).cpu().numpy()
    shape = (frame.shape[0], frame.shape[1])
    return {
        arm.spec: (
            output_labels(label, shape, config, arm, Resolution.NATIVE) == FIELD_CHANNEL
        ).astype(np.uint8)
        for arm, label in zip(arms, labels)
    }


def save_overlay(path: Path, frame: np.ndarray, masks: dict[str, np.ndarray]) -> None:
    """Deployed arm in red, training arm in green, overlap in yellow."""
    deployed = masks[DEPLOYED_UNTIL_2026_11.spec].astype(bool)
    training = masks[TRAINING.spec].astype(bool)
    tint = np.zeros_like(frame)
    tint[deployed, 2] = 255
    tint[training, 1] = 255
    cv2.imwrite(str(path), cv2.addWeighted(frame, 0.55, tint, 0.45, 0.0))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("recordings", nargs="+", help="MCAP paths or globs")
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--every", type=int, default=150, help="also score every Nth camera frame")
    parser.add_argument("--overlays", type=int, default=12, help="overlay images to save")
    parser.add_argument("--output", type=Path, default=OUT)
    args = parser.parse_args()

    paths = sorted({Path(p) for pattern in args.recordings for p in glob(pattern)})
    if not paths:
        raise SystemExit("No recordings matched")
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "overlays").mkdir(exist_ok=True)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model, config = load_model(args.checkpoint, device)
    arms = all_arms()
    rows = []
    overlays_left = args.overlays
    for path in tqdm(paths, desc="recordings"):
        for kind, stamp, frame, overlay in frames_from(path, args.every):
            masks = run_arms(model, config, arms, frame, device)
            recorded = recover_deployed_mask(overlay, frame) if overlay is not None else None
            for arm in arms:
                mask = masks[arm.spec]
                rows.append(
                    {
                        "recording": path.name,
                        "kind": kind,
                        "stamp_ns": stamp,
                        "arm": arm.spec,
                        "floor_fraction": float(mask.mean()),
                        "iou_vs_training": iou(mask, masks[TRAINING.spec]),
                        "iou_vs_deployed_arm": iou(mask, masks[DEPLOYED_UNTIL_2026_11.spec]),
                        "iou_vs_recorded": iou(mask, recorded) if recorded is not None else np.nan,
                    }
                )
            if kind == "mask_event" and overlays_left > 0:
                overlays_left -= 1
                save_overlay(args.output / "overlays" / f"{path.stem}_{stamp}.jpg", frame, masks)
    frame_table = pd.DataFrame(rows)
    frame_table.to_csv(args.output / "frames.csv", index=False)
    summary = (
        frame_table.groupby(["kind", "arm"])
        .agg(
            frames=("stamp_ns", "size"),
            floor_fraction=("floor_fraction", "mean"),
            iou_vs_training=("iou_vs_training", "mean"),
            iou_vs_training_p10=("iou_vs_training", lambda s: s.quantile(0.1)),
            iou_vs_recorded=("iou_vs_recorded", "mean"),
        )
        .reset_index()
    )
    summary.to_csv(args.output / "summary.csv", index=False)
    print(summary.to_string(index=False, float_format=lambda v: f"{v:.4f}"))


if __name__ == "__main__":
    main()
