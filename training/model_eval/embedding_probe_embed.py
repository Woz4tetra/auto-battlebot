#!/usr/bin/env python3
"""Stage 2 of the embedding prototype probe: cache frozen embeddings for all candidates.

Reads candidates.csv per recording (stage 1), crops each candidate box from the dataset
PNGs at the plan's context paddings (0%, 10%, 25%), and embeds every crop with each
frozen embedder. Saves one npz per (recording, embedder) with L2-normalized float32
arrays pad000/pad010/pad025 of shape (n_candidates, dim), row-aligned with the csv
through the stored row_index array.

Embedders (timm, pretrained, frozen): DINOv2 ViT-S/14 (primary), CLIP ViT-B/32,
ResNet-50 ImageNet (deployable-size floor).

Usage:
    cd training/model_eval && python embedding_probe_embed.py [--recordings 10-06,massd]
        [--embedders dinov2,clip,resnet50] [--batch 64]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import embedding_probe_common as common
import numpy as np
import torch
from tqdm import tqdm

EMBEDDERS = {
    # name: (timm model, create kwargs)
    "dinov2": (
        "vit_small_patch14_reg4_dinov2.lvd142m",
        {"img_size": 224, "dynamic_img_size": True},
    ),
    "clip": ("vit_base_patch32_clip_224.openai", {}),
    "resnet50": ("resnet50.a1_in1k", {}),
}
PAD_KEYS = {0.0: "pad000", 0.10: "pad010", 0.25: "pad025"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recordings", type=str, default="")
    parser.add_argument("--embedders", type=str, default=",".join(EMBEDDERS))
    parser.add_argument("--batch", type=int, default=64)
    parser.add_argument("--eval-root", type=Path, default=common.DEFAULT_EVAL_ROOT)
    return parser.parse_args()


class Embedder:
    def __init__(self, name: str, device: torch.device) -> None:
        import timm

        model_name, kwargs = EMBEDDERS[name]
        self.model = timm.create_model(model_name, pretrained=True, num_classes=0, **kwargs)
        self.model.eval().to(device)
        config = timm.data.resolve_model_data_config(self.model)
        self.size = int(kwargs.get("img_size", config["input_size"][1]))
        self.mean = np.asarray(config["mean"], dtype=np.float32)
        self.std = np.asarray(config["std"], dtype=np.float32)
        self.device = device

    def __call__(self, crops: list[np.ndarray]) -> np.ndarray:
        blob = np.stack(
            [
                cv2.resize(crop, (self.size, self.size), interpolation=cv2.INTER_LINEAR)
                for crop in crops
            ]
        )
        blob = blob[..., ::-1].astype(np.float32) / 255.0  # BGR -> RGB
        blob = (blob - self.mean) / self.std
        tensor = torch.from_numpy(blob.transpose(0, 3, 1, 2).copy()).to(self.device)
        with torch.no_grad():
            features = self.model(tensor).float().cpu().numpy()
        norms = np.linalg.norm(features, axis=1, keepdims=True)
        return features / np.clip(norms, 1e-12, None)


def main() -> None:
    args = parse_args()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    recordings = common.load_recordings(args.eval_root)
    wanted = {name.strip() for name in args.recordings.split(",") if name.strip()}
    embedder_names = [name.strip() for name in args.embedders.split(",") if name.strip()]

    for embedder_name in embedder_names:
        embedder = Embedder(embedder_name, device)
        for recording, entry in sorted(recordings.items()):
            short = common.short_name(recording)
            if wanted and short not in wanted:
                continue
            out_path = common.recording_dir(recording) / common.EMBEDDINGS_NPZ.format(
                embedder=embedder_name
            )
            candidates = common.load_candidates(recording)
            arrays: dict[str, np.ndarray] = {}
            for pad_fraction, key in PAD_KEYS.items():
                features = []
                for stamp, group in tqdm(
                    candidates.groupby("stamp_ns", sort=True),
                    desc=f"{embedder_name}/{short}/{key}",
                    leave=False,
                ):
                    image = cv2.imread(str(entry["images"][stamp]))
                    crops = [
                        common.crop_box(
                            image,
                            row[["x1", "y1", "x2", "y2"]].to_numpy(dtype=np.float64),
                            pad_fraction,
                        )
                        for _, row in group.iterrows()
                    ]
                    for start in range(0, len(crops), args.batch):
                        features.append(embedder(crops[start : start + args.batch]))
                stacked = np.concatenate(features) if features else np.zeros((0, 1), np.float32)
                arrays[key] = stacked.astype(np.float32)
            # groupby(sort=True) orders rows by stamp; store that order explicitly.
            order = candidates.sort_values("stamp_ns", kind="stable").index.to_numpy()
            arrays["row_index"] = order
            np.savez_compressed(out_path, **arrays)
            print(f"{embedder_name}/{short}: {arrays['pad010'].shape} -> {out_path.name}")
        del embedder
        torch.cuda.empty_cache()


if __name__ == "__main__":
    main()
