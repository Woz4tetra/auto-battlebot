#!/usr/bin/env python3
"""Stage 1 of the open-set detector probe: promptable detectors on validated GT frames.

Runs zero-shot / promptable detectors over every validated frame of every eval
recording and stores raw detections for scoring by openset_probe_score.py. The
question under test (see docs/experiments/opponent_embedding/openset_probe_report.md):
does an off-the-shelf promptable model detect arbitrary robots uniformly across
OOD scenes, with zero prompting or a single pre-match exemplar?

Methods:

- owlv2_text:      google/owlv2-large-patch14-ensemble, text prompt (zero-prompt arm)
- owlv2_base_text: google/owlv2-base-patch16-ensemble, text prompt (edge-sized arm)
- owlv2_exemplar:  OWLv2 image-guided detection, seeded with the same 2 pre-match
                   opponent crops the embedding probe used (single-prompt arm)
- gdino_text:      IDEA-Research/grounding-dino-base, text prompt
- omdet_text:      omlab/omdet-turbo-swin-tiny-hf, text prompt (real-time-sized arm)
- yoloworld_text:  ultralytics yolov8x-worldv2, text prompt
- sam3_text:       facebook/sam3 concept prompt. Gated on HF; requires an approved
                   access request and a login token. Skipped if inaccessible.

Writes per recording under training/data/openset_probe/<recording>/:

- detections_<method>.csv: one row per detection (stamp_ns, role, score, xyxy, text)
- manifest_<method>.json: model id, prompt, score floor, per-frame latency stats

Usage:
    cd training/model_eval && python openset_probe_predict.py --methods owlv2_text,gdino_text
        [--eval-root DIR] [--recordings 10-06,massd] [--prompt robot] [--floor 0.02]
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import embedding_probe_common as common
import numpy as np
import pandas as pd
import torch
from tqdm import tqdm

OUTPUT_ROOT = common._REPO / "training" / "data" / "openset_probe"
SCORE_FLOOR = 0.02
DEFAULT_PROMPT = "robot"
EXEMPLAR_PAD = 0.10

METHOD_CHOICES = (
    "owlv2_text",
    "owlv2_base_text",
    "owlv2_exemplar",
    "gdino_text",
    "omdet_text",
    "yoloworld_text",
    "sam3_text",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--eval-root", type=Path, default=common.DEFAULT_EVAL_ROOT)
    parser.add_argument("--methods", type=str, default="owlv2_text")
    parser.add_argument("--recordings", type=str, default="")
    parser.add_argument("--prompt", type=str, default=DEFAULT_PROMPT)
    parser.add_argument("--floor", type=float, default=SCORE_FLOOR)
    return parser.parse_args()


def load_image(path: Path) -> np.ndarray:
    """RGB uint8 HxWx3."""
    bgr = cv2.imread(str(path))
    if bgr is None:
        raise FileNotFoundError(path)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def seed_crops(entry: dict) -> list[np.ndarray]:
    """Opponent crops from the seed frames, padded like the embedding probe gallery."""
    crops = []
    for stamp in common.seed_stamps(entry):
        boxes, labels = entry["frames"][stamp]
        image = load_image(entry["images"][stamp])
        height, width = image.shape[:2]
        for box, label in zip(boxes, labels):
            if label != common.OPPONENT:
                continue
            x1, y1, x2, y2 = box
            pad_x = (x2 - x1) * EXEMPLAR_PAD
            pad_y = (y2 - y1) * EXEMPLAR_PAD
            x1 = max(0, int(x1 - pad_x))
            y1 = max(0, int(y1 - pad_y))
            x2 = min(width, int(x2 + pad_x))
            y2 = min(height, int(y2 + pad_y))
            crops.append(image[y1:y2, x1:x2])
    return crops


class HfTextDetector:
    """Text-prompted open-vocabulary detectors from transformers."""

    def __init__(self, method: str, prompt: str, floor: float) -> None:
        self.method = method
        self.prompt = prompt
        self.floor = floor
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        if method in ("owlv2_text", "owlv2_base_text"):
            from transformers import Owlv2ForObjectDetection, Owlv2Processor

            self.model_id = (
                "google/owlv2-large-patch14-ensemble"
                if method == "owlv2_text"
                else "google/owlv2-base-patch16-ensemble"
            )
            self.processor = Owlv2Processor.from_pretrained(self.model_id)
            self.model = Owlv2ForObjectDetection.from_pretrained(self.model_id)
        elif method == "gdino_text":
            from transformers import GroundingDinoForObjectDetection, GroundingDinoProcessor

            self.model_id = "IDEA-Research/grounding-dino-base"
            self.processor = GroundingDinoProcessor.from_pretrained(self.model_id)
            self.model = GroundingDinoForObjectDetection.from_pretrained(self.model_id)
        elif method == "omdet_text":
            from transformers import OmDetTurboForObjectDetection, OmDetTurboProcessor

            self.model_id = "omlab/omdet-turbo-swin-tiny-hf"
            self.processor = OmDetTurboProcessor.from_pretrained(self.model_id)
            self.model = OmDetTurboForObjectDetection.from_pretrained(self.model_id)
        elif method == "sam3_text":
            from transformers import Sam3Model, Sam3Processor

            self.model_id = "facebook/sam3"
            self.processor = Sam3Processor.from_pretrained(self.model_id)
            self.model = Sam3Model.from_pretrained(self.model_id)
        else:
            raise ValueError(method)
        self.model = self.model.to(self.device).eval()

    @torch.inference_mode()
    def detect(self, image: np.ndarray) -> list[tuple[float, float, float, float, float]]:
        height, width = image.shape[:2]
        target = [(height, width)]
        if self.method == "gdino_text":
            text = self.prompt.rstrip(".") + "."
            inputs = self.processor(images=image, text=text, return_tensors="pt").to(self.device)
            outputs = self.model(**inputs)
            result = self.processor.post_process_grounded_object_detection(
                outputs, inputs.input_ids, threshold=self.floor, target_sizes=target
            )[0]
        elif self.method == "omdet_text":
            inputs = self.processor(images=image, text=[self.prompt], return_tensors="pt").to(
                self.device
            )
            outputs = self.model(**inputs)
            result = self.processor.post_process_grounded_object_detection(
                outputs,
                text_labels=[[self.prompt]],
                threshold=self.floor,
                nms_threshold=common.NMS_IOU,
                target_sizes=target,
            )[0]
        elif self.method == "sam3_text":
            inputs = self.processor(images=image, text=self.prompt, return_tensors="pt").to(
                self.device
            )
            outputs = self.model(**inputs)
            result = self.processor.post_process_object_detection(
                outputs, threshold=self.floor, target_sizes=target
            )[0]
        else:
            # OWLv2 pads the image to a square before resizing; its post-processing
            # scales normalized boxes by target_sizes, so the target must be the
            # padded square, not the original size (else boxes compress vertically).
            side = max(height, width)
            inputs = self.processor(text=[[self.prompt]], images=image, return_tensors="pt").to(
                self.device
            )
            outputs = self.model(**inputs)
            result = self.processor.post_process_grounded_object_detection(
                outputs, threshold=self.floor, target_sizes=[(side, side)]
            )[0]
        boxes = result["boxes"].cpu().numpy()
        if len(boxes):
            boxes[:, 0::2] = boxes[:, 0::2].clip(0, width)
            boxes[:, 1::2] = boxes[:, 1::2].clip(0, height)
        scores = result["scores"].cpu().numpy()
        return [
            (float(b[0]), float(b[1]), float(b[2]), float(b[3]), float(s))
            for b, s in zip(boxes, scores)
        ]


class Owlv2ExemplarDetector:
    """OWLv2 image-guided (one-shot) detection seeded with pre-match opponent crops."""

    def __init__(self, floor: float) -> None:
        from transformers import Owlv2ForObjectDetection, Owlv2Processor

        self.model_id = "google/owlv2-large-patch14-ensemble"
        self.floor = floor
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.processor = Owlv2Processor.from_pretrained(self.model_id)
        self.model = Owlv2ForObjectDetection.from_pretrained(self.model_id).to(self.device).eval()
        self.queries: list[np.ndarray] = []

    def set_queries(self, crops: list[np.ndarray]) -> None:
        self.queries = crops

    @torch.inference_mode()
    def detect(self, image: np.ndarray) -> list[tuple[float, float, float, float, float]]:
        height, width = image.shape[:2]
        side = max(height, width)  # padded-square target, same reason as the text path
        merged: list[tuple[float, float, float, float, float]] = []
        for query in self.queries:
            inputs = self.processor(images=image, query_images=query, return_tensors="pt").to(
                self.device
            )
            outputs = self.model.image_guided_detection(**inputs)
            result = self.processor.post_process_image_guided_detection(
                outputs,
                threshold=self.floor,
                nms_threshold=common.NMS_IOU,
                target_sizes=torch.tensor([[side, side]]),
            )[0]
            boxes = result["boxes"].cpu().numpy()
            if len(boxes):
                boxes[:, 0::2] = boxes[:, 0::2].clip(0, width)
                boxes[:, 1::2] = boxes[:, 1::2].clip(0, height)
            scores = result["scores"].cpu().numpy()
            merged += [
                (float(b[0]), float(b[1]), float(b[2]), float(b[3]), float(s))
                for b, s in zip(boxes, scores)
            ]
        return nms(merged, common.NMS_IOU)


class YoloWorldDetector:
    """Ultralytics YOLO-World with a custom text vocabulary."""

    def __init__(self, prompt: str, floor: float) -> None:
        from ultralytics import YOLOWorld

        self.model_id = "yolov8x-worldv2.pt"
        self.floor = floor
        self.model = YOLOWorld(self.model_id)
        # set_classes once at init: a second call after .predict hits an
        # ultralytics device mismatch in the cached CLIP text encoder.
        self.model.set_classes([prompt])

    def detect(self, image: np.ndarray) -> list[tuple[float, float, float, float, float]]:
        results = self.model.predict(
            cv2.cvtColor(image, cv2.COLOR_RGB2BGR),
            conf=self.floor,
            iou=common.NMS_IOU,
            verbose=False,
        )[0]
        boxes = results.boxes.xyxy.cpu().numpy()
        scores = results.boxes.conf.cpu().numpy()
        return [
            (float(b[0]), float(b[1]), float(b[2]), float(b[3]), float(s))
            for b, s in zip(boxes, scores)
        ]


def nms(
    detections: list[tuple[float, float, float, float, float]], iou_threshold: float
) -> list[tuple[float, float, float, float, float]]:
    if not detections:
        return []
    order = sorted(detections, key=lambda d: -d[4])
    boxes = np.asarray([d[:4] for d in order])
    keep: list[int] = []
    for index in range(len(order)):
        if all(
            common.iou_matrix(boxes[index : index + 1], boxes[kept : kept + 1])[0, 0]
            < iou_threshold
            for kept in keep
        ):
            keep.append(index)
    return [order[index] for index in keep]


def build_detector(method: str, prompt: str, floor: float):
    if method == "owlv2_exemplar":
        return Owlv2ExemplarDetector(floor)
    if method == "yoloworld_text":
        return YoloWorldDetector(prompt, floor)
    return HfTextDetector(method, prompt, floor)


def run_method(method: str, recordings: dict[str, dict], prompt: str, floor: float) -> None:
    detector = build_detector(method, prompt, floor)
    for recording, entry in recordings.items():
        short = common.SHORT_NAMES.get(recording, recording)
        out_dir = OUTPUT_ROOT / recording
        out_dir.mkdir(parents=True, exist_ok=True)
        seeds = set(common.seed_stamps(entry))
        if method == "owlv2_exemplar":
            detector.set_queries(seed_crops(entry))
        rows = []
        latencies = []
        for stamp in tqdm(entry["stamps"], desc=f"{method} {short}"):
            image = load_image(entry["images"][stamp])
            start = time.perf_counter()
            detections = detector.detect(image)
            latencies.append(time.perf_counter() - start)
            role = "seed" if stamp in seeds else "eval"
            for x1, y1, x2, y2, score in detections:
                rows.append(
                    {
                        "stamp_ns": stamp,
                        "role": role,
                        "score": score,
                        "x1": x1,
                        "y1": y1,
                        "x2": x2,
                        "y2": y2,
                    }
                )
        pd.DataFrame(rows).to_csv(out_dir / f"detections_{method}.csv", index=False)
        manifest = {
            "method": method,
            "model_id": detector.model_id,
            "prompt": prompt if "text" in method else None,
            "floor": floor,
            "frames": len(entry["stamps"]),
            "detections": len(rows),
            "latency_ms_median": float(np.median(latencies) * 1000),
            "latency_ms_p90": float(np.quantile(latencies, 0.9) * 1000),
        }
        (out_dir / f"manifest_{method}.json").write_text(json.dumps(manifest, indent=2) + "\n")
        print(
            f"{method} {short}: {len(rows)} detections, "
            f"median {manifest['latency_ms_median']:.0f} ms/frame"
        )


def main() -> None:
    args = parse_args()
    recordings = common.load_recordings(args.eval_root)
    if args.recordings:
        wanted = set(args.recordings.split(","))
        recordings = {
            name: entry
            for name, entry in recordings.items()
            if common.SHORT_NAMES.get(name, name) in wanted
        }
    for method in args.methods.split(","):
        if method not in METHOD_CHOICES:
            raise SystemExit(f"unknown method {method}; choices: {METHOD_CHOICES}")
        run_method(method, recordings, args.prompt, args.floor)


if __name__ == "__main__":
    main()
