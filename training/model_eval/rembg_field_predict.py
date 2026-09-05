#!/usr/bin/env python3
"""Run rembg over the eval set in three arms, tune, and write predictions for score.py.

Three subcommands, run in order:

    cache    one rembg pass on the raw frame and one on the floor raster per frame, plus the
             DeepLab floor mask, all written to a PNG cache so nothing below re-runs a network
    tune     sweep matte threshold, min_area and overlap fraction on the first frames of each
             recording (held out of scoring), pick each arm's values by F1 at IoU 0.5, and
             keep the whole sweep surface
    predict  write one predictions JSON per arm over the scored frames, plus a scoring root
             (symlinks and a filtered validation_state.json) that pins score.py to exactly
             those frames

Usage:
    venv/bin/python training/model_eval/rembg_field_predict.py cache \
        training/data/nhrl_keypoints_eval_test -o training/data/eval_results/rembg
    venv/bin/python training/model_eval/rembg_field_predict.py tune ...same args...
    venv/bin/python training/model_eval/rembg_field_predict.py predict ...same args...

See rembg_field.py for what the arms are and why the field mask is applied after inference.
"""

from __future__ import annotations

import argparse
import itertools
import json
import time
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
from rembg_field import (
    ARMS,
    POST_HOC_ARMS,
    TUNING_FRAMES_PER_RECORDING,
    ArmParams,
    Component,
    FieldSegmenter,
    FrameRef,
    MatteCache,
    Saliency,
    enumerate_frames,
    geometric_field_mask,
    largest_component_hull,
    load_params,
    make_raster,
    matte_components,
    parallax_px,
    post_hoc_select,
    scored_frames,
    tuning_frames,
    warp_to_raster,
    warped_select,
    write_predictions,
)
from score import Frame, Taxonomy, load_gt, pr_from_counts, pr_per_frame
from tqdm import tqdm

from auto_battlebot.camera_geometry import ground_range_m, pixels_to_floor
from auto_battlebot.floor_background import FloorRaster

# The plan swept 64..208. Every arm's optimum sat at 208, the top of that range, because the
# network paints the whole floor at moderate alpha and only the robots reach the top of the
# scale, so the grid extends to 240 to find where F1 actually turns over.
THRESHOLDS = list(range(64, 241, 16))  # 12 values
OVERLAPS = [round(0.1 * i, 1) for i in range(1, 10)]  # 9 values
# min_area is swept per arm because the units differ: image pixels for the post-hoc arms,
# raster pixels (400 px/m) for the warped one. Both are 8 log-spaced values.
MIN_AREAS = {
    "rembg_geom": [100 * 2**i for i in range(8)],
    "rembg_deeplab": [100 * 2**i for i in range(8)],
    "rembg_warped": [200 * 2**i for i in range(8)],
}
TUNE_IOU = 0.5
MASK_NAMES = {"rembg_geom": "geom", "rembg_deeplab": "deeplab"}

# Robot heights the parallax figure is quoted at.
PARALLAX_HEIGHTS_M = (0.1, 0.2)


def read_frame(frame: FrameRef) -> np.ndarray:
    image = cv2.imread(str(frame.image_path))
    if image is None:
        raise SystemExit(f"Failed to read {frame.image_path}")
    return image


# --- cache -----------------------------------------------------------------------------


def cache_frames(frames: list[FrameRef], cache: MatteCache, raster: FloorRaster) -> dict:
    """Fill the cache for every frame given. Returns the timing report."""
    saliency = Saliency()
    print(f"rembg providers: {saliency.providers}")
    segmenter = FieldSegmenter()
    print(f"deeplab: {segmenter.config}")
    for frame in tqdm(frames, desc="cache", unit="frame"):
        image = None
        if cache.get("raw", frame) is None:
            image = read_frame(frame)
            cache.put("raw", frame, saliency.matte(image))
        if cache.get("deeplab_raw", frame) is None:
            image = read_frame(frame) if image is None else image
            cache.put("deeplab_raw", frame, segmenter.raw_mask(image))
        if frame.posed and cache.get("raster", frame) is None:
            image = read_frame(frame) if image is None else image
            assert frame.geometry is not None
            warped, _ = warp_to_raster(image, raster, raster.image_from_raster(frame.geometry))
            cache.put("raster", frame, saliency.matte(warped))
    return {
        "rembg": saliency.timing_summary(),
        "rembg_providers": saliency.providers,
        "deeplab_ms_median": float(np.median(segmenter.timings[1:] or [0.0]) * 1000.0),
    }


# --- per-frame evaluation ----------------------------------------------------------------


class FrameContext:
    """Everything the arms need for one frame, computed once and shared across parameters."""

    def __init__(self, frame: FrameRef, cache: MatteCache, raster: FloorRaster) -> None:
        self.frame = frame
        self.raw = cache.get("raw", frame)
        deeplab_raw = cache.get("deeplab_raw", frame)
        if self.raw is None or deeplab_raw is None:
            raise SystemExit(f"{frame.image_path}: run `cache` first")
        self.masks: dict[str, np.ndarray] = {"deeplab": largest_component_hull(deeplab_raw)}
        self.homography: np.ndarray | None = None
        self.raster_matte: np.ndarray | None = None
        if frame.geometry is not None:
            self.homography = raster.image_from_raster(frame.geometry)
            self.masks["geom"] = geometric_field_mask(raster, frame.geometry, frame.size)
            self.raster_matte = cache.get("raster", frame)
        self._components: dict[tuple[str, int], list[Component]] = {}

    def components(self, kind: str, threshold: int) -> list[Component]:
        key = (kind, threshold)
        if key not in self._components:
            if kind == "raw":
                self._components[key] = matte_components(self.raw, threshold, self.masks)
            else:
                assert self.raster_matte is not None
                self._components[key] = matte_components(self.raster_matte, threshold)
        return self._components[key]

    def detections(self, arm: str, params: ArmParams) -> list[Component]:
        if arm == "rembg_warped":
            if self.homography is None or self.raster_matte is None:
                return []
            return warped_select(
                self.components("raster", params.threshold),
                self.homography,
                self.frame.size,
                params.min_area,
            )
        mask_name = MASK_NAMES[arm]
        if mask_name not in self.masks:
            return []
        assert params.overlap is not None
        return post_hoc_select(
            self.components("raw", params.threshold), mask_name, params.min_area, params.overlap
        )


def score_frame(gt: tuple, detections: list[Component]) -> Frame:
    gt_boxes, gt_labels, gt_keypoints = gt
    return Frame(
        gt_boxes=gt_boxes,
        gt_labels=gt_labels,
        gt_keypoints=gt_keypoints,
        pred_boxes=np.asarray([d.box for d in detections], dtype=np.float64).reshape(-1, 4),
        pred_labels=["robot"] * len(detections),
        pred_scores=np.asarray([d.mean_alpha / 255.0 for d in detections]),
        pred_keypoints=[np.zeros((0, 3)) for _ in detections],
    )


def f1_for(contexts: list[FrameContext], gts: dict, arm: str, params: ArmParams) -> dict:
    taxonomy = Taxonomy(None)
    frames = [score_frame(gts[ctx.frame.stamp_ns], ctx.detections(arm, params)) for ctx in contexts]
    counts, _ = pr_per_frame(frames, taxonomy, "agnostic", TUNE_IOU)
    return pr_from_counts(counts)


# --- tune ------------------------------------------------------------------------------


def param_grid(arm: str) -> list[ArmParams]:
    if arm in POST_HOC_ARMS:
        return [
            ArmParams(t, a, o)
            for t, a, o in itertools.product(THRESHOLDS, MIN_AREAS[arm], OVERLAPS)
        ]
    return [ArmParams(t, a) for t, a in itertools.product(THRESHOLDS, MIN_AREAS[arm])]


def sweep_arm(contexts: list[FrameContext], gts: dict, arm: str) -> pd.DataFrame:
    rows = []
    for params in tqdm(param_grid(arm), desc=f"sweep {arm}", unit="cfg", leave=False):
        metrics = f1_for(contexts, gts, arm, params)
        rows.append({"arm": arm, **params.to_json(), **metrics})
    return pd.DataFrame(rows)


def choose(surface: pd.DataFrame) -> ArmParams:
    """Argmax F1; ties go to the larger threshold, then larger min_area (the stricter cut)."""
    ordered = surface.sort_values(["f1", "threshold", "min_area"], ascending=False)
    best = ordered.iloc[0]
    overlap = None if pd.isna(best["overlap"]) else float(best["overlap"])
    return ArmParams(int(best["threshold"]), int(best["min_area"]), overlap)


def tune(frames: list[FrameRef], cache: MatteCache, raster: FloorRaster, out: Path) -> None:
    held_out = tuning_frames(frames)
    gts, _, _ = load_gt(frames[0].subdataset.parent)
    contexts = [FrameContext(f, cache, raster) for f in tqdm(held_out, desc="load", unit="frame")]
    report: dict = {"tuning_frames": len(held_out), "iou": TUNE_IOU}
    surfaces = []
    for arm in ARMS:
        surface = sweep_arm(contexts, gts, arm)
        surfaces.append(surface)
        chosen = choose(surface)
        best = surface.sort_values("f1", ascending=False).iloc[0]
        report[arm] = {
            "chosen": chosen.to_json(),
            "f1": float(best["f1"]),
            "precision": float(best["precision"]),
            "recall": float(best["recall"]),
            "grid_size": len(surface),
        }
        print(
            f"{arm}: {chosen} -> F1 {best['f1']:.3f} "
            f"(P {best['precision']:.3f} R {best['recall']:.3f})"
        )
    pd.concat(surfaces).to_csv(out / "sweep_surface.csv", index=False)
    (out / "params.json").write_text(json.dumps(report, indent=1))
    print(f"wrote {out / 'params.json'} and sweep_surface.csv")


# --- predict ---------------------------------------------------------------------------


def write_scoring_root(root: Path, frames: list[FrameRef], dest: Path) -> None:
    """A dataset root score.py reads as exactly `frames`: symlinked subdatasets and a
    validation_state.json that passes only those stems."""
    dest.mkdir(parents=True, exist_ok=True)
    for subdataset in sorted({f.subdataset for f in frames}):
        link = dest / subdataset.name
        if link.is_symlink() or link.exists():
            link.unlink()
        link.symlink_to(subdataset.resolve())
    state = {f"{f.recording}/images/{f.stem}.png": "pass" for f in frames}
    (dest / "validation_state.json").write_text(json.dumps(state, indent=1))


def parallax_report(frames: list[FrameRef], gts: dict) -> dict:
    """Expected raster smear of each GT box's floor footprint, from the frame's own pose."""
    rows = []
    for frame in frames:
        geometry = frame.geometry
        assert geometry is not None
        boxes, _, _ = gts[frame.stamp_ns]
        for box in boxes:
            foot = np.array([[(box[0] + box[2]) / 2.0, box[3]]])
            floor = pixels_to_floor(foot, geometry)[0]
            distance = ground_range_m(floor, geometry)
            if not np.isfinite(distance):
                continue
            rows.append(
                {
                    "camera_height_m": geometry.camera_height_m,
                    "distance_m": distance,
                    **{
                        f"smear_px_h{int(h * 100)}": parallax_px(
                            distance, h, geometry.camera_height_m
                        )
                        for h in PARALLAX_HEIGHTS_M
                    },
                }
            )
    table = pd.DataFrame(rows)
    return {
        "boxes": len(table),
        "camera_height_m": {
            "median": float(table["camera_height_m"].median()),
            "min": float(table["camera_height_m"].min()),
            "max": float(table["camera_height_m"].max()),
        },
        "distance_m": {
            "median": float(table["distance_m"].median()),
            "p90": float(table["distance_m"].quantile(0.9)),
        },
        **{
            column: {
                "median": float(table[column].median()),
                "p90": float(table[column].quantile(0.9)),
            }
            for column in table.columns
            if column.startswith("smear_px")
        },
    }


def predict(frames: list[FrameRef], cache: MatteCache, raster: FloorRaster, out: Path) -> None:
    params = load_params(out / "params.json")
    scored = scored_frames(frames)
    write_scoring_root(frames[0].subdataset.parent, scored, out / "scoring_root")
    gts, _, _ = load_gt(frames[0].subdataset.parent)
    unposed = [f for f in frames if not f.posed and not f.tuning]

    predictions: dict[str, dict[str, list[dict]]] = {arm: {} for arm in ARMS}
    extension: dict[str, list[dict]] = {}
    started = time.perf_counter()
    mask_seconds = 0.0
    for frame in tqdm(scored + unposed, desc="predict", unit="frame"):
        mask_started = time.perf_counter()
        context = FrameContext(frame, cache, raster)
        mask_seconds += time.perf_counter() - mask_started
        for arm in ARMS:
            rows = [c.row() for c in context.detections(arm, params[arm])]
            if frame.posed:
                predictions[arm][frame.stem] = rows
            if arm == "rembg_deeplab":
                extension[frame.stem] = rows
    elapsed = time.perf_counter() - started

    for arm in ARMS:
        write_predictions(
            out / f"{arm}.json",
            predictions[arm],
            {"source": "rembg_field_predict.py", "arm": arm, "params": params[arm].to_json()},
        )
        total = sum(len(rows) for rows in predictions[arm].values())
        print(f"{arm}: {total} detections over {len(predictions[arm])} frames")
    write_predictions(
        out / "rembg_deeplab_all_frames.json",
        extension,
        {
            "source": "rembg_field_predict.py",
            "arm": "rembg_deeplab",
            "note": "all non-tuning frames including unposed; footnote only",
            "params": params["rembg_deeplab"].to_json(),
        },
    )
    write_scoring_root(frames[0].subdataset.parent, scored + unposed, out / "scoring_root_all")

    report = {
        "scored_frames": len(scored),
        "unposed_frames": len(unposed),
        "tuning_frames": len(tuning_frames(frames)),
        "by_recording": {
            name: sum(1 for f in scored if f.recording == name)
            for name in sorted({f.recording for f in frames})
        },
        "by_resolution": {
            f"{w}x{h}": sum(1 for f in scored if f.size == (w, h))
            for (w, h) in sorted({f.size for f in scored})
        },
        "post_process_ms_per_frame": 1000.0 * elapsed / max(1, len(scored) + len(unposed)),
        "mask_ms_per_frame": 1000.0 * mask_seconds / max(1, len(scored) + len(unposed)),
        "parallax": parallax_report(scored, gts),
    }
    (out / "predict_report.json").write_text(json.dumps(report, indent=1))
    print(json.dumps(report, indent=1))


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("command", choices=("cache", "tune", "predict"))
    parser.add_argument("gt", type=Path, help="eval dataset root (a dir of sub datasets)")
    parser.add_argument("-o", "--output", type=Path, required=True, help="results directory")
    parser.add_argument(
        "--tuning-per-recording",
        type=int,
        default=TUNING_FRAMES_PER_RECORDING,
        help="frames per recording held out for tuning",
    )
    parser.add_argument("--tuning-only", action="store_true", help="cache: only the tuning frames")
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    cache = MatteCache(args.output / "cache")
    raster = make_raster()
    frames = enumerate_frames(args.gt, args.tuning_per_recording)
    print(
        f"{len(frames)} frames, {sum(f.posed for f in frames)} posed, "
        f"{len(tuning_frames(frames))} tuning, {len(scored_frames(frames))} scored"
    )

    if args.command == "cache":
        wanted = tuning_frames(frames) if args.tuning_only else frames
        timing = cache_frames(wanted, cache, raster)
        (args.output / "timing.json").write_text(json.dumps(timing, indent=1))
        print(json.dumps(timing, indent=1))
    elif args.command == "tune":
        tune(frames, cache, raster, args.output)
    else:
        predict(frames, cache, raster, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
