#!/usr/bin/env python3
"""Extract ZED depth for every ground-truth frame in an eval dataset, into a cache.

Depth is not recorded in the MCAP: the ZED SDK computes it on device and the pipeline never
publishes it. The stereo pair is in the SVO though, so replaying the SVO through pyzed with the
same DEPTH_MODE the runtime uses reproduces what the camera would have measured.

The join is by SVO frame index, never by timestamp. Each GT frame's
`camera_transforms/<stamp_ns>.json` carries `image_stream_index`, which is the absolute index
into the SVO. SVO image stamps sit about half a frame before the pipeline stamps that name the
GT images, so a nearest-timestamp match silently picks the wrong frame.

Frames whose transform method is `ambiguous` or `unavailable` are skipped: the first has two
processed frames claiming one index and the second has no pose at all, so neither can be placed
in the field frame later.

Output is one `<subdataset>.npz` per sub dataset, keyed by stamp_ns, holding float16 depth in
metres at the SVO's native resolution. Non-finite depth (no stereo match) is preserved as NaN,
because which pixels the camera failed to measure is the result this experiment turns on.

Usage:
    python playground/cache_gt_depth.py training/data/nhrl_keypoints_eval_test \
        --svo-roots data/svo -o <cache dir>
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import yaml

USABLE_METHODS = ("exact", "interpolated")

# ZED_NEURAL_LIGHT is the config/_common.toml default and the cheapest neural mode. NEURAL_PLUS
# is the most accurate one the SDK offers and is what a depth-based detector deserves to be
# graded on: on a sample MassD frame it returns depth for 0.718 of pixels against
# NEURAL_LIGHT's 0.583. Pass --depth-mode to pick; the cache name records the choice.
DEFAULT_DEPTH_MODE = "NEURAL_PLUS"

# config/_common.toml does not set depth_minimum_distance; 0.3 m is the ZED default for METER
# units and the arena floor is never closer than that.
DEPTH_MIN_M = 0.3


def resolve_svo(source_mcap: str, roots: list[Path]) -> Path | None:
    """The SVO for a recording, found by the timestamp suffix the MCAP name carries.

    Recordings are named `<profile>_<start>__<svo stamp>.mcap` and the SVO is `<svo stamp>.svo2`,
    so the suffix after the double underscore is the key. Roots are searched recursively because
    the May fights live in `svo/NHRL_2026-05-02/` but the 05-01 one is in `svo/tests/`.
    """
    stem = Path(source_mcap).stem
    if "__" not in stem:
        return None
    wanted = f"{stem.rsplit('__', 1)[1]}.svo2"
    for root in roots:
        matches = sorted(root.rglob(wanted)) if root.is_dir() else []
        if matches:
            return matches[0]
    return None


def gt_frame_indices(subdataset: Path) -> dict[int, int]:
    """{stamp_ns: svo frame index} for every labelled frame with a usable pose."""
    transforms = subdataset / "camera_transforms"
    labels = subdataset / "labels"
    if not transforms.exists():
        return {}
    out: dict[int, int] = {}
    for path in sorted(transforms.glob("*.json")):
        record = json.loads(path.read_text())
        if record.get("method") not in USABLE_METHODS:
            continue
        stamp = int(record["stamp_ns"])
        if not (labels / f"{stamp}.txt").exists():
            continue
        out[stamp] = int(record["image_stream_index"])
    return out


def extract(svo_path: Path, wanted: dict[int, int], out_path: Path, depth_mode: str) -> dict:
    """Grab depth at each wanted SVO index. Seeks forward in index order, never backwards."""
    import pyzed.sl as sl  # type: ignore[import-untyped]

    init = sl.InitParameters()
    init.set_from_svo_file(str(svo_path))
    init.depth_mode = getattr(sl.DEPTH_MODE, depth_mode)
    init.coordinate_units = sl.UNIT.METER
    init.depth_minimum_distance = DEPTH_MIN_M
    init.sdk_verbose = 0

    camera = sl.Camera()
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        return {"error": str(status)}

    total = camera.get_svo_number_of_frames()
    runtime = sl.RuntimeParameters()
    depth_mat = sl.Mat()

    payload: dict[str, np.ndarray] = {}
    validity: list[float] = []
    missed = 0
    for stamp, index in sorted(wanted.items(), key=lambda kv: kv[1]):
        if index < 0 or index >= total:
            missed += 1
            continue
        camera.set_svo_position(index)
        if camera.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            missed += 1
            continue
        camera.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
        depth = depth_mat.get_data().astype(np.float16)
        payload[str(stamp)] = depth
        validity.append(float(np.isfinite(depth).mean()))

    camera.close()
    if not payload:
        return {"error": "no frames grabbed", "missed": missed}

    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out_path, **payload)  # type: ignore[arg-type]
    sample = next(iter(payload.values()))
    return {
        "frames": len(payload),
        "missed": missed,
        "shape": list(sample.shape),
        "mean_validity": round(float(np.mean(validity)), 4),
        "depth_mode": depth_mode,
        "svo": str(svo_path),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("gt", type=Path, help="eval dataset root")
    parser.add_argument("--svo-roots", type=Path, nargs="+", default=[Path("data/svo")])
    parser.add_argument("-o", "--output", type=Path, required=True, help="cache dir")
    parser.add_argument(
        "--depth-mode",
        default=DEFAULT_DEPTH_MODE,
        help="ZED DEPTH_MODE name (NEURAL_PLUS, NEURAL, NEURAL_LIGHT, ULTRA, ...)",
    )
    args = parser.parse_args()

    subdatasets = sorted(d for d in args.gt.iterdir() if (d / "data.yaml").exists())
    if not subdatasets:
        raise SystemExit(f"No data.yaml found under {args.gt}")

    report: dict[str, dict] = {}
    for subdataset in subdatasets:
        name = subdataset.name
        wanted = gt_frame_indices(subdataset)
        if not wanted:
            print(f"{name}: no usable poses, skipped")
            report[name] = {"error": "no usable poses"}
            continue

        data = yaml.safe_load((subdataset / "data.yaml").read_text())
        svo = resolve_svo(str(data.get("source_mcap", "")), list(args.svo_roots))
        if svo is None:
            print(f"{name}: no SVO found, skipped")
            report[name] = {"error": "svo not found"}
            continue

        out_path = args.output / f"{name}.npz"
        if out_path.exists():
            print(f"{name}: cached already, skipped")
            report[name] = {"cached": True}
            continue

        print(f"{name}: {len(wanted)} frames from {svo.name} ...", flush=True)
        result = extract(svo, wanted, out_path, args.depth_mode)
        report[name] = result
        print(f"  {result}")

    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "cache_report.json").write_text(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
