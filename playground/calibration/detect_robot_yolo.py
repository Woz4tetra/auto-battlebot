"""Run the YOLO pose engine over a sysid recording's video and write our robot's detections.

One row per ``MR_STABS_MK2`` detection: the box, its confidence, and the front and back keypoints
in pixels. A frame can have several, because the model sometimes labels the floor marker board as
the robot; ``smooth_tag_poses.py --yolo`` drops those with size gates and keeps the best survivor
per frame, projects it onto the field, and uses it where the tag drops out. Kept as its own stage
so the detections can be inspected, and so the smoother does not need TensorRT.

    source scripts/activate_python.sh
    python playground/calibration/detect_robot_yolo.py data/recordings/<session>.mcap \\
        --out playground/calibration/out/mujoco_sysid/<session>/yolo.csv
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import pandas as pd

from auto_battlebot.perception.trt_yolo import TrtYoloModel
from auto_battlebot.recording.mcap_io import iter_video_frames

REPO_ROOT = Path(__file__).resolve().parents[2]
# Same candidates and class order as config/_desktop.toml [keypoint_model].
ENGINE_CANDIDATES = [
    "data/models/yolo26l-pose_all_keypoints_2026-09-25_rect416x640_x86_64_sm89.engine",
    "data/models/yolo26l-pose_all_keypoints_2026-09-25_rect416x640_x86_64_sm86.engine",
]
LABELS = ["MR_STABS_MK2", "MRS_BUFF_MK3", "OPPONENT", "HOUSE_BOT"]
OUR_ROBOT = LABELS.index("MR_STABS_MK2")


def default_engine() -> Path | None:
    for candidate in ENGINE_CANDIDATES:
        path = REPO_ROOT / candidate
        if path.exists():
            return path
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("mcap", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--engine", type=Path, default=None, help="default: first engine found")
    parser.add_argument("--conf", type=float, default=0.25, help="lowest confidence written")
    args = parser.parse_args()

    engine = args.engine or default_engine()
    if engine is None:
        raise SystemExit(f"no engine found among {ENGINE_CANDIDATES}; pass --engine")
    model = TrtYoloModel(str(engine), conf_threshold=args.conf, num_classes=len(LABELS))
    print(f"{engine.name}: {model.describe()}", flush=True)

    rows = []
    frames = 0
    start = time.monotonic()
    for stamp_ns, image in iter_video_frames(args.mcap):
        if stamp_ns == 0:
            continue
        frames += 1
        for box, conf, cls, keypoints in model.infer(image):
            if cls != OUR_ROBOT:
                continue
            kp = np.asarray(keypoints, dtype=np.float64).reshape(-1, 3)
            rows.append(
                {
                    "stamp_ns": int(stamp_ns),
                    "conf": float(conf),
                    "x1": float(box[0]),
                    "y1": float(box[1]),
                    "x2": float(box[2]),
                    "y2": float(box[3]),
                    "front_u": kp[0, 0],
                    "front_v": kp[0, 1],
                    "front_conf": kp[0, 2],
                    "back_u": kp[1, 0],
                    "back_v": kp[1, 1],
                    "back_conf": kp[1, 2],
                }
            )
        if frames % 5000 == 0:
            print(f"{frames} frames, {time.monotonic() - start:.0f} s", flush=True)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_csv(args.out, index=False)
    print(f"wrote {args.out}: {len(rows)} detections in {frames} frames", flush=True)


if __name__ == "__main__":
    main()
