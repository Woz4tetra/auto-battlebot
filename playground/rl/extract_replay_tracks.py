"""Extract human opponent tracks from the KALMAN match replays for the `replay` behaviour.

Reads the filtered opponent poses (`/robot_markers`, THEIR_ROBOT_*) from the NHRL May and MassD
match replays in `playground/opponent_prediction/out/replays/`, splits them into continuous
segments (no gap over 0.25 s, at least 3 s long), resamples each to 50 Hz, and stores positions
as fractions of the recording's field half size so the environment can scale them into any arena.

Two whole recordings (one per venue) are the evaluation set, never trained on: the plan holds out
20% of the replay tracks (section 6), and holding out recordings rather than segments keeps one
driver's habits from leaking across the split.

Output: `simulation/opponent_tracks/replay_tracks.npz`.

    source scripts/activate_python.sh
    python playground/rl/extract_replay_tracks.py
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from auto_battlebot.recording.diag_io import (
    ROBOT_MARKERS_TOPIC,
    decode_scene_update,
    frame_name,
    group_of,
    load_field_size,
)
from auto_battlebot.recording.mcap_io import iter_messages

REPO = Path(__file__).resolve().parents[2]
REPLAYS = REPO / "playground/opponent_prediction/out/replays"
OUT = REPO / "simulation/opponent_tracks/replay_tracks.npz"
HOLDOUT = ("2026-05-02T17-26-14_KALMAN", "2026-08-29T15-21-23_KALMAN")
RATE_HZ = 50.0
MAX_GAP_S = 0.25
MIN_LEN_S = 3.0


def opponent_samples(path: Path) -> dict[str, np.ndarray]:
    rows: dict[str, list[tuple[float, float, float, float]]] = {}
    for _topic, ts, data in iter_messages(path, [ROBOT_MARKERS_TOPIC]):
        for e in decode_scene_update(data).entities:
            if e.namespace != "robot_bounds" or not e.cubes:
                continue
            name = frame_name(e.index)
            if group_of(name) != "THEIRS":
                continue
            p = e.cubes[0].pose
            qx, qy, qz, qw = p.orientation
            yaw = float(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
            rows.setdefault(name, []).append(
                (ts / 1e9, float(p.position[0]), float(p.position[1]), yaw)
            )
    return {k: np.array(sorted(v)) for k, v in rows.items()}


def segments(samples: np.ndarray) -> list[np.ndarray]:
    t = samples[:, 0]
    breaks = np.flatnonzero(np.diff(t) > MAX_GAP_S) + 1
    out = []
    for seg in np.split(samples, breaks):
        if seg.shape[0] < 2 or seg[-1, 0] - seg[0, 0] < MIN_LEN_S:
            continue
        grid = np.arange(seg[0, 0], seg[-1, 0], 1.0 / RATE_HZ)
        yaw = np.unwrap(seg[:, 3])
        out.append(
            np.stack(
                [
                    np.interp(grid, seg[:, 0], seg[:, 1]),
                    np.interp(grid, seg[:, 0], seg[:, 2]),
                    np.interp(grid, seg[:, 0], yaw),
                ],
                1,
            )
        )
    return out


def main() -> None:
    tracks: list[np.ndarray] = []
    holdout: list[bool] = []
    names: list[str] = []
    report = {}
    for path in sorted(REPLAYS.glob("*_KALMAN.mcap")):
        size = load_field_size(path)
        if size is None:
            print(f"skip {path.name}: no field size")
            continue
        half = np.array(size) / 2.0
        count = 0
        for frame, samples in opponent_samples(path).items():
            for seg in segments(samples):
                scaled = seg.copy()
                scaled[:, 0] = np.clip(seg[:, 0] / half[0], -1.0, 1.0)
                scaled[:, 1] = np.clip(seg[:, 1] / half[1], -1.0, 1.0)
                tracks.append(scaled.astype(np.float32))
                holdout.append(path.stem in HOLDOUT)
                names.append(f"{path.stem}:{frame}")
                count += 1
        report[path.stem] = {"field_m": [round(s, 3) for s in size], "segments": count}
    lengths = np.array([len(t) for t in tracks])
    padded = np.zeros((len(tracks), int(lengths.max()), 3), dtype=np.float32)
    for i, t in enumerate(tracks):
        padded[i, : len(t)] = t
        padded[i, len(t) :] = t[-1]
    OUT.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        OUT,
        tracks=padded,
        lengths=lengths,
        holdout=np.array(holdout),
        names=np.array(names),
        rate_hz=RATE_HZ,
    )
    report["total_segments"] = len(tracks)
    report["holdout_segments"] = int(np.sum(holdout))
    report["seconds"] = float(lengths.sum() / RATE_HZ)
    print(json.dumps(report, indent=2))
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
