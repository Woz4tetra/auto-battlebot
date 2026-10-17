"""Plan step 3 gate: the Warp firmware kernel against the scalar `FirmwareMixer` on logged sticks.

Feeds the 2026-10-15 sysid sessions' logged stick percents, BNO055 heading and gyro rate (the
ESP32 telemetry, 100 Hz) through both the scalar mirror (`auto_battlebot/mujoco_sim/firmware.py`)
and the Warp port (`auto_battlebot/rl/envs/warp_kernels.py`), and reports the per-motor RMS
difference in percent of full command. The gate is under 0.5%.

    source scripts/activate_python.sh
    python playground/rl/check_warp_firmware.py
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import warp as wp

from auto_battlebot.mujoco_sim.firmware import FirmwareMixer
from auto_battlebot.recording.sysid_io import load_esp32_diagnostics
from auto_battlebot.rl.envs.warp_kernels import FW_COLUMNS, firmware_test

REPO = Path(__file__).resolve().parents[2]
SESSIONS = (
    "data/recordings/auto_battlebot_mr_stabs_mk2_sysid_zed_box_2026-10-15_00-20-58.mcap",
    "data/recordings/auto_battlebot_mr_stabs_mk2_sysid_zed_box_2026-10-15_00-29-00.mcap",
)
DT = 0.01


def armed_spans(df: object) -> list[tuple[int, int]]:
    armed = np.asarray(df["armed"], dtype=bool)  # type: ignore[index]
    edges = np.flatnonzero(np.diff(np.r_[0, armed.astype(int), 0]))
    return [(int(a), int(b)) for a, b in zip(edges[::2], edges[1::2]) if b - a > 200]


def main() -> None:
    wp.init()
    report = {}
    for path in SESSIONS:
        df = load_esp32_diagnostics(REPO / path)
        rows = []
        for a, b in armed_spans(df):
            seg = df.iloc[a:b]
            rows.append(
                (
                    seg["a_percent"].to_numpy(float),
                    seg["b_percent"].to_numpy(float),
                    seg["orientation_x"].to_numpy(float),
                    np.nan_to_num(seg["yaw_rate"].to_numpy(float)),
                )
            )
        length = max(len(r[0]) for r in rows)
        w = len(rows)
        arr = np.zeros((4, w, length))
        valid = np.zeros((w, length), dtype=bool)
        for i, r in enumerate(rows):
            n = len(r[0])
            for k in range(4):
                arr[k, i, :n] = r[k]
            valid[i, :n] = True

        scalar = np.zeros((w, length, 2))
        for i in range(w):
            mixer = FirmwareMixer(auto_steer=True, sample_period=DT)
            for t in range(int(valid[i].sum())):
                scalar[i, t] = mixer.step(
                    arr[0, i, t], arr[1, i, t], arr[2, i, t], DT, yaw_rate_dps=arr[3, i, t]
                )

        fw = wp.zeros((w, FW_COLUMNS), dtype=float)
        out = wp.zeros((w, length, 2), dtype=float)
        a, b, h, r = (wp.array(arr[k].astype(np.float32), dtype=float) for k in range(4))
        wp.launch(firmware_test, dim=w, inputs=[a, b, h, r, DT, fw], outputs=[out])
        kernel = out.numpy()
        diff = (kernel - scalar)[valid]
        rms = np.sqrt((diff**2).mean(axis=0))
        report[Path(path).stem] = {
            "spans": w,
            "samples": int(valid.sum()),
            "rms_percent_left": float(rms[0]),
            "rms_percent_right": float(rms[1]),
            "max_abs_percent": float(np.abs(diff).max()),
        }
    worst = max(max(v["rms_percent_left"], v["rms_percent_right"]) for v in report.values())
    report["gate_rms_under_0.5_percent"] = bool(worst < 0.5)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
