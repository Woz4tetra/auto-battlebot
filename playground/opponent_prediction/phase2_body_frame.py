"""Plan step 7 prototype: the body-frame opponent EKF, scored against phase 1 and the baselines.

State `[x, y, u, s, theta, omega]`: forward speed `u` and lateral speed `s` in the body frame, so
the predicted path bends with the yaw rate instead of running straight. The measurement rows
(x, y, theta) sit at the same indices as phase 1, so gating, flips and rewind are the mirror's.
Propagation is an EKF in 10 ms substeps. A heading re-initialization rotates (u, s) so the world
velocity survives it. Lateral speed gets its own process noise (`lateral_accel_psd`), small for a
two-wheel robot.

The gate (fixed before running this): median rendered position error at the 0.1 s and 0.35 s
bins below both `hold` and `kalman4`, on the held-out session, natural gaps and 1 s gaps.

Usage:
    source scripts/activate_python.sh
    python playground/opponent_prediction/phase2_body_frame.py
"""

from __future__ import annotations

import itertools
import json
import math

import numpy as np
import torch

from auto_battlebot.perception.opponent_filter import (
    MAX_INTEGRATION_STEP_S,
    OMEGA,
    PX,
    PY,
    THETA,
    OpponentFilter,
    OpponentFilterConfig,
    RenderedEstimate,
    wrap_angle,
)
from auto_battlebot.perception.sysid_sessions import OUT, load_session
from playground.opponent_prediction import evaluate_opponent_prediction as ev

U, S = 2, 3
SUBSTEP_S = 0.01


class BodyFrameFilter(OpponentFilter):
    """Phase 2: body-frame velocity, coupled to heading in prediction."""

    def __init__(
        self,
        batch: int,
        config: OpponentFilterConfig,
        lateral_accel_psd: float | torch.Tensor,
        speed_tau_s: float | torch.Tensor = math.inf,
    ) -> None:
        super().__init__(batch, config)
        self.lateral_accel_psd = torch.as_tensor(lateral_accel_psd, dtype=self.dtype).expand(batch)
        # Optional first-order decay of the body velocity while coasting; inf is none.
        self.speed_tau_s = torch.as_tensor(speed_tau_s, dtype=self.dtype).expand(batch)

    def _substep(
        self, x: torch.Tensor, p: torch.Tensor, h: torch.Tensor, move: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """One Euler substep of length h (B,); the state moves only where `move`."""
        b = self.batch
        th, u, s = x[:, THETA], x[:, U], x[:, S]
        c, sn = torch.cos(th), torch.sin(th)
        decay = torch.exp(-h / self.speed_tau_s)
        f = torch.eye(6, dtype=self.dtype).repeat(b, 1, 1)
        f[:, PX, THETA] = (-u * sn - s * c) * h
        f[:, PX, U] = c * h
        f[:, PX, S] = -sn * h
        f[:, PY, THETA] = (u * c - s * sn) * h
        f[:, PY, U] = sn * h
        f[:, PY, S] = c * h
        f[:, THETA, OMEGA] = h
        f[:, U, U] = decay
        f[:, S, S] = decay
        q = torch.zeros(b, 6, 6, dtype=self.dtype)
        q[:, U, U] = self.opponent_accel_psd * h
        q[:, S, S] = self.lateral_accel_psd * h
        q[:, OMEGA, OMEGA] = self.opponent_yaw_accel_psd * h
        q[:, THETA, THETA] = self.opponent_yaw_accel_psd * h**3 / 3.0
        q[:, THETA, OMEGA] = q[:, OMEGA, THETA] = self.opponent_yaw_accel_psd * h**2 / 2.0
        nx = x.clone()
        nx[:, PX] = x[:, PX] + (u * c - s * sn) * h
        nx[:, PY] = x[:, PY] + (u * sn + s * c) * h
        nx[:, THETA] = wrap_angle(x[:, THETA] + x[:, OMEGA] * h)
        nx[:, U] = u * decay
        nx[:, S] = s * decay
        nx = torch.where(move[:, None], nx, x)
        p = f @ p @ f.transpose(1, 2) + q
        return nx, (p + p.transpose(1, 2)) * 0.5

    def _advance(self, target: torch.Tensor, mask: torch.Tensor) -> None:
        dt_total = target - self.stamp
        active = mask & (dt_total > 0.0)
        if not bool(active.any()):
            return
        dt = torch.clamp(dt_total, max=MAX_INTEGRATION_STEP_S)
        age = self.stamp - self.last_measured
        movable = torch.where(
            dt_total > MAX_INTEGRATION_STEP_S,
            torch.zeros_like(dt),
            torch.minimum(dt, torch.clamp(self.opponent_max_coast_s - age, min=0.0)),
        )
        x, p = self.x.clone(), self.p.clone()
        steps = int(math.ceil(float(dt[active].max()) / SUBSTEP_S))
        done = torch.zeros_like(dt)
        for _ in range(steps):
            h = torch.clamp(torch.minimum(dt - done, torch.full_like(dt, SUBSTEP_S)), min=0.0)
            move = (done + h <= movable + 1e-12) & (h > 0)
            x, p = self._substep(x, p, h, move)
            done = done + h
        for index, cap in (
            (U, self.initial_velocity_sigma**2),
            (S, self.initial_velocity_sigma**2),
            (THETA, torch.full_like(dt, math.pi**2)),
            (OMEGA, self.initial_yaw_rate_sigma**2),
        ):
            v = p[:, index, index]
            scale = torch.where(v > cap, torch.sqrt(cap / torch.clamp(v, min=1e-300)), 1.0)
            p[:, index, :] *= scale[:, None]
            p[:, :, index] *= scale[:, None]
        self.x = torch.where(active[:, None], x, self.x)
        self.p = torch.where(active[:, None, None], p, self.p)
        self.stamp = torch.where(active, target, self.stamp)
        self._push_snapshot(active)

    def _init_heading(self, mask: torch.Tensor, yaw: torch.Tensor) -> None:
        rows = mask.nonzero(as_tuple=True)[0]
        if rows.numel() == 0:
            return
        # Keep the world velocity: rotate (u, s) from the old heading to the new one.
        old = self.x[rows, THETA]
        new = wrap_angle(yaw[rows])
        u, s = self.x[rows, U], self.x[rows, S]
        vx = u * torch.cos(old) - s * torch.sin(old)
        vy = u * torch.sin(old) + s * torch.cos(old)
        known = self.heading_initialized[rows]
        self.x[rows, U] = torch.where(known, vx * torch.cos(new) + vy * torch.sin(new), u)
        self.x[rows, S] = torch.where(known, -vx * torch.sin(new) + vy * torch.cos(new), s)
        super()._init_heading(mask, yaw)

    def render(self, stale: torch.Tensor) -> RenderedEstimate:
        out = super().render(stale)
        th = self.x[:, THETA]
        u, s = self.x[:, U], self.x[:, S]
        out.vx = u * torch.cos(th) - s * torch.sin(th)
        out.vy = u * torch.sin(th) + s * torch.cos(th)
        return out


def gate_rows(data, traces: dict[str, ev.Trace], scenario_index: dict[str, int]) -> dict:
    """Median position error (mm) at the 0.1 and 0.35 s bins, per arm and scenario."""
    out: dict = {}
    for arm, (trace, row_of) in traces.items():
        for scen, r in row_of.items():
            s = ev.score_trace(data, trace, r)
            out.setdefault(scen, {})[arm] = {
                b: (s["by_gap"][b].get("pos_p50_mm"), s["by_gap"][b].get("pos_p90_mm"))
                for b in ("0", "0.1", "0.35")
            } | {"yaw_0.35": s["by_gap"]["0.35"].get("yaw_p50_deg")}
    return out


def main() -> None:
    torch.set_num_threads(6)
    cfg, sigma = ev.tuned_config()
    candidates = []
    for coast, lat, tau in itertools.product([0.05, 0.1, 0.2], [0.3, 3.0, 30.0], [math.inf, 0.15]):
        candidates.append({"coast": coast, "lateral_psd": lat, "tau": tau})
    report: dict = {}
    for name in ("2026-10-15_00-29-00", "2026-10-15_00-20-58"):
        data = load_session(name)
        k = ev.scenario_keep(data)
        scen = {"natural": 0, "gap_1": 2}
        keep_base = np.stack([k[i] for i in scen.values()])
        b = len(candidates) * len(scen)
        keep = np.concatenate([keep_base] * len(candidates))
        c = OpponentFilterConfig(**{**vars(cfg)})
        c.opponent_max_coast_s = torch.tensor(
            [cand["coast"] for cand in candidates for _ in scen], dtype=torch.float64
        )
        filt = BodyFrameFilter(
            b,
            c,
            torch.tensor([cand["lateral_psd"] for cand in candidates for _ in scen]),
            torch.tensor([cand["tau"] for cand in candidates for _ in scen]),
        )
        trace = ev.simulate_filter(data, filt, keep, sigma, record_stride=2)
        base = {
            "hold": (ev.simulate_hold(data, keep_base, cfg.render_lead_s, record_stride=2), scen),
            "kalman4": (
                ev.simulate_filter(
                    data, ev.LegacyOpponentFilter(2, cfg), keep_base, 0.03, record_stride=2
                ),
                scen,
            ),
            "phase1": (
                ev.simulate_filter(data, OpponentFilter(2, cfg), keep_base, sigma, record_stride=2),
                scen,
            ),
        }
        base_rows = {s: j for j, s in enumerate(scen)}
        rows = gate_rows(data, {a: (t, base_rows) for a, (t, _) in base.items()}, scen)
        for i, cand in enumerate(candidates):
            label = f"phase2 coast={cand['coast']} lat={cand['lateral_psd']} tau={cand['tau']}"
            rows_c = gate_rows(
                data, {label: (trace, {s: i * len(scen) + j for j, s in enumerate(scen)})}, scen
            )
            for s in rows_c:
                rows[s].update(rows_c[s])
        report[name] = rows
        print(name)
        for s, arms in rows.items():
            print(" ", s)
            for arm, v in arms.items():
                print(
                    f"    {arm:42s} 0:{v['0'][0]:5.0f}/{v['0'][1]:4.0f}  0.1:{v['0.1'][0]:5.0f}"
                    f"/{v['0.1'][1]:4.0f}  0.35:{v['0.35'][0]:5.0f}/{v['0.35'][1]:4.0f}"
                    f"  yaw0.35 {v['yaw_0.35']:.0f}"
                )
    (OUT / "phase2_gate.json").write_text(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
