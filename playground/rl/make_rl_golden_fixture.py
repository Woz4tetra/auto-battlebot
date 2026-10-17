"""Write the observation-spec golden fixture the C++ builder is tested against (plan gate G3).

Each case is one world's `ObsInputs` and the 138 floats `build_v1` makes from it in float64.
`tests/rl_policy/test_observation_v1.cpp` feeds the same inputs to `ObservationBuilderV1` and
requires agreement to 1e-5. Edge cases first (no target, stale target with unknown sigmas, no
hazards, our robot against a wall, starting inside a hole, more than four hazards, a tie in edge
distance), then random worlds.

    source scripts/activate_python.sh
    python playground/rl/make_rl_golden_fixture.py
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import torch

from auto_battlebot.rl.obs_spec import (
    HOLE,
    OBS_DIM,
    OBS_SPEC_VERSION,
    TRACKED,
    WALL_BLOCK,
    ObsInputs,
    build_v1,
)

REPO = Path(__file__).resolve().parents[2]
OUT = REPO / "tests/rl_policy/rl_policy_golden_data.hpp"


def case(rng: np.random.Generator, **over: object) -> dict[str, object]:
    half = np.array([1.2, 1.0])
    c: dict[str, object] = {
        "our_xy": rng.uniform(-0.9, 0.9, 2) * half,
        "our_yaw": rng.uniform(-math.pi, math.pi),
        "our_speed": rng.uniform(-2.0, 3.0),
        "our_yaw_rate": rng.uniform(-20.0, 20.0),
        "our_age_s": rng.uniform(0.0, 0.8),
        "our_stale": bool(rng.random() < 0.2),
        "run_away": bool(rng.random() < 0.3),
        "hist_action": rng.uniform(-1, 1, (8, 2)),
        "hist_speed": rng.uniform(-2, 3, 8),
        "hist_yaw_rate": rng.uniform(-20, 20, 8),
        "opp_present": True,
        "opp_xy": rng.uniform(-0.9, 0.9, 2) * half,
        "opp_yaw": rng.uniform(-math.pi, math.pi),
        "opp_velocity": rng.uniform(-2, 2, 2),
        "opp_yaw_rate": rng.uniform(-10, 10),
        "opp_position_sigma": rng.uniform(0.0, 0.2),
        "opp_heading_sigma": rng.uniform(0.0, 2.5),
        "opp_yaw_rate_sigma": rng.uniform(0.0, 20.0),
        "opp_half_diagonal": rng.uniform(0.1, 0.25),
        "opp_age_s": rng.uniform(0.0, 0.8),
        "opp_stale": bool(rng.random() < 0.3),
        "field_half": half,
        "hazards": [
            (
                *(rng.uniform(-0.9, 0.9, 2) * half),
                rng.uniform(0.05, 0.3),
                int(rng.integers(0, 3)),
                *rng.uniform(-1, 1, 2),
            )
            for _ in range(int(rng.integers(0, 7)))
        ],
    }
    c.update(over)
    return c


def cases() -> list[tuple[str, dict[str, object]]]:
    rng = np.random.default_rng(20261017)
    out: list[tuple[str, dict[str, object]]] = [
        ("no target", case(rng, opp_present=False)),
        (
            "stale target, heading never measured",
            case(rng, opp_stale=True, opp_heading_sigma=math.inf, opp_yaw_rate_sigma=math.inf),
        ),
        ("no hazards", case(rng, hazards=[])),
        (
            "against the +x wall, facing it",
            case(rng, our_xy=np.array([1.2, 0.1]), our_yaw=0.0, hazards=[]),
        ),
        (
            "inside a hole",
            case(rng, our_xy=np.array([0.3, 0.3]), hazards=[(0.35, 0.3, 0.2, HOLE, 0.0, 0.0)]),
        ),
        (
            "six hazards, one of each kind twice",
            case(
                rng,
                our_xy=np.array([0.0, 0.0]),
                our_yaw=0.3,
                hazards=[
                    (0.5, 0.0, 0.1, HOLE, 0.0, 0.0),
                    (-0.6, 0.2, 0.2, WALL_BLOCK, 0.0, 0.0),
                    (0.0, 0.7, 0.15, TRACKED, 0.3, -0.2),
                    (0.9, 0.9, 0.3, HOLE, 0.0, 0.0),
                    (-0.9, -0.8, 0.1, WALL_BLOCK, 0.0, 0.0),
                    (0.2, -0.5, 0.12, TRACKED, -0.5, 0.0),
                ],
            ),
        ),
        (
            "tie in edge distance",
            case(
                rng,
                our_xy=np.array([0.0, 0.0]),
                hazards=[
                    (0.5, 0.0, 0.1, HOLE, 0.0, 0.0),
                    (0.0, 0.5, 0.1, WALL_BLOCK, 0.0, 0.0),
                    (-0.5, 0.0, 0.1, TRACKED, 0.0, 0.0),
                    (0.0, -0.5, 0.1, HOLE, 0.0, 0.0),
                    (0.0, 0.5, 0.1, TRACKED, 0.0, 0.0),
                ],
            ),
        ),
        ("opponent on top of us", case(rng, opp_xy=None)),
    ]
    out[-1][1]["opp_xy"] = np.array(out[-1][1]["our_xy"])  # type: ignore[arg-type]
    for i in range(16):
        out.append((f"random {i}", case(rng)))
    return out


def to_inputs(c: dict[str, object]) -> ObsInputs:
    def t(x: object) -> torch.Tensor:
        return torch.as_tensor(np.asarray(x, dtype=float), dtype=torch.float64)[None]

    def b(x: object) -> torch.Tensor:
        return torch.tensor([bool(x)])

    hz = np.asarray(c["hazards"], dtype=float).reshape(-1, 6)
    return ObsInputs(
        our_xy=t(c["our_xy"]),
        our_yaw=t(c["our_yaw"]),
        our_speed=t(c["our_speed"]),
        our_yaw_rate=t(c["our_yaw_rate"]),
        our_age_s=t(c["our_age_s"]),
        our_stale=b(c["our_stale"]),
        run_away=b(c["run_away"]),
        hist_action=t(c["hist_action"]),
        hist_speed=t(c["hist_speed"]),
        hist_yaw_rate=t(c["hist_yaw_rate"]),
        opp_present=b(c["opp_present"]),
        opp_xy=t(c["opp_xy"]),
        opp_yaw=t(c["opp_yaw"]),
        opp_velocity=t(c["opp_velocity"]),
        opp_yaw_rate=t(c["opp_yaw_rate"]),
        opp_position_sigma=t(c["opp_position_sigma"]),
        opp_heading_sigma=t(c["opp_heading_sigma"]),
        opp_yaw_rate_sigma=t(c["opp_yaw_rate_sigma"]),
        opp_half_diagonal=t(c["opp_half_diagonal"]),
        opp_age_s=t(c["opp_age_s"]),
        opp_stale=b(c["opp_stale"]),
        field_half=t(c["field_half"]),
        hazard_xy=t(hz[:, 0:2]),
        hazard_radius=t(hz[:, 2]),
        hazard_kind=torch.as_tensor(hz[:, 3], dtype=torch.long)[None],
        hazard_velocity=t(hz[:, 4:6]),
        hazard_present=torch.ones(1, hz.shape[0], dtype=torch.bool),
    )


def cpp_double(x: float) -> str:
    if math.isinf(x):
        return "std::numeric_limits<double>::infinity()"
    return repr(float(x))


def main() -> None:
    lines = [
        "// Generated by playground/rl/make_rl_golden_fixture.py. Do not edit.",
        "#pragma once",
        "",
        "#include <array>",
        "#include <limits>",
        "#include <vector>",
        "",
        "namespace auto_battlebot::rl_golden {",
        "",
        f"constexpr int kObsSpecVersion = {OBS_SPEC_VERSION};",
        f"constexpr int kObsDim = {OBS_DIM};",
        "",
        "struct Hazard {",
        "    double x, y, radius;",
        "    int kind;  // 0 hole, 1 wall block, 2 tracked",
        "    double vx, vy;",
        "};",
        "",
        "struct Case {",
        "    const char *name;",
        "    double our_x, our_y, our_yaw, our_speed, our_yaw_rate, our_age_s;",
        "    bool our_stale, run_away;",
        "    std::array<double, 16> hist_action;  // newest first: linear, angular per step",
        "    std::array<double, 8> hist_speed, hist_yaw_rate;",
        "    bool opp_present;",
        "    double opp_x, opp_y, opp_yaw, opp_vx, opp_vy, opp_yaw_rate;",
        "    double opp_position_sigma, opp_heading_sigma, opp_yaw_rate_sigma;",
        "    double opp_half_diagonal, opp_age_s;",
        "    bool opp_stale;",
        "    double field_half_x, field_half_y;",
        "    std::vector<Hazard> hazards;",
        f"    std::array<double, {OBS_DIM}> expected;",
        "};",
        "",
        "inline const std::vector<Case> &cases() {",
        "    static const std::vector<Case> kCases = {",
    ]
    for name, c in cases():
        obs = build_v1(to_inputs(c))[0].tolist()
        f = cpp_double
        hz = np.asarray(c["hazards"], dtype=float).reshape(-1, 6)
        ha = np.asarray(c["hist_action"], dtype=float).reshape(-1)
        lines.append("        Case{")
        lines.append(f'            "{name}",')
        ox, oy = np.asarray(c["our_xy"], dtype=float)
        lines.append(
            f"            {f(ox)}, {f(oy)}, {f(c['our_yaw'])}, {f(c['our_speed'])}, "  # type: ignore[arg-type]
            f"{f(c['our_yaw_rate'])}, {f(c['our_age_s'])},"  # type: ignore[arg-type]
        )
        lines.append(f"            {str(c['our_stale']).lower()}, {str(c['run_away']).lower()},")
        lines.append("            {" + ", ".join(f(x) for x in ha) + "},")
        lines.append(
            "            {"
            + ", ".join(f(x) for x in np.asarray(c["hist_speed"], dtype=float))
            + "},"
        )
        lines.append(
            "            {"
            + ", ".join(f(x) for x in np.asarray(c["hist_yaw_rate"], dtype=float))
            + "},"
        )
        px, py = np.asarray(c["opp_xy"], dtype=float)
        vx, vy = np.asarray(c["opp_velocity"], dtype=float)
        lines.append(f"            {str(c['opp_present']).lower()},")
        lines.append(
            f"            {f(px)}, {f(py)}, {f(c['opp_yaw'])}, "  # type: ignore[arg-type]
            f"{f(vx)}, {f(vy)}, {f(c['opp_yaw_rate'])},"  # type: ignore[arg-type]
        )
        lines.append(
            f"            {f(c['opp_position_sigma'])}, {f(c['opp_heading_sigma'])}, "  # type: ignore[arg-type]
            f"{f(c['opp_yaw_rate_sigma'])},"  # type: ignore[arg-type]
        )
        lines.append(f"            {f(c['opp_half_diagonal'])}, {f(c['opp_age_s'])},")  # type: ignore[arg-type]
        lines.append(f"            {str(c['opp_stale']).lower()},")
        hx, hy = np.asarray(c["field_half"], dtype=float)
        lines.append(f"            {f(hx)}, {f(hy)},")
        lines.append(
            "            {"
            + ", ".join(
                "Hazard{"
                + f"{f(r[0])}, {f(r[1])}, {f(r[2])}, {int(r[3])}, {f(r[4])}, {f(r[5])}"
                + "}"
                for r in hz
            )
            + "},"
        )
        lines.append("            {" + ", ".join(repr(float(v)) for v in obs) + "},")
        lines.append("        },")
    lines += [
        "    };",
        "    return kCases;",
        "}",
        "",
        "}  // namespace auto_battlebot::rl_golden",
        "",
    ]
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(lines))
    print(f"wrote {len(cases())} cases to {OUT}")


if __name__ == "__main__":
    main()
