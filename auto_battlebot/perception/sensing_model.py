"""Batched sensing model: what perception gets wrong, for the RL environment and the lockstep sim.

`docs/plans/sensing_model_plan.md` holds the design. Two pieces:

- `MeasurementModel` turns true poses into the measurement stream the filters receive: a frame
  clock with frame drops, a semi-Markov dropout process per robot conditioned on the distance to
  the nearest robot and wall, body-frame position noise and bias by speed, heading noise with
  180 degree flips, a per-episode field offset, and a capture-to-delivery latency.
- `OurEkfErrorModel` stands in for the rendered our-robot EKF output, which has no Python mirror:
  truth plus an AR(1) error per component, coasting at constant velocity and turn rate through a
  measurement gap and holding at `max_coast_s`.

Both are vectorized over worlds in torch, so the RL environment runs one per world on the GPU and
the sim server runs one world on the CPU. Each instance owns its `torch.Generator`, so a seed
reproduces a run on the same device and batch shape.

The fitted parameters live in `simulation/sensing_models/<venue>.toml`, written by
`playground/sensing_model/fit_sensing_model.py`.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

import numpy as np
import torch

from auto_battlebot.compat import tomllib

SPEC_VERSION = 1
EKF_COMPONENTS = ("along", "cross", "theta", "v", "omega")
# A gap in our robot's measurements starts once the last one is this many frame periods old;
# shorter waits are the normal spacing between frames and are part of the fitted EKF error.
GAP_MIN_FRAMES = 2.5
CLOCK_EPS_S = 1e-6


class FlipModel(Enum):
    INDEPENDENT = "independent"
    BURSTY = "bursty"


def _enum(cls: type[FlipModel], value: Any, where: str) -> FlipModel:
    try:
        return cls(value)
    except ValueError:
        valid = ", ".join(m.value for m in cls)
        raise ValueError(f"{where}: {value!r} is not one of {valid}") from None


def _bool(value: Any, where: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{where}: {value!r} is not true or false")
    return value


def _array(value: Any, where: str, size: int | None = None) -> np.ndarray:
    arr = np.asarray(value, dtype=float)
    if arr.ndim != 1 or (size is not None and arr.size != size):
        want = f"{size} values" if size is not None else "a list"
        raise ValueError(f"{where}: expected {want}, got {value!r}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{where}: values must be finite")
    return arr


def _edges(value: Any, where: str) -> np.ndarray:
    """Bin left edges: first edge 0, strictly increasing, last bin open-ended."""
    arr = _array(value, where)
    if arr.size == 0 or arr[0] != 0.0 or np.any(np.diff(arr) <= 0.0):
        raise ValueError(f"{where}: bin edges must start at 0 and increase, got {value!r}")
    return arr


def _range(value: Any, where: str) -> tuple[float, float]:
    arr = _array(value, where, 2)
    if arr[0] > arr[1]:
        raise ValueError(f"{where}: range low exceeds high, got {value!r}")
    return float(arr[0]), float(arr[1])


@dataclass(frozen=True)
class DurationTable:
    """Inverse CDFs of a state's duration, one row per context cell.

    `quantiles` runs from 0 to 1 inclusive, so the first and last columns are the observed minimum
    and maximum and a draw never leaves the observed range. Sampling interpolates in log duration,
    which keeps the heavy tail between p95 and p99 from being drawn as a straight line.
    """

    quantiles: np.ndarray
    values_s: np.ndarray

    @classmethod
    def parse(cls, quantiles: Any, rows: Any, cells: int, where: str) -> DurationTable:
        q = _array(quantiles, f"{where}.quantiles")
        if q.size < 2 or q[0] != 0.0 or q[-1] != 1.0 or np.any(np.diff(q) <= 0.0):
            raise ValueError(f"{where}.quantiles must run from 0 to 1 and increase")
        values = np.asarray(rows, dtype=float)
        if values.ndim == 1:
            values = values[None, :]
        if values.shape != (cells, q.size):
            raise ValueError(
                f"{where}: expected {cells} rows of {q.size} durations, got shape {values.shape}"
            )
        if not np.all(np.isfinite(values)) or np.any(values <= 0.0):
            raise ValueError(f"{where}: durations must be positive and finite")
        if np.any(np.diff(values, axis=1) < 0.0):
            raise ValueError(f"{where}: durations must not decrease along the quantiles")
        return cls(q, values)


@dataclass(frozen=True)
class DropoutSpec:
    """Measured-run and dropped-gap durations by (nearest robot, nearest wall) distance bin."""

    robot_distance_bins_m: np.ndarray
    wall_distance_bins_m: np.ndarray
    measured: DurationTable
    dropped: DurationTable
    borrowed: tuple[str, ...]
    # False delivers every captured frame: the robot never drops out.
    enabled: bool = True

    @property
    def cells(self) -> int:
        return self.robot_distance_bins_m.size * self.wall_distance_bins_m.size


@dataclass(frozen=True)
class EkfErrorSpec:
    """Our-robot EKF output error: AR(1) std by speed bin and correlation time per component."""

    sigma: np.ndarray  # [len(EKF_COMPONENTS), speed bins]
    tau_s: np.ndarray  # [len(EKF_COMPONENTS)]


@dataclass(frozen=True)
class RobotSpec:
    """Measurement model for one robot role (`ours`, `opponent`, ...)."""

    speed_bins_mps: np.ndarray
    sigma_along_m: np.ndarray
    sigma_cross_m: np.ndarray
    bias_along_m: np.ndarray
    bias_along_reverse_m: np.ndarray
    bias_cross_m: np.ndarray
    heading_sigma_rad: np.ndarray
    flip_model: FlipModel
    flip_rate: np.ndarray | None
    flip_runs: tuple[DurationTable, DurationTable] | None  # (normal, flipped)
    flip_scale_by_label: dict[str, float]
    dropout: DropoutSpec
    ekf_error: EkfErrorSpec | None


@dataclass(frozen=True)
class Randomization:
    """Per-episode draw ranges. Scales multiply the fitted values."""

    frame_rate_hz: tuple[float, float]
    latency_s: tuple[float, float]
    frame_drop_scale: tuple[float, float]
    error_scale: tuple[float, float]
    offset_sigma_m: float
    flip_scale: tuple[float, float]
    dropout_stretch: tuple[float, float]
    ekf_sigma_scale: tuple[float, float]
    ekf_tau_scale: tuple[float, float]


@dataclass(frozen=True)
class SensingModelFile:
    """A parsed and validated sensing model TOML."""

    venue: str
    frame_rate_hz: float
    latency_s: float
    frame_drop_prob: float
    robots: dict[str, RobotSpec]
    randomization: Randomization

    @classmethod
    def load(cls, path: Path) -> SensingModelFile:
        with open(path, "rb") as f:
            return cls.from_dict(tomllib.load(f), str(path))

    @classmethod
    def from_dict(cls, d: Mapping[str, Any], where: str = "sensing model") -> SensingModelFile:
        version = d.get("spec_version")
        if version != SPEC_VERSION:
            raise ValueError(f"{where}: spec_version {version!r}, this code reads {SPEC_VERSION}")
        robots = {
            name: _parse_robot(body, f"{where}.robots.{name}")
            for name, body in d.get("robots", {}).items()
        }
        if not robots:
            raise ValueError(f"{where}: no [robots.<role>] tables")
        r = d["randomization"]
        rnd = Randomization(
            frame_rate_hz=_range(r["frame_rate_hz"], f"{where}.randomization.frame_rate_hz"),
            latency_s=_range(r["latency_s"], f"{where}.randomization.latency_s"),
            frame_drop_scale=_range(r["frame_drop_scale"], "randomization.frame_drop_scale"),
            error_scale=_range(r["error_scale"], f"{where}.randomization.error_scale"),
            offset_sigma_m=float(r["offset_sigma_m"]),
            flip_scale=_range(r["flip_scale"], f"{where}.randomization.flip_scale"),
            dropout_stretch=_range(r["dropout_stretch"], "randomization.dropout_stretch"),
            ekf_sigma_scale=_range(r["ekf_sigma_scale"], "randomization.ekf_sigma_scale"),
            ekf_tau_scale=_range(r["ekf_tau_scale"], "randomization.ekf_tau_scale"),
        )
        if rnd.frame_rate_hz[0] <= 0.0 or rnd.latency_s[0] < 0.0:
            raise ValueError(f"{where}: frame rate must be positive and latency non-negative")
        return cls(
            venue=str(d.get("venue", "")),
            frame_rate_hz=float(d["frame_rate_hz"]),
            latency_s=float(d["latency_s"]),
            frame_drop_prob=float(d["frame_drop_prob"]),
            robots=robots,
            randomization=rnd,
        )


def _parse_robot(d: Mapping[str, Any], where: str) -> RobotSpec:
    speed = _edges(d["speed_bins_mps"], f"{where}.speed_bins_mps")
    n = speed.size
    pos = d["position"]
    head = d["heading"]
    flip_model = _enum(FlipModel, head["flip_model"], f"{where}.heading.flip_model")
    flip_rate = None
    flip_runs = None
    if flip_model is FlipModel.INDEPENDENT:
        flip_rate = _array(head["flip_rate"], f"{where}.heading.flip_rate", n)
        if np.any(flip_rate < 0.0) or np.any(flip_rate > 1.0):
            raise ValueError(f"{where}.heading.flip_rate: probabilities must lie in [0, 1]")
    else:
        runs = head["flip_runs"]
        flip_runs = (
            DurationTable.parse(runs["quantiles"], runs["normal_s"], 1, f"{where}.flip_runs"),
            DurationTable.parse(runs["quantiles"], runs["flipped_s"], 1, f"{where}.flip_runs"),
        )
    drop = d["dropout"]
    robot_bins = _edges(drop["robot_distance_bins_m"], f"{where}.dropout.robot_distance_bins_m")
    wall_bins = _edges(drop["wall_distance_bins_m"], f"{where}.dropout.wall_distance_bins_m")
    cells = robot_bins.size * wall_bins.size
    dropout = DropoutSpec(
        robot_distance_bins_m=robot_bins,
        wall_distance_bins_m=wall_bins,
        measured=DurationTable.parse(
            drop["quantiles"], drop["measured_s"], cells, f"{where}.dropout.measured_s"
        ),
        dropped=DurationTable.parse(
            drop["quantiles"], drop["dropped_s"], cells, f"{where}.dropout.dropped_s"
        ),
        borrowed=tuple(drop.get("borrowed", [])),
        enabled=_bool(drop.get("enabled", True), f"{where}.dropout.enabled"),
    )
    ekf = None
    if "ekf_error" in d:
        e = d["ekf_error"]
        sigma = np.stack(
            [_array(e["sigma"][c], f"{where}.ekf_error.sigma.{c}", n) for c in EKF_COMPONENTS]
        )
        tau = np.array([float(e["tau_s"][c]) for c in EKF_COMPONENTS])
        if np.any(tau <= 0.0) or np.any(sigma < 0.0):
            raise ValueError(f"{where}.ekf_error: tau must be positive and sigma non-negative")
        ekf = EkfErrorSpec(sigma=sigma, tau_s=tau)
    sigmas = [
        _array(pos[k], f"{where}.position.{k}", n)
        for k in (
            "sigma_along_m",
            "sigma_cross_m",
            "bias_along_m",
            "bias_along_reverse_m",
            "bias_cross_m",
        )
    ]
    heading_sigma = _array(head["sigma_rad"], f"{where}.heading.sigma_rad", n)
    if np.any(sigmas[0] < 0.0) or np.any(sigmas[1] < 0.0) or np.any(heading_sigma < 0.0):
        raise ValueError(f"{where}: sigmas must be non-negative")
    return RobotSpec(
        speed_bins_mps=speed,
        sigma_along_m=sigmas[0],
        sigma_cross_m=sigmas[1],
        bias_along_m=sigmas[2],
        bias_along_reverse_m=sigmas[3],
        bias_cross_m=sigmas[4],
        heading_sigma_rad=heading_sigma,
        flip_model=flip_model,
        flip_rate=flip_rate,
        flip_runs=flip_runs,
        flip_scale_by_label={
            str(k): float(v) for k, v in head.get("flip_scale_by_label", {}).items()
        },
        dropout=dropout,
        ekf_error=ekf,
    )


# ---------------------------------------------------------------------------
# Inputs and outputs
# ---------------------------------------------------------------------------


@dataclass
class TruePoses:
    """True state of every robot slot, field frame. Shapes [W, R, ...]."""

    xy: torch.Tensor  # [W, R, 2]
    yaw: torch.Tensor  # [W, R]
    velocity: torch.Tensor  # [W, R, 2]
    yaw_rate: torch.Tensor  # [W, R]


@dataclass
class Context:
    """What the dropout process conditions on. Shapes [W, R]."""

    robot_distance_m: torch.Tensor
    wall_distance_m: torch.Tensor

    @staticmethod
    def from_rect_arena(
        xy: torch.Tensor, radius_m: torch.Tensor | float, half_extent: torch.Tensor
    ) -> Context:
        """Edge-to-edge distance to the nearest other robot and centre distance to the nearest wall
        of an axis-aligned arena centred on the origin. `half_extent` is [W, 2] or [2]."""
        w, r, _ = xy.shape
        radius = torch.as_tensor(radius_m, dtype=xy.dtype, device=xy.device).expand(w, r)
        if r > 1:
            centre = torch.cdist(xy, xy)
            edge = (centre - radius[:, :, None] - radius[:, None, :]).clamp(min=0.0)
            eye = torch.eye(r, dtype=torch.bool, device=xy.device)
            robot = edge.masked_fill(eye, math.inf).min(dim=2).values
        else:
            robot = torch.full((w, r), math.inf, dtype=xy.dtype, device=xy.device)
        half = half_extent.to(dtype=xy.dtype, device=xy.device).expand(w, 2)[:, None, :]
        wall = (half - xy.abs()).min(dim=2).values.clamp(min=0.0)
        return Context(robot_distance_m=robot, wall_distance_m=wall)


@dataclass
class Deliveries:
    """At most one frame per world per step. Rows with `delivered` false carry stale values."""

    delivered: torch.Tensor  # [W] bool
    stamp: torch.Tensor  # [W] capture time of the delivered frame
    measured: torch.Tensor  # [W, R] bool, the robot was detected in that frame
    xy: torch.Tensor  # [W, R, 2]
    yaw: torch.Tensor  # [W, R]


@dataclass
class SensingParams:
    """Per-world draws for the current episode. The RL critic reads these; the actor never does."""

    frame_period_s: torch.Tensor  # [W]
    latency_s: torch.Tensor  # [W]
    frame_drop_prob: torch.Tensor  # [W]
    error_scale: torch.Tensor  # [W]
    offset_m: torch.Tensor  # [W, 2]
    flip_scale: torch.Tensor  # [W]
    dropout_stretch: torch.Tensor  # [W]


@dataclass
class RenderedOurRobot:
    """Stand-in for the our-robot EKF output. Shapes [W]."""

    xy: torch.Tensor  # [W, 2]
    yaw: torch.Tensor
    forward_speed: torch.Tensor
    yaw_rate: torch.Tensor
    stale: torch.Tensor  # bool


def keypoint_position_variance(
    xy: torch.Tensor,
    sigma_m: float,
    sigma_per_meter: float = 0.0,
    camera_xyz: Sequence[float] | None = None,
    z_m: float = 0.0,
) -> torch.Tensor:
    """`KalmanMotionEstimator::measurement_noise_for` for keypoint poses: the base sigma plus the
    per-metre range term, squared. `xy` is [..., 2]; returns [...]."""
    sigma = torch.full(xy.shape[:-1], sigma_m, dtype=xy.dtype, device=xy.device)
    if sigma_per_meter > 0.0:
        if camera_xyz is None:
            raise ValueError("position_sigma_per_meter needs the camera position")
        cam = torch.as_tensor(camera_xyz, dtype=xy.dtype, device=xy.device)
        dz = torch.full_like(sigma, z_m - float(cam[2]))
        sigma = sigma + sigma_per_meter * torch.sqrt(((xy - cam[:2]) ** 2).sum(-1) + dz**2)
    return sigma * sigma


# ---------------------------------------------------------------------------
# Samplers
# ---------------------------------------------------------------------------


def _wrap(a: torch.Tensor) -> torch.Tensor:
    return torch.atan2(torch.sin(a), torch.cos(a))


def _bin(x: torch.Tensor, edges: torch.Tensor) -> torch.Tensor:
    """Index of the left-closed bin `[edges[i], edges[i + 1])` holding x; the last is open."""
    return torch.bucketize(x.contiguous(), edges[1:], right=True)


class _Durations:
    """A `DurationTable` on the device, sampled by inverse transform in log duration."""

    def __init__(self, table: DurationTable, device: torch.device, dtype: torch.dtype) -> None:
        self.q = torch.as_tensor(table.quantiles, device=device, dtype=dtype)
        self.log_v = torch.as_tensor(np.log(table.values_s), device=device, dtype=dtype)

    def sample(self, cell: torch.Tensor, u: torch.Tensor) -> torch.Tensor:
        i = torch.searchsorted(self.q, u.contiguous(), right=True).clamp(1, self.q.numel() - 1)
        q0, q1 = self.q[i - 1], self.q[i]
        lo, hi = self.log_v[cell, i - 1], self.log_v[cell, i]
        return torch.exp(lo + (u - q0) / (q1 - q0) * (hi - lo))


class _RobotSampler:
    """One robot slot's tables on the device."""

    def __init__(self, spec: RobotSpec, device: torch.device, dtype: torch.dtype) -> None:
        def t(a: np.ndarray) -> torch.Tensor:
            return torch.as_tensor(a, device=device, dtype=dtype)

        self.spec = spec
        self.speed_edges = t(spec.speed_bins_mps)
        self.sigma_along = t(spec.sigma_along_m)
        self.sigma_cross = t(spec.sigma_cross_m)
        self.bias_along = t(spec.bias_along_m)
        self.bias_along_reverse = t(spec.bias_along_reverse_m)
        self.bias_cross = t(spec.bias_cross_m)
        self.heading_sigma = t(spec.heading_sigma_rad)
        self.flip_rate = t(spec.flip_rate) if spec.flip_rate is not None else None
        self.flip_runs = (
            (
                _Durations(spec.flip_runs[0], device, dtype),
                _Durations(spec.flip_runs[1], device, dtype),
            )
            if spec.flip_runs is not None
            else None
        )
        self.robot_edges = t(spec.dropout.robot_distance_bins_m)
        self.wall_edges = t(spec.dropout.wall_distance_bins_m)
        self.wall_bins = spec.dropout.wall_distance_bins_m.size
        self.measured = _Durations(spec.dropout.measured, device, dtype)
        self.dropped = _Durations(spec.dropout.dropped, device, dtype)
        self.dropout_enabled = spec.dropout.enabled


class MeasurementModel:
    """Batched measurement stream for `worlds` worlds of `len(roles)` robots each.

    Call `reset` for new episodes, then `step` once per simulation tick with the current time.
    A frame is captured on the first step at or after each frame time, stamped with that step's
    time (truth is known only at step times), and delivered on the first step at least `latency`
    later. Step intervals must not exceed the shortest frame period, so at most one frame is
    captured and one delivered per step.

    `frame_rate_hz` fixes the frame rate, overriding the file's nominal value and its
    randomization range. The lockstep sim passes its tick rate, since it captures one frame per
    tick.
    """

    def __init__(
        self,
        spec: SensingModelFile,
        roles: Sequence[str],
        worlds: int,
        device: torch.device | str = "cpu",
        dtype: torch.dtype = torch.float64,
        seed: int = 0,
        randomize: bool = True,
        enabled: bool = True,
        frame_rate_hz: float | None = None,
    ) -> None:
        missing = [r for r in roles if r not in spec.robots]
        if missing:
            raise ValueError(f"roles {missing} not in the sensing model ({sorted(spec.robots)})")
        self.spec = spec
        self.roles = tuple(roles)
        self.worlds = worlds
        self.robots = len(roles)
        self.device = torch.device(device)
        self.dtype = dtype
        self.randomize = randomize
        self.enabled = enabled
        self.fixed_frame_rate_hz = frame_rate_hz
        self.generator = torch.Generator(device=self.device)
        self.generator.manual_seed(seed)
        self._samplers = [_RobotSampler(spec.robots[r], self.device, dtype) for r in roles]

        rnd = spec.randomization
        max_latency = rnd.latency_s[1] if randomize else spec.latency_s
        max_rate = rnd.frame_rate_hz[1] if randomize else spec.frame_rate_hz
        if frame_rate_hz is not None:
            max_rate = frame_rate_hz
        self.capacity = math.ceil(max_latency * max_rate) + 2

        w, r, q = worlds, self.robots, self.capacity
        z = self._zeros
        self.params = SensingParams(
            frame_period_s=z(w) + 1.0 / (frame_rate_hz or spec.frame_rate_hz),
            latency_s=z(w) + spec.latency_s,
            frame_drop_prob=z(w) + spec.frame_drop_prob,
            error_scale=z(w) + 1.0,
            offset_m=z(w, 2),
            flip_scale=z(w) + 1.0,
            dropout_stretch=z(w) + 1.0,
        )
        self.label_flip_scale = z(w, r) + 1.0
        self.next_capture = z(w)
        # Dropout and flip state per robot. Expiry toggles the state, so reset() parks each one
        # in the opposite of its starting state with an expiry of now.
        self.is_measured = torch.zeros(w, r, dtype=torch.bool, device=self.device)
        self.measured_until = z(w, r)
        self.is_flipped = torch.ones(w, r, dtype=torch.bool, device=self.device)
        self.flipped_until = z(w, r)
        self.last_measured_stamp = z(w, r) - math.inf
        self._q_stamp = z(w, q)
        self._q_filled = torch.zeros(w, q, dtype=torch.bool, device=self.device)
        self._q_measured = torch.zeros(w, q, r, dtype=torch.bool, device=self.device)
        self._q_xy = z(w, q, r, 2)
        self._q_yaw = z(w, q, r)
        self._write = torch.zeros(w, dtype=torch.long, device=self.device)
        self._read = torch.zeros(w, dtype=torch.long, device=self.device)
        self._rows = torch.arange(w, device=self.device)

    def _zeros(self, *shape: int) -> torch.Tensor:
        return torch.zeros(*shape, device=self.device, dtype=self.dtype)

    def _rand(self, *shape: int) -> torch.Tensor:
        return torch.rand(*shape, generator=self.generator, device=self.device, dtype=self.dtype)

    def _randn(self, *shape: int) -> torch.Tensor:
        return torch.randn(*shape, generator=self.generator, device=self.device, dtype=self.dtype)

    def _uniform(self, lo_hi: tuple[float, float], n: int) -> torch.Tensor:
        lo, hi = lo_hi
        return lo + (hi - lo) * self._rand(n)

    def _times(self, value: float | torch.Tensor) -> torch.Tensor:
        return torch.as_tensor(value, device=self.device, dtype=self.dtype).expand(self.worlds)

    def reset(
        self,
        mask: torch.Tensor,
        now: float | torch.Tensor,
        labels: Sequence[Sequence[str]] | None = None,
    ) -> None:
        """Start new episodes in the `mask` worlds at time `now`: draw their parameters, clear
        their delivery queues and restart the dropout and flip processes. `labels[r]` names each
        world's label in robot slot r, for the per-label flip scale; None keeps a scale of 1."""
        mask = mask.to(self.device)
        t = self._times(now)
        w = self.worlds
        p = self.params
        spec = self.spec
        rnd = spec.randomization
        if self.randomize:
            draws = {
                "period": 1.0 / self._uniform(rnd.frame_rate_hz, w),
                "latency": self._uniform(rnd.latency_s, w),
                "drop": spec.frame_drop_prob * self._uniform(rnd.frame_drop_scale, w),
                "error": self._uniform(rnd.error_scale, w),
                "offset": rnd.offset_sigma_m * self._randn(w, 2),
                "flip": self._uniform(rnd.flip_scale, w),
                "stretch": self._uniform(rnd.dropout_stretch, w),
            }
            if self.fixed_frame_rate_hz is None:
                p.frame_period_s = torch.where(mask, draws["period"], p.frame_period_s)
            p.latency_s = torch.where(mask, draws["latency"], p.latency_s)
            p.frame_drop_prob = torch.where(mask, draws["drop"].clamp(0.0, 1.0), p.frame_drop_prob)
            p.error_scale = torch.where(mask, draws["error"], p.error_scale)
            p.offset_m = torch.where(mask[:, None], draws["offset"], p.offset_m)
            p.flip_scale = torch.where(mask, draws["flip"], p.flip_scale)
            p.dropout_stretch = torch.where(mask, draws["stretch"], p.dropout_stretch)
        if labels is not None:
            for r, sampler in enumerate(self._samplers):
                table = sampler.spec.flip_scale_by_label
                scale = torch.as_tensor(
                    [table.get(label, 1.0) for label in labels[r]],
                    device=self.device,
                    dtype=self.dtype,
                )
                self.label_flip_scale[:, r] = torch.where(mask, scale, self.label_flip_scale[:, r])
        self.next_capture = torch.where(mask, t, self.next_capture)
        m2 = mask[:, None]
        self.is_measured = torch.where(m2, False, self.is_measured)
        self.measured_until = torch.where(m2, t[:, None], self.measured_until)
        self.is_flipped = torch.where(m2, True, self.is_flipped)
        self.flipped_until = torch.where(m2, t[:, None], self.flipped_until)
        self.last_measured_stamp = torch.where(m2, -math.inf, self.last_measured_stamp)
        self._q_filled = torch.where(mask[:, None], False, self._q_filled)
        self._write = torch.where(mask, 0, self._write)
        self._read = torch.where(mask, 0, self._read)

    def step(self, now: float | torch.Tensor, truth: TruePoses, context: Context) -> Deliveries:
        t = self._times(now)
        if not self.enabled:
            delivered = torch.ones(self.worlds, dtype=torch.bool, device=self.device)
            measured = torch.ones(self.worlds, self.robots, dtype=torch.bool, device=self.device)
            self.last_measured_stamp = t[:, None].expand(-1, self.robots).clone()
            return Deliveries(delivered, t.clone(), measured, truth.xy.clone(), truth.yaw.clone())

        # The tolerance keeps a tick that lands on the frame time from missing it to rounding.
        capture = t >= self.next_capture - CLOCK_EPS_S
        period = self.params.frame_period_s
        upcoming = self.next_capture + period
        self.next_capture = torch.where(
            capture, torch.where(upcoming > t, upcoming, t + period), self.next_capture
        )
        kept = capture & (self._rand(self.worlds) >= self.params.frame_drop_prob)

        measured = torch.zeros(self.worlds, self.robots, dtype=torch.bool, device=self.device)
        xy = self._zeros(self.worlds, self.robots, 2)
        yaw = self._zeros(self.worlds, self.robots)
        for r, sampler in enumerate(self._samplers):
            measured[:, r] = capture & self._advance_dropout(r, sampler, capture, t, context)
            flipped = self._advance_flip(r, sampler, capture, t, truth)
            xy[:, r], yaw[:, r] = self._noisy_pose(sampler, truth, r, flipped)

        if bool(kept.any()):
            rows = self._rows[kept]
            slot = self._write[kept] % self.capacity
            self._q_stamp[rows, slot] = t[kept]
            self._q_filled[rows, slot] = True
            self._q_measured[rows, slot] = measured[kept]
            self._q_xy[rows, slot] = xy[kept]
            self._q_yaw[rows, slot] = yaw[kept]
            self._write = self._write + kept.long()

        head = self._read % self.capacity
        head_stamp = self._q_stamp[self._rows, head]
        ready = head_stamp + self.params.latency_s <= t + CLOCK_EPS_S
        due = self._q_filled[self._rows, head] & ready
        out = Deliveries(
            delivered=due,
            stamp=head_stamp,
            measured=self._q_measured[self._rows, head] & due[:, None],
            xy=self._q_xy[self._rows, head],
            yaw=self._q_yaw[self._rows, head],
        )
        if bool(due.any()):
            self._q_filled[self._rows[due], head[due]] = False
            self._read = self._read + due.long()
            self.last_measured_stamp = torch.where(
                out.measured, head_stamp[:, None], self.last_measured_stamp
            )
        return out

    def _advance_dropout(
        self,
        r: int,
        sampler: _RobotSampler,
        capture: torch.Tensor,
        t: torch.Tensor,
        context: Context,
    ) -> torch.Tensor:
        """Advance robot r's measured/dropped process to time t on capture rows."""
        if not sampler.dropout_enabled:
            self.is_measured[:, r] = True
            return self.is_measured[:, r]
        expired = capture & (t >= self.measured_until[:, r])
        state = torch.where(expired, ~self.is_measured[:, r], self.is_measured[:, r])
        robot_bin = _bin(context.robot_distance_m[:, r].to(self.dtype), sampler.robot_edges)
        wall_bin = _bin(context.wall_distance_m[:, r].to(self.dtype), sampler.wall_edges)
        cell = robot_bin * sampler.wall_bins + wall_bin
        u = self._rand(self.worlds)
        stretch = self.params.dropout_stretch
        duration = torch.where(
            state,
            sampler.measured.sample(cell, u) / stretch,
            sampler.dropped.sample(cell, u) * stretch,
        )
        self.measured_until[:, r] = torch.where(expired, t + duration, self.measured_until[:, r])
        self.is_measured[:, r] = state
        return state

    def _advance_flip(
        self,
        r: int,
        sampler: _RobotSampler,
        capture: torch.Tensor,
        t: torch.Tensor,
        truth: TruePoses,
    ) -> torch.Tensor:
        """Whether robot r's heading reads flipped in a frame captured now."""
        scale = self.params.flip_scale * self.label_flip_scale[:, r]
        if sampler.flip_rate is not None:
            speed = torch.linalg.vector_norm(truth.velocity[:, r], dim=-1).to(self.dtype)
            rate = sampler.flip_rate[_bin(speed, sampler.speed_edges)]
            return self._rand(self.worlds) < (rate * scale).clamp(0.0, 1.0)
        assert sampler.flip_runs is not None
        normal, flipped = sampler.flip_runs
        expired = capture & (t >= self.flipped_until[:, r])
        state = torch.where(expired, ~self.is_flipped[:, r], self.is_flipped[:, r])
        cell = torch.zeros(self.worlds, dtype=torch.long, device=self.device)
        u = self._rand(self.worlds)
        duration = torch.where(state, flipped.sample(cell, u), normal.sample(cell, u) / scale)
        self.flipped_until[:, r] = torch.where(expired, t + duration, self.flipped_until[:, r])
        self.is_flipped[:, r] = state
        return state

    def _noisy_pose(
        self, sampler: _RobotSampler, truth: TruePoses, r: int, flipped: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Body-frame position noise and bias, the field offset, and heading noise plus flip."""
        theta = truth.yaw[:, r].to(self.dtype)
        speed = torch.linalg.vector_norm(truth.velocity[:, r], dim=-1).to(self.dtype)
        b = _bin(speed, sampler.speed_edges)
        scale = self.params.error_scale
        n = self._randn(self.worlds, 3)
        c, s = torch.cos(theta), torch.sin(theta)
        # The along-track bias differs by direction of travel: on the 2026-10-15 sysid sessions
        # it is +4 mm forward and +13 mm in reverse above 1.5 m/s.
        velocity = truth.velocity[:, r].to(self.dtype)
        reversing = velocity[:, 0] * c + velocity[:, 1] * s < 0.0
        bias_along = torch.where(reversing, sampler.bias_along_reverse[b], sampler.bias_along[b])
        along = bias_along + scale * sampler.sigma_along[b] * n[:, 0]
        cross = sampler.bias_cross[b] + scale * sampler.sigma_cross[b] * n[:, 1]
        xy = truth.xy[:, r].to(self.dtype) + self.params.offset_m
        xy = xy + torch.stack([c * along - s * cross, s * along + c * cross], dim=-1)
        heading = theta + scale * sampler.heading_sigma[b] * n[:, 2] + math.pi * flipped
        return xy, _wrap(heading)


class OurEkfErrorModel:
    """Rendered our-robot EKF output: truth plus AR(1) error, coasting through measurement gaps.

    Each component of `EKF_COMPONENTS` follows
    `e_k = rho * e_{k-1} + sigma(v) * sqrt(1 - rho^2) * n`, `rho = exp(-dt / tau)`, so its
    long-run std is `sigma(v)` whatever `tau` is. Along and cross errors rotate with the true
    heading. Once the last delivered measurement is `GAP_MIN_FRAMES` frame periods old, the
    rendered pose coasts at its last forward speed and yaw rate until it is `max_coast_s` old,
    then holds and reads stale, as `KalmanMotionEstimator` does.
    """

    def __init__(
        self,
        spec: SensingModelFile,
        worlds: int,
        role: str = "ours",
        max_coast_s: float = 0.2,
        device: torch.device | str = "cpu",
        dtype: torch.dtype = torch.float64,
        seed: int = 0,
        randomize: bool = True,
    ) -> None:
        ekf = spec.robots[role].ekf_error
        if ekf is None:
            raise ValueError(f"sensing model has no [robots.{role}.ekf_error] table")
        self.spec = spec
        self.worlds = worlds
        self.max_coast_s = max_coast_s
        self.device = torch.device(device)
        self.dtype = dtype
        self.randomize = randomize
        self.generator = torch.Generator(device=self.device)
        self.generator.manual_seed(seed)
        self.speed_edges = torch.as_tensor(
            spec.robots[role].speed_bins_mps, device=self.device, dtype=dtype
        )
        self.sigma = torch.as_tensor(ekf.sigma, device=self.device, dtype=dtype)
        self.tau = torch.as_tensor(ekf.tau_s, device=self.device, dtype=dtype)
        k = len(EKF_COMPONENTS)
        self.sigma_scale = torch.ones(worlds, device=self.device, dtype=dtype)
        self.tau_scale = torch.ones(worlds, device=self.device, dtype=dtype)
        self.error = torch.zeros(worlds, k, device=self.device, dtype=dtype)
        self.last_now = torch.zeros(worlds, device=self.device, dtype=dtype)
        self.coasting = torch.zeros(worlds, dtype=torch.bool, device=self.device)
        self._origin_xy = torch.zeros(worlds, 2, device=self.device, dtype=dtype)
        self._origin_yaw = torch.zeros(worlds, device=self.device, dtype=dtype)
        self._origin_v = torch.zeros(worlds, device=self.device, dtype=dtype)
        self._origin_w = torch.zeros(worlds, device=self.device, dtype=dtype)
        self._origin_t = torch.zeros(worlds, device=self.device, dtype=dtype)

    def _rand(self, *shape: int) -> torch.Tensor:
        return torch.rand(*shape, generator=self.generator, device=self.device, dtype=self.dtype)

    def _sigma(self, speed: torch.Tensor) -> torch.Tensor:
        """[W, components] std at each world's speed bin."""
        return self.sigma[:, _bin(speed, self.speed_edges)].T * self.sigma_scale[:, None]

    def reset(self, mask: torch.Tensor, now: float | torch.Tensor, speed: torch.Tensor) -> None:
        """Start the `mask` worlds at `now`, with the error drawn from its stationary law."""
        mask = mask.to(self.device)
        t = torch.as_tensor(now, device=self.device, dtype=self.dtype).expand(self.worlds)
        if self.randomize:
            rnd = self.spec.randomization
            lo, hi = rnd.ekf_sigma_scale
            self.sigma_scale = torch.where(
                mask, lo + (hi - lo) * self._rand(self.worlds), self.sigma_scale
            )
            lo, hi = rnd.ekf_tau_scale
            self.tau_scale = torch.where(
                mask, lo + (hi - lo) * self._rand(self.worlds), self.tau_scale
            )
        start = self._sigma(speed.to(self.dtype)) * torch.randn(
            self.worlds,
            len(EKF_COMPONENTS),
            generator=self.generator,
            device=self.device,
            dtype=self.dtype,
        )
        self.error = torch.where(mask[:, None], start, self.error)
        self.last_now = torch.where(mask, t, self.last_now)
        self.coasting = torch.where(mask, False, self.coasting)

    def step(
        self,
        now: float | torch.Tensor,
        xy: torch.Tensor,
        yaw: torch.Tensor,
        velocity: torch.Tensor,
        yaw_rate: torch.Tensor,
        last_measured_stamp: torch.Tensor,
        frame_period_s: torch.Tensor,
    ) -> RenderedOurRobot:
        """Our robot's true state ([W, 2] or [W]) and its last delivered measurement stamp in,
        the rendered EKF output out."""
        t = torch.as_tensor(now, device=self.device, dtype=self.dtype).expand(self.worlds)
        xy, yaw = xy.to(self.dtype), yaw.to(self.dtype)
        velocity, yaw_rate = velocity.to(self.dtype), yaw_rate.to(self.dtype)
        dt = (t - self.last_now).clamp(min=0.0)
        self.last_now = t.clone()
        rho = torch.exp(-dt[:, None] / (self.tau[None, :] * self.tau_scale[:, None]))
        speed = torch.linalg.vector_norm(velocity, dim=-1)
        noise = torch.randn(
            self.worlds,
            len(EKF_COMPONENTS),
            generator=self.generator,
            device=self.device,
            dtype=self.dtype,
        )
        self.error = rho * self.error + self._sigma(speed) * torch.sqrt(1.0 - rho * rho) * noise

        c, s = torch.cos(yaw), torch.sin(yaw)
        e = self.error
        fresh_xy = xy + torch.stack([c * e[:, 0] - s * e[:, 1], s * e[:, 0] + c * e[:, 1]], -1)
        fresh_yaw = _wrap(yaw + e[:, 2])
        fresh_v = velocity[:, 0] * c + velocity[:, 1] * s + e[:, 3]
        fresh_w = yaw_rate + e[:, 4]

        age = t - last_measured_stamp.to(self.dtype)
        in_gap = age > GAP_MIN_FRAMES * frame_period_s.to(self.dtype)
        starting = in_gap & ~self.coasting
        self._origin_xy = torch.where(starting[:, None], fresh_xy, self._origin_xy)
        self._origin_yaw = torch.where(starting, fresh_yaw, self._origin_yaw)
        self._origin_v = torch.where(starting, fresh_v, self._origin_v)
        self._origin_w = torch.where(starting, fresh_w, self._origin_w)
        self._origin_t = torch.where(starting, t, self._origin_t)
        self.coasting = in_gap

        # Coast from the origin until the measurement is max_coast_s old, then hold.
        origin_age = self._origin_t - last_measured_stamp.to(self.dtype)
        span = torch.minimum(t - self._origin_t, (self.max_coast_s - origin_age).clamp(min=0.0))
        coast_xy, coast_yaw = _unicycle(
            self._origin_xy, self._origin_yaw, self._origin_v, self._origin_w, span
        )
        return RenderedOurRobot(
            xy=torch.where(in_gap[:, None], coast_xy, fresh_xy),
            yaw=torch.where(in_gap, coast_yaw, fresh_yaw),
            forward_speed=torch.where(in_gap, self._origin_v, fresh_v),
            yaw_rate=torch.where(in_gap, self._origin_w, fresh_w),
            stale=age > self.max_coast_s,
        )


def _unicycle(
    xy: torch.Tensor, yaw: torch.Tensor, v: torch.Tensor, w: torch.Tensor, dt: torch.Tensor
) -> tuple[torch.Tensor, torch.Tensor]:
    """Constant forward speed and yaw rate for dt, exact arc."""
    yaw1 = yaw + w * dt
    turning = w.abs() > 1e-9
    safe_w = torch.where(turning, w, torch.ones_like(w))
    arc_x = v / safe_w * (torch.sin(yaw1) - torch.sin(yaw))
    arc_y = v / safe_w * (torch.cos(yaw) - torch.cos(yaw1))
    line_x = v * dt * torch.cos(yaw)
    line_y = v * dt * torch.sin(yaw)
    dx = torch.where(turning, arc_x, line_x)
    dy = torch.where(turning, arc_y, line_y)
    return xy + torch.stack([dx, dy], -1), _wrap(yaw1)
