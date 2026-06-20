"""Typed config for the fast headless kinematic sim (kinematic_sim_server.py).

Loaded and validated via config.loader.load_config (dacite, strict). Keep [arena] in sync with
config/headless_sim.toml on the C++ side.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class KinematicServerConfig:
    host: str = "127.0.0.1"
    port: int = 14882


@dataclass
class SimParamsConfig:
    dt: float = 1.0 / 30.0  # seconds of sim time per tick
    max_ticks: int = 5400  # server closes the connection after this many ticks
    seed: int = 0


@dataclass
class ArenaConfig:
    width: float = 2.4
    height: float = 2.4


@dataclass
class LatencyConfig:
    command_ms: float = 0.0  # actuation latency, converted to sim ticks
    observation_ms: float = 0.0  # perception latency, converted to sim ticks


@dataclass
class CameraConfig:
    # Resolution is tiny because no images are rendered (Noop perception ignores them); only the
    # world position is used, for the flat-plane projection bias.
    res_width: int = 16
    res_height: int = 16
    fov: float = 70.0
    pos: list[float] = field(default_factory=lambda: [0.0, -1.5, 0.6])
    lookat: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


@dataclass
class PlantConfig:
    start_pos: list[float] = field(default_factory=lambda: [-0.6, -0.6])
    start_yaw_deg: float = 45.0
    max_linear_speed: float = 0.434  # m/s at command magnitude 1.0
    max_angular_speed: float = 0.212  # rad/s at command magnitude 1.0
    tau_linear: float = 0.060  # s, first-order coast time constant (FIT TO RECORDINGS)
    tau_angular: float = 0.064  # s (FIT TO RECORDINGS)
    radius: float = 0.11  # m, half robot length for wall clamp

    # Physical-realism fields fit by playground/calibration/fit_plant_calib.py. Each defaults to 0 meaning
    # "unset, fall back to the symmetric values above", so existing configs behave exactly as before.
    max_linear_speed_fwd: float = 0.0  # forward gain; falls back to max_linear_speed
    max_linear_speed_rev: float = 0.0  # reverse gain (asymmetry); falls back to the forward gain
    tau_linear_accel: float = 0.0  # spin-up; falls back to tau_linear
    tau_linear_decel: float = 0.0  # coast/brake; falls back to tau_linear_accel
    tau_angular_accel: float = 0.0  # falls back to tau_angular
    tau_angular_decel: float = 0.0  # falls back to tau_angular_accel
    steer_brake_coeff: float = 0.0  # forward-speed loss per unit |angular cmd| (physical coupling)
    # Residual deadzone the plant still shows AFTER the transmitter's lifted_deadzone compensation. The
    # measured physical deadzone belongs in config/main.toml lifted_deadzone_percent, not here; set this
    # only if the real closed loop still has a dead low end. 0 = none.
    deadzone_linear: float = 0.0  # command fraction
    deadzone_angular: float = 0.0


@dataclass
class ProjectionBiasConfig:
    enabled: bool = False
    keypoint_height: float = 0.06  # m above the field plane


@dataclass
class PerceptionConfig:
    pos_noise_std: float = 0.0  # m
    yaw_noise_std: float = 0.0  # rad
    dropout_prob: float = 0.0  # per-robot per-tick probability of omission
    projection_bias: ProjectionBiasConfig = field(default_factory=ProjectionBiasConfig)


@dataclass
class OpponentConfig:
    behavior: str = "static"  # static | straight | circle | random_walk | replay
    start_pos: list[float] = field(default_factory=lambda: [0.5, 0.5])
    speed: float = 1.0
    heading_deg: float = 180.0
    radius: float = 0.5  # circle radius
    replay_csv: str = ""


@dataclass
class KinematicSimConfig:
    server: KinematicServerConfig = field(default_factory=KinematicServerConfig)
    sim: SimParamsConfig = field(default_factory=SimParamsConfig)
    arena: ArenaConfig = field(default_factory=ArenaConfig)
    latency: LatencyConfig = field(default_factory=LatencyConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    our_robot: PlantConfig = field(default_factory=PlantConfig)
    perception: PerceptionConfig = field(default_factory=PerceptionConfig)
    opponents: list[OpponentConfig] = field(default_factory=lambda: [OpponentConfig()])
