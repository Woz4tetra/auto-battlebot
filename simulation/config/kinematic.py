from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from auto_battlebot.mujoco_sim.firmware import FlipSwitch
from auto_battlebot.rl.envs.trained_opponent import TRAINED_BEHAVIOURS


@dataclass
class KinematicServerConfig:
    host: str = "127.0.0.1"
    port: int = 14882


@dataclass
class SimParamsConfig:
    dt: float = 1.0 / 30.0
    max_ticks: int = 0
    seed: int = 0
    stop_on_outcome: bool = False
    trace_csv: str = ""


@dataclass
class ArenaConfig:
    width: float = 2.4
    height: float = 2.4


@dataclass
class LatencyConfig:
    command_ms: float = 0.0


@dataclass
class CameraConfig:
    res_width: int = 640
    res_height: int = 360
    fov: float = 52.0
    pos: list[float] = field(default_factory=lambda: [0.0, -0.5, 2.6])
    lookat: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


class PlantType(str, Enum):
    """Which model drives our robot. See simulation/plants/."""

    KINEMATIC = "kinematic"
    MUJOCO = "mujoco"

    @classmethod
    def _missing_(cls, value: object) -> PlantType:
        raise ValueError(f"unknown plant {value!r}; valid: {', '.join(m.value for m in cls)}")


@dataclass
class MujocoPlantConfig:
    fit_file: str = ""
    timestep: float = 1e-3
    pack_voltage: float = 15.2
    flip_switch: FlipSwitch = FlipSwitch.DOWN


@dataclass
class PlantConfig:
    plant: PlantType = PlantType.KINEMATIC
    mujoco: MujocoPlantConfig = field(default_factory=MujocoPlantConfig)
    start_pos: list[float] = field(default_factory=lambda: [-0.6, -0.6])
    start_yaw_deg: float = 45.0
    sprite: str = "mrs_buff_mk3"
    max_linear_speed: float = 0.434
    max_angular_speed: float = 0.212
    tau_linear: float = 0.060
    tau_angular: float = 0.064
    radius: float = 0.11

    max_linear_speed_fwd: float = 0.0
    max_linear_speed_rev: float = 0.0
    tau_linear_accel: float = 0.0
    tau_linear_decel: float = 0.0
    tau_angular_accel: float = 0.0
    tau_angular_decel: float = 0.0
    steer_brake_coeff: float = 0.0
    angular_droop_coeff: float = 0.0
    deadzone_linear: float = 0.0
    deadzone_linear_rev: float | None = None
    deadzone_angular: float = 0.0
    deadzone_angular_right: float | None = None


@dataclass
class ProjectionBiasConfig:
    enabled: bool = False
    keypoint_height: float = 0.06


@dataclass
class PerceptionConfig:
    sensing_model: str = ""
    randomize: bool = False
    projection_bias: ProjectionBiasConfig = field(default_factory=ProjectionBiasConfig)


@dataclass
class OpponentConfig:
    behavior: str = "static"
    start_pos: list[float] = field(default_factory=lambda: [0.5, 0.5])
    speed: float = 1.0
    heading_deg: float = 180.0
    radius: float = 0.5
    replay_csv: str = ""
    sprite: str = "mrs_buff_mk2"
    hazard_radius: float = 0.0
    collision_box: list[float] = field(default_factory=list)
    trained_behaviour: str = "any"
    trained_holdout: bool = False
    policy_bundle: str = ""

    def __post_init__(self) -> None:
        if self.behavior == "policy" and not self.policy_bundle:
            raise ValueError('behavior = "policy" needs policy_bundle')
        if self.trained_behaviour not in TRAINED_BEHAVIOURS:
            raise ValueError(
                f"unknown trained_behaviour {self.trained_behaviour!r}; "
                f"valid: {', '.join(TRAINED_BEHAVIOURS)}"
            )


@dataclass
class ObstacleConfig:
    kind: str = "hole"
    center: list[float] = field(default_factory=lambda: [0.0, 0.0])
    radius: float = 0.25


@dataclass
class ViewerConfig:
    enable: bool = True
    window_px: int = 900
    render_every: int = 1
    realtime: bool = True
    sprites_dir: str = "simulation/assets/sprites"
    background: str = "simulation/assets/cage/nhrl_cage_floor.png"
    mujoco: bool = False


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
    viewer: ViewerConfig = field(default_factory=ViewerConfig)
    obstacles_file: str = ""
