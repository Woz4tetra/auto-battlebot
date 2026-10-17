"""Typed config for the fast headless kinematic sim (kinematic_sim_server.py).

Loaded and validated via config.loader.load_config (dacite, strict). Keep [arena] in sync with
config/simulation/kinematic_sim.toml on the C++ side. Obstacle geometry is not duplicated here:
`obstacles_file` names the shared TOML that the C++ field filter also reads.
"""

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
    dt: float = 1.0 / 30.0  # seconds of sim time per tick
    max_ticks: int = 0  # 0 = run until interrupted; > 0 closes the connection after N ticks
    seed: int = 0
    # What to do when an episode ends early -- today that means falling in a hole.
    #
    # False (interactive): report it, respawn our robot at its start pose, keep the connection.
    # Falling in latches the plant's velocity to zero, so without a respawn the robot would sit in
    # the hole for the rest of the session.
    #
    # True (batch): close the connection. sim_sweep waits for the binary to exit and reads the
    # EPISODE line, so an outcome has to end the run for a sweep to score it at all.
    #
    # max_ticks is separate and always ends the run: it is an explicit cap, not an outcome.
    stop_on_outcome: bool = False
    # Per-tick CSV of the applied command and true poses, rewritten on each client connect. Empty
    # means none. sim_stop_test.py scores stops from it.
    trace_csv: str = ""


@dataclass
class ArenaConfig:
    width: float = 2.4
    height: float = 2.4


@dataclass
class LatencyConfig:
    command_ms: float = 0.0  # actuation latency, converted to sim ticks


@dataclass
class CameraConfig:
    # Noop perception ignores the pixels, but the UI draws its overlay on this frame by
    # projecting field points through these intrinsics. At the old 16x16 placeholder the whole
    # overlay landed inside a handful of pixels and the pane read as black. 640x360 is small
    # enough to stay cheap over the socket and large enough to look like the real camera pane.
    res_width: int = 640
    res_height: int = 360
    fov: float = 52.0
    pos: list[float] = field(default_factory=lambda: [0.0, -0.5, 2.6])
    lookat: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


class PlantType(str, Enum):
    """Which model drives our robot. See simulation/plants/."""

    KINEMATIC = "kinematic"  # 2D first-order lag, the model the C++ filter and nav assume
    MUJOCO = "mujoco"  # Mr Stabs Mk2 rigid body with the firmware mixer; Mr Stabs Mk2 only

    @classmethod
    def _missing_(cls, value: object) -> PlantType:
        raise ValueError(f"unknown plant {value!r}; valid: {', '.join(m.value for m in cls)}")


@dataclass
class MujocoPlantConfig:
    # fit.json from playground/calibration/fit_mujoco_plant.py, repo-relative; the lowest-loss
    # run is used. Empty runs the unfit PlantParams defaults.
    fit_file: str = ""
    timestep: float = 1e-3  # s; the fit and its checks run at 1 ms
    pack_voltage: float = 15.2  # V, nominal 4S; the fit's logged vbat is not replayed here
    # The transmitter's flip switch. "down" (power-on, competition) runs the heading hold and
    # reverses the throttle while the IMU reads upside down, so a flipped robot keeps driving
    # wedge-first; "middle" turns both off; "up" always reverses. See FlipSwitch.
    flip_switch: FlipSwitch = FlipSwitch.DOWN


@dataclass
class PlantConfig:
    plant: PlantType = PlantType.KINEMATIC
    # Read only when plant = "mujoco". The kinematic fields below are read only when plant =
    # "kinematic"; start_pos, start_yaw_deg, sprite and radius apply to both.
    mujoco: MujocoPlantConfig = field(default_factory=MujocoPlantConfig)
    start_pos: list[float] = field(default_factory=lambda: [-0.6, -0.6])
    start_yaw_deg: float = 45.0
    # Which entry in the sprite manifest the viewer draws for this robot. The sprites are named
    # after the robot, not its role, because which one is "ours" is what this config decides.
    sprite: str = "mrs_buff_mk3"
    max_linear_speed: float = 0.434  # m/s at command magnitude 1.0
    max_angular_speed: float = 0.212  # rad/s at command magnitude 1.0
    tau_linear: float = 0.060  # s, first-order coast time constant (FIT TO RECORDINGS)
    tau_angular: float = 0.064  # s (FIT TO RECORDINGS)
    radius: float = 0.11  # m, half robot length for wall clamp

    # Physical-realism fields fit by playground/calibration/fit_plant_calib.py. Each defaults
    # to 0 meaning "unset, fall back to the symmetric values above", so existing configs behave
    # exactly as before.
    max_linear_speed_fwd: float = 0.0  # forward gain; falls back to max_linear_speed
    max_linear_speed_rev: float = 0.0  # reverse gain (asymmetry); falls back to the forward gain
    tau_linear_accel: float = 0.0  # spin-up; falls back to tau_linear
    tau_linear_decel: float = 0.0  # coast/brake; falls back to tau_linear_accel
    tau_angular_accel: float = 0.0  # falls back to tau_angular
    tau_angular_decel: float = 0.0  # falls back to tau_angular_accel
    steer_brake_coeff: float = 0.0  # forward-speed loss per unit |angular cmd| (physical coupling)
    angular_droop_coeff: float = 0.0  # yaw-rate loss per unit |linear cmd| (physical coupling)
    # Deadzone on the command the plant actually receives. SimTransmitter hands the navigation
    # command straight to the sim with no drive processor, so unlike the deployed path there is
    # no lifted_deadzone_percent upstream and these carry the raw physical deadzone. Per sign,
    # matching the jig fit: the _rev / _right value falls back to its counterpart when unset.
    deadzone_linear: float = 0.0  # command fraction, forward
    deadzone_linear_rev: float | None = None
    deadzone_angular: float = 0.0  # command fraction, left
    deadzone_angular_right: float | None = None


@dataclass
class ProjectionBiasConfig:
    enabled: bool = False
    keypoint_height: float = 0.06  # m above the field plane


@dataclass
class PerceptionConfig:
    # Sensing model TOML, repo-relative (docs/plans/sensing_model_plan.md): perception latency,
    # dropouts, position and heading noise, flips. Empty delivers the true poses every tick with
    # no latency.
    sensing_model: str = ""
    # Draw the model's per-episode randomization (latency, noise scale, flip rate, ...) from
    # [sim] seed. False runs its nominal values. The frame rate is always the tick rate.
    randomize: bool = False
    projection_bias: ProjectionBiasConfig = field(default_factory=ProjectionBiasConfig)


@dataclass
class OpponentConfig:
    behavior: str = "static"  # static | straight | circle | random_walk | replay | trained
    start_pos: list[float] = field(default_factory=lambda: [0.5, 0.5])
    speed: float = 1.0
    heading_deg: float = 180.0
    radius: float = 0.5  # circle radius
    replay_csv: str = ""
    sprite: str = "mrs_buff_mk2"  # manifest entry the viewer draws; "house_bot" for the house bot
    # Treat this opponent as a moving obstacle of this radius (m), the way the house bot behaves:
    # the plant is clamped out of it and clearance to it is scored. 0 = not an obstacle. On the
    # C++ side the matching track arrives as NEUTRAL (see GroundTruthRobotFilter.neutral_count)
    # and the hazard assembler turns it into a TRACKED keep-out disc.
    #
    # Unlike a static hazard there is no shared geometry file here: both sides derive the radius
    # from the robot itself, so this has to be the track's own half-diagonal. GroundTruthRobotFilter
    # reports every robot as 0.15 x 0.15 m, which is 0.106. Setting it larger models a house bot
    # the controller cannot see the true size of, and the sweep will report the scrapes.
    hazard_radius: float = 0.0
    # The MuJoCo plant's contact shape for this obstacle: a box of [length, width, height] in m,
    # length along the opponent's heading, turned with its yaw. Empty is a cylinder of
    # hazard_radius. Read only when hazard_radius > 0, and only by plant = "mujoco": the kinematic
    # plant and the clearance score always use the hazard_radius disc.
    collision_box: list[float] = field(default_factory=list)
    # behavior = "trained" only: the RL training opponents (rl.envs.trained_opponent).
    # trained_behaviour is one of auto_battlebot.rl.envs.opponents.BEHAVIOURS, or "any" to draw
    # one per reset as stage D does; parameters are drawn as in training. trained_holdout draws
    # the evaluation's held-out pursuit corner and replay tracks instead of the training ones.
    trained_behaviour: str = "any"
    trained_holdout: bool = False

    def __post_init__(self) -> None:
        if self.trained_behaviour not in TRAINED_BEHAVIOURS:
            raise ValueError(
                f"unknown trained_behaviour {self.trained_behaviour!r}; "
                f"valid: {', '.join(TRAINED_BEHAVIOURS)}"
            )


@dataclass
class ObstacleConfig:
    """One keep-out disc in the arena, in field-frame metres.

    ``hole`` ends the run when the robot's centre crosses into it; ``wall_block`` clamps the
    robot to its boundary and kills forward speed, exactly like an arena wall. ``radius`` is the
    raw geometry, before any inflation: the controller inflates by its own half-diagonal, the
    simulated floor does not.
    """

    kind: str = "hole"  # hole | wall_block
    center: list[float] = field(default_factory=lambda: [0.0, 0.0])
    radius: float = 0.25


@dataclass
class ViewerConfig:
    """In-loop OpenCV window. On by default: running the sim by hand means watching it.

    ``sim_sweep`` turns it off for every run it drives, so batch runs stay headless and fast.

    Rendering costs wall-clock time but cannot corrupt a run: the sim owns logical time and ships
    ``sim_time`` in the response header, which the C++ side adopts through ManualClock. A slow
    render makes the sim slower in real time and changes nothing the controller sees.
    """

    enable: bool = True
    window_px: int = 900
    render_every: int = 1  # tick decimation
    realtime: bool = True  # pace sim.dt to a human; False = free-run
    sprites_dir: str = "simulation/assets/sprites"
    background: str = "simulation/assets/cage/nhrl_cage_floor.png"
    # Also open MuJoCo's 3D viewer on the plant's contact geometry (simulation/mujoco_window.py).
    # Needs enable = true and [our_robot] plant = "mujoco"; ignored otherwise.
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
    # Shared arena geometry, as a path to a TOML file holding [[hazards]] entries. The C++ field
    # filter reads the same file through [field_filter].hazards_file, so the simulated floor and
    # the controller's keep-out discs cannot drift apart. Empty = no obstacles.
    obstacles_file: str = ""
