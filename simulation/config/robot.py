from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class RobotConfig:
    model_path: str = "robot.gltf"
    scale: float = 1.0
    start_pos: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    start_rotation: float = 0.0
    model_euler: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    max_linear_speed: float = 2.0
    max_angular_speed: float = 6.0
    behavior: str = "static"
    speed: float = 0.3

    # Wheel-based control (active when wheel_radius > 0)
    wheel_radius: float = 0.0
    track_width: float = 0.0
    left_wheel_joints: list[str] = field(default_factory=list)
    right_wheel_joints: list[str] = field(default_factory=list)
    wheel_kv: float = 50.0
    wheel_force_limit: float = 80.0
    # MuJoCo joint dynamics (replaces Genesis wheel_kp/wheel_friction)
    wheel_armature: float = 1.2e-4
    wheel_damping: float = 0.034
    wheel_frictionloss: float = 0.05
    # Contact friction [longitudinal, lateral, rolling]
    wheel_contact_friction: list[float] = field(default_factory=lambda: [1.6, 1.6, 0.001])

    @property
    def has_wheels(self) -> bool:
        return self.wheel_radius > 0.0
