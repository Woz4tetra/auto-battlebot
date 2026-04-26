from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np
import numpy.typing as npt

if TYPE_CHECKING:
    import mujoco

from sim_types.wheel_drive_info import WheelDriveInfo


@dataclass
class RobotHandle:
    """State handle for a physics-simulated wheeled robot."""

    body_id: int
    freejoint_qposadr: int  # start index in data.qpos (7 values: xyz + quat wxyz)
    freejoint_qveladr: int  # start index in data.qvel (6 values)
    start_pos: tuple[float, float, float]
    start_quat_wxyz: tuple[float, float, float, float]
    wheel_drive: WheelDriveInfo


@dataclass
class MocapHandle:
    """State handle for a kinematically-controlled (mocap) body."""

    mocap_id: int
    body_id: int
    start_pos: tuple[float, float, float]
    start_quat_wxyz: tuple[float, float, float, float]


@dataclass
class SceneHandles:
    """All MuJoCo objects needed after scene construction."""

    model: mujoco.MjModel
    data: mujoco.MjData
    renderer: mujoco.Renderer
    our_robot: RobotHandle
    opponents: list[RobotHandle | MocapHandle]
    panorama_bg: npt.NDArray[np.uint8] | None
