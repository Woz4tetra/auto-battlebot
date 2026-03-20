from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
import numpy.typing as npt

if TYPE_CHECKING:
    import genesis as gs
    from genesis.engine.entities.rigid_entity import RigidEntity
    from genesis.vis.camera import Camera


@dataclass
class SceneHandles:
    """All Genesis objects and derived config needed after scene construction."""

    scene: gs.Scene
    camera: Camera
    our_robot: RigidEntity
    opponents: list[RigidEntity]
    panorama_bg: npt.NDArray[np.uint8] | None
