"""MuJoCo's own 3D viewer on the MuJoCo plant, next to the OpenCV top-down window.

Shows the contact geometry the plant actually steps: the hull meshes, wheels, skid, arena walls,
wall blocks and the opponents (a box or a cylinder each, per [[opponents]] collision_box). The
OpenCV window draws sprites at the same poses, so the two side by side show where the sprites and
the collision shapes disagree.

Passive mode: the sim server keeps stepping, and sync() copies the state to the render thread.
"""

from __future__ import annotations

import atexit
import threading
from typing import Any

import mujoco
import mujoco.viewer
from plants import PlantInterface
from plants.mujoco_plant import MujocoPlant

from auto_battlebot.mujoco_sim.closed_loop import BLOCK_PREFIX, MOVING_BLOCK_PREFIX

# The hull meshes are collision-only geoms in group 3 (auto_battlebot/mujoco_sim/mjcf.py), which
# the viewer hides by default.
HULL_GEOM_GROUP = 3
OPPONENT_RGBA = (1.0, 0.45, 0.1, 0.6)
WALL_BLOCK_RGBA = (0.6, 0.6, 0.6, 0.8)


class MujocoWindow:
    def __init__(self, arena_w: float, arena_h: float) -> None:
        self._arena = (arena_w, arena_h)
        self._handle: Any = None
        self._thread: threading.Thread | None = None
        self._model: mujoco.MjModel | None = None
        self._closed = False

    def _launch(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        before = set(threading.enumerate())
        self._handle = mujoco.viewer.launch_passive(
            model, data, show_left_ui=False, show_right_ui=False
        )
        # launch_passive does not return its render thread, and Handle.close() only asks it to
        # stop. Exiting while it still renders races the glfw.terminate atexit hook and segfaults.
        self._thread = next((t for t in threading.enumerate() if t not in before), None)
        # Registered after launch_passive registers glfw.terminate, so it runs first (LIFO).
        atexit.unregister(self.close)
        atexit.register(self.close)
        self._model = model
        with self._handle.lock():
            self._handle.opt.geomgroup[HULL_GEOM_GROUP] = 1
            # The blocks are floor grey by default and show only as shadows from above. Colour
            # is render-only, so this does not touch the contacts the plant steps.
            for i in range(model.ngeom):
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
                if name.startswith(MOVING_BLOCK_PREFIX):
                    model.geom_rgba[i] = OPPONENT_RGBA
                elif name.startswith(BLOCK_PREFIX):
                    model.geom_rgba[i] = WALL_BLOCK_RGBA
            # Top down with +x right and +y up, the OpenCV window's orientation.
            cam = self._handle.cam
            cam.lookat[:] = (0.0, 0.0, 0.0)
            cam.distance = 1.4 * max(self._arena)
            cam.elevation = -90.0
            cam.azimuth = 90.0

    def sync(self, plant: PlantInterface) -> None:
        """Call once per tick. A respawn builds a new plant, so a new model reopens the window."""
        assert isinstance(plant, MujocoPlant), "the server opens this only for plant = mujoco"
        model, data = plant.model, plant.data
        if self._closed:
            return
        if self._handle is not None and not self._handle.is_running():
            # Closed by hand: stay closed rather than reopening on the next respawn.
            self._closed = True
            return
        if model is not self._model:
            self.close()
            self._launch(model, data)
        self._handle.sync()

    def close(self) -> None:
        if self._handle is not None:
            self._handle.close()
            self._handle = None
            self._model = None
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
