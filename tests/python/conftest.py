"""Skip test modules whose x86-only dependencies are absent (the Jetson has no MuJoCo)."""

import importlib.util

collect_ignore = []
if importlib.util.find_spec("mujoco") is None:
    collect_ignore.append("test_mujoco_sim.py")
