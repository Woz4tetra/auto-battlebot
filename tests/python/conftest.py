"""Skip test modules whose x86-only dependencies are absent (the Jetson has no MuJoCo)."""

import importlib.util
import pathlib
import sys

import pytest

collect_ignore = []
if importlib.util.find_spec("mujoco") is None:
    collect_ignore.append("test_mujoco_sim.py")


@pytest.fixture(scope="session")
def load_training():
    """Load a `training/` script by repo-relative path (`training/` is not a package)."""
    repo_root = pathlib.Path(__file__).resolve().parents[2]

    def load(relative: str):
        path = repo_root / relative
        spec = importlib.util.spec_from_file_location(f"training_{path.stem}", path)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        return module

    return load
