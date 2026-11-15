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
    """Load a `training/` script by repo-relative path, preloading bare-name sibling imports."""
    repo_root = pathlib.Path(__file__).resolve().parents[2]

    def load_file(path: pathlib.Path, name: str):
        spec = importlib.util.spec_from_file_location(name, path)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        return module

    def load(relative: str, siblings: tuple[str, ...] = ()):
        path = repo_root / relative
        for sibling in siblings:
            if sibling not in sys.modules:
                load_file(path.parent / f"{sibling}.py", sibling)
        return load_file(path, f"training_{path.stem}")

    return load
