"""Import a repo script that lives outside a package (scripts/, training/, tests/) by its path."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

from auto_battlebot.paths import REPO_ROOT

__all__ = ["REPO_ROOT", "load_script", "module_name"]


def module_name(relative: str) -> str:
    """The sys.modules key for a repo-relative script path."""
    return "repo_script." + Path(relative).with_suffix("").as_posix().replace("/", ".")


def load_script(relative: str) -> ModuleType:
    """Load a repo-relative .py file once and return it as a module."""
    name = module_name(relative)
    if name in sys.modules:
        return sys.modules[name]
    path = REPO_ROOT / relative
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
    except BaseException:
        del sys.modules[name]
        raise
    return module
