"""Give each new mesh object a unique segmentation pass index (requires Blender)."""

from __future__ import annotations

from collections.abc import Iterable
from typing import Any, TypeVar

import bpy

T = TypeVar("T")


def next_pass_index(taken: Iterable[int]) -> int:
    return max(taken, default=0) + 1


def assign_pass_index(obj: T) -> T:
    target: Any = getattr(obj, "blender_obj", obj)
    target.pass_index = next_pass_index(other.pass_index for other in bpy.data.objects)
    return obj
