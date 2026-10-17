"""Name-keyed registries for the RL seams, the Python counterpart of `REGISTER_CONFIG`."""

from __future__ import annotations

from collections.abc import Callable
from typing import Generic, TypeVar

T = TypeVar("T")


class Registry(Generic[T]):
    """Maps a config `type` string to a factory. An unknown name fails listing the valid ones."""

    def __init__(self, seam: str) -> None:
        self.seam = seam
        self._entries: dict[str, T] = {}

    def register(self, name: str) -> Callable[[T], T]:
        def add(entry: T) -> T:
            if name in self._entries:
                raise ValueError(f"{self.seam} {name!r} is registered twice")
            self._entries[name] = entry
            return entry

        return add

    def __getitem__(self, name: str) -> T:
        try:
            return self._entries[name]
        except KeyError:
            valid = ", ".join(sorted(self._entries))
            raise KeyError(f"unknown {self.seam} {name!r}; valid: {valid}") from None

    def names(self) -> list[str]:
        return sorted(self._entries)
