"""Learners behind the `LEARNERS` registry. Importing this package registers them."""

from auto_battlebot.rl.learners import fasttd3, ppo  # noqa: F401
from auto_battlebot.rl.learners.base import LEARNERS

__all__ = ["LEARNERS"]
