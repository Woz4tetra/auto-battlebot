"""Policies behind the `POLICIES` registry. Importing this package registers them."""

from auto_battlebot.rl.policies import mlp, scripted  # noqa: F401
from auto_battlebot.rl.policies.base import POLICIES, Policy

__all__ = ["POLICIES", "Policy"]
