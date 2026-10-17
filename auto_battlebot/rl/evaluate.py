"""Batched evaluation of any `Policy` on a fixed, seeded scenario set.

Each world plays exactly one episode: the environment is reset with the scenario seed, the policy
acts deterministically, and each world's totals are taken the step it first finishes. A world that
finishes keeps stepping (the environment auto-resets it) but its later episodes are ignored, so
every policy is scored on the same starting states.
"""

from __future__ import annotations

import torch

from auto_battlebot.rl.env import VecEnv
from auto_battlebot.rl.metrics import EpisodeMetrics, summarize
from auto_battlebot.rl.policies.base import Policy


@torch.no_grad()
def evaluate(policy: Policy, env: VecEnv, seed: int, deterministic: bool = True) -> EpisodeMetrics:
    obs, _ = env.reset(seed)
    w = env.num_worlds
    finished = torch.zeros(w, dtype=torch.bool, device=env.device)
    totals: dict[str, torch.Tensor] = {}
    state = None
    for _ in range(env.spec.max_episode_steps + 1):
        action, state = policy.act(obs, state, deterministic)
        step = env.step(action)
        first = step.done & ~finished
        if bool(first.any()):
            for key, value in step.episode.items():
                if key not in totals:
                    totals[key] = torch.zeros_like(value)
                totals[key] = torch.where(first, value, totals[key])
        finished |= step.done
        state = policy.reset_state(state, step.done)
        obs = step.obs
        if bool(finished.all()):
            break
    episode_s = env.spec.max_episode_steps * env.spec.policy_dt_s
    return summarize(totals, episode_s)
