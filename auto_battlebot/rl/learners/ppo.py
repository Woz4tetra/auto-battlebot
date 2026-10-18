"""`ppo`: proximal policy optimization for a tanh-squashed Gaussian actor (plan section 11).

Clipped surrogate, generalized advantage estimation (lambda 0.95, gamma 0.99), 24 steps per world
per update, a critic that also sees the environment's privileged inputs, and an adaptive learning
rate that holds the per-update KL near a target, as rsl_rl does. Truncated episodes bootstrap from
the critic's value of their final state. The ratio is taken on the pre-tanh sample, where the tanh
Jacobian cancels.

An optional anchor policy keeps behaviour already learned while fine-tuning on a narrower
problem: on the samples `anchor_mask` selects, the loss adds `anchor_coef` times the squared
distance from the anchor's mean action, in units of the student's own std. That is the KL from a
Gaussian at the anchor's mean with the student's std, so it scales with exploration the way the
surrogate does.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import torch
from torch import nn

from auto_battlebot.rl.env import VecEnv
from auto_battlebot.rl.learners.base import LEARNERS, EpisodeLog
from auto_battlebot.rl.policies.mlp import MlpGaussianPolicy, RunningNorm, mlp


@dataclass
class PpoConfig:
    steps_per_update: int = 24
    epochs: int = 5
    minibatches: int = 4
    gamma: float = 0.99
    lam: float = 0.95
    clip: float = 0.2
    value_coef: float = 1.0
    entropy_coef: float = 0.003
    lr: float = 3e-4
    min_lr: float = 1e-5
    max_lr: float = 1e-2
    desired_kl: float = 0.01
    max_grad_norm: float = 1.0
    critic_hidden: tuple[int, ...] = (256, 256, 128)
    anchor_coef: float = 0.0


def _gaussian_logp(u: torch.Tensor, mean: torch.Tensor, std: torch.Tensor) -> torch.Tensor:
    z = (u - mean) / std
    return (-0.5 * z * z - std.log() - 0.5 * math.log(2 * math.pi)).sum(-1)


@LEARNERS.register("ppo")
class Ppo:
    def __init__(self, policy: MlpGaussianPolicy, source: VecEnv, cfg: PpoConfig, seed: int = 0):
        self.policy = policy
        self.env = source
        self.cfg = cfg
        self.device = source.device
        spec = source.spec
        torch.manual_seed(seed)
        self.priv_norm = RunningNorm(spec.privileged_dim).to(self.device)
        self.critic = mlp([spec.obs_dim + spec.privileged_dim, *cfg.critic_hidden, 1]).to(
            self.device
        )
        self.policy.to(self.device)
        params = list(self.policy.parameters()) + list(self.critic.parameters())
        self.opt = torch.optim.Adam(params, lr=cfg.lr)
        self.lr = cfg.lr
        self.obs, self.priv = source.reset(seed)
        desync = getattr(source, "desynchronize", None)
        if desync is not None:
            desync()
        self.env_steps = 0
        self.episodes = EpisodeLog()
        self.anchor: MlpGaussianPolicy | None = None
        self.anchor_mask: Callable[[], torch.Tensor] | None = None  # [W] bool, this step's worlds

    def _value(self, obs: torch.Tensor, priv: torch.Tensor) -> torch.Tensor:
        x = torch.cat([self.policy.norm(obs), self.priv_norm(priv)], dim=-1)
        out: torch.Tensor = self.critic(x).squeeze(-1)
        return out

    @torch.no_grad()
    def _collect(self) -> dict[str, torch.Tensor]:
        cfg = self.cfg
        steps, worlds = cfg.steps_per_update, self.env.num_worlds
        spec = self.env.spec
        dev = self.device
        buf = {
            "obs": torch.zeros(steps, worlds, spec.obs_dim, device=dev),
            "priv": torch.zeros(steps, worlds, spec.privileged_dim, device=dev),
            "u": torch.zeros(steps, worlds, spec.action_dim, device=dev),
            "logp": torch.zeros(steps, worlds, device=dev),
            "value": torch.zeros(steps, worlds, device=dev),
            "reward": torch.zeros(steps, worlds, device=dev),
            "done": torch.zeros(steps, worlds, device=dev),
        }
        anchored = self.anchor is not None and self.anchor_mask is not None
        if anchored:
            buf["anchor_mean"] = torch.zeros(steps, worlds, spec.action_dim, device=dev)
            buf["anchor_w"] = torch.zeros(steps, worlds, device=dev)
        for t in range(steps):
            buf["obs"][t] = self.obs
            buf["priv"][t] = self.priv
            if anchored:
                assert self.anchor is not None and self.anchor_mask is not None
                buf["anchor_mean"][t] = self.anchor.distribution(self.obs)[0]
                buf["anchor_w"][t] = self.anchor_mask().float()
            mean, std = self.policy.distribution(self.obs)
            u = mean + std * torch.randn_like(mean)
            buf["u"][t] = u
            buf["logp"][t] = _gaussian_logp(u, mean, std)
            buf["value"][t] = self._value(self.obs, self.priv)
            step = self.env.step(torch.tanh(u))
            reward = step.reward.clone()
            boot = step.truncated & ~step.terminated
            if bool(boot.any()):
                final_value = self._value(step.final_obs, step.final_privileged)
                reward = reward + cfg.gamma * torch.where(
                    boot, final_value, torch.zeros_like(reward)
                )
            buf["reward"][t] = reward
            buf["done"][t] = step.done.float()
            self.episodes.add(step.episode, step.done)
            self.obs, self.priv = step.obs, step.privileged
        self.env_steps += steps * worlds
        last_value = self._value(self.obs, self.priv)
        adv = torch.zeros(steps, worlds, device=dev)
        gae = torch.zeros(worlds, device=dev)
        for t in reversed(range(steps)):
            next_value = last_value if t == steps - 1 else buf["value"][t + 1]
            not_done = 1.0 - buf["done"][t]
            delta = buf["reward"][t] + cfg.gamma * next_value * not_done - buf["value"][t]
            gae = delta + cfg.gamma * cfg.lam * not_done * gae
            adv[t] = gae
        buf["adv"] = adv
        buf["ret"] = adv + buf["value"]
        return buf

    def iterate(self) -> dict[str, float]:
        cfg = self.cfg
        buf = self._collect()
        n = buf["obs"].shape[0] * buf["obs"].shape[1]
        flat = {k: v.reshape(n, *v.shape[2:]) for k, v in buf.items()}
        adv = flat["adv"]
        flat["adv"] = (adv - adv.mean()) / (adv.std() + 1e-8)
        size = n // cfg.minibatches
        stats: dict[str, float] = {
            "policy_loss": 0.0,
            "value_loss": 0.0,
            "kl": 0.0,
            "clipfrac": 0.0,
        }
        if "anchor_w" in flat:
            stats["anchor_loss"] = 0.0
        count = 0
        for _ in range(cfg.epochs):
            perm = torch.randperm(n, device=self.device)
            for i in range(cfg.minibatches):
                idx = perm[i * size : (i + 1) * size]
                mean, std = self.policy.distribution(flat["obs"][idx])
                logp = _gaussian_logp(flat["u"][idx], mean, std)
                ratio = (logp - flat["logp"][idx]).exp()
                a = flat["adv"][idx]
                surrogate = torch.min(ratio * a, ratio.clamp(1 - cfg.clip, 1 + cfg.clip) * a)
                value = self._value(flat["obs"][idx], flat["priv"][idx])
                value_loss = (value - flat["ret"][idx]).pow(2).mean()
                entropy = (std.log() + 0.5 * math.log(2 * math.pi * math.e)).sum(-1).mean()
                loss = -surrogate.mean() + cfg.value_coef * value_loss - cfg.entropy_coef * entropy
                if "anchor_w" in flat:
                    anchor_loss = self._anchor_loss(mean, std, flat, idx)
                    loss = loss + cfg.anchor_coef * anchor_loss
                    stats["anchor_loss"] += float(anchor_loss.detach())
                with torch.no_grad():
                    kl = (flat["logp"][idx] - logp).mean().abs()
                    self._adapt_lr(float(kl))
                self.opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(
                    list(self.policy.parameters()) + list(self.critic.parameters()),
                    cfg.max_grad_norm,
                )
                self.opt.step()
                stats["policy_loss"] += float(-surrogate.detach().mean())
                stats["value_loss"] += float(value_loss.detach())
                stats["kl"] += float(kl)
                stats["clipfrac"] += float(((ratio - 1).abs() > cfg.clip).float().mean())
                count += 1
        stats = {k: v / count for k, v in stats.items()}
        if "anchor_w" in flat:
            stats["anchor_share"] = float(flat["anchor_w"].mean())
        # After the update, so the epochs see the normalization the rollout was collected with.
        self.policy.norm.update(buf["obs"])
        self.priv_norm.update(buf["priv"])
        stats["lr"] = self.lr
        stats["std"] = float(self.policy.log_std.detach().exp().mean())
        stats["reward_per_step"] = float(buf["reward"].mean())
        stats["env_steps"] = float(self.env_steps)
        stats.update(self.episodes.summary())
        return stats

    @staticmethod
    def _anchor_loss(
        mean: torch.Tensor, std: torch.Tensor, flat: dict[str, torch.Tensor], idx: torch.Tensor
    ) -> torch.Tensor:
        # Clamped pre-tanh: past +-2.5 the action is within 1.3% of full and the gap is moot.
        gap = (mean.clamp(-2.5, 2.5) - flat["anchor_mean"][idx].clamp(-2.5, 2.5)) / std.detach()
        w = flat["anchor_w"][idx]
        return (w * 0.5 * gap.pow(2).sum(-1)).sum() / w.sum().clamp(min=1.0)

    def _adapt_lr(self, kl: float) -> None:
        cfg = self.cfg
        if kl > 2.0 * cfg.desired_kl:
            self.lr = max(cfg.min_lr, self.lr / 1.5)
        elif kl < 0.5 * cfg.desired_kl:
            self.lr = min(cfg.max_lr, self.lr * 1.5)
        for group in self.opt.param_groups:
            group["lr"] = self.lr

    def state_dict(self) -> dict[str, Any]:
        return {
            "policy": self.policy.state_dict(),
            "critic": self.critic.state_dict(),
            "priv_norm": self.priv_norm.state_dict(),
            "opt": self.opt.state_dict(),
            "lr": self.lr,
            "env_steps": self.env_steps,
        }

    def load_state_dict(self, state: dict[str, Any]) -> None:
        self.policy.load_state_dict(state["policy"])
        self.critic.load_state_dict(state["critic"])
        self.priv_norm.load_state_dict(state["priv_norm"])
        self.opt.load_state_dict(state["opt"])
        self.lr = float(state["lr"])
        self.env_steps = int(state["env_steps"])
