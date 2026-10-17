"""`fasttd3`: TD3 for massively parallel simulation, after Seo et al. 2025 (FastTD3).

What makes it fast, kept here: thousands of parallel worlds feeding a GPU replay buffer, a large
batch, n-step returns, a distributional (C51) critic with clipped double Q, and per-world
exploration noise drawn from a range so some worlds explore and some exploit. The actor is the
same Dense/ELU stack as `mlp_gaussian` (its mean path), so it exports to RTNeural unchanged. No
LayerNorm anywhere: RTNeural cannot run it (plan section 11), so this arm trains without it.

The critic sees the privileged inputs, as PPO's does.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import torch
from torch import nn
from torch.nn import functional

from auto_battlebot.rl.env import VecEnv
from auto_battlebot.rl.learners.base import LEARNERS, EpisodeLog
from auto_battlebot.rl.policies.mlp import MlpGaussianPolicy, RunningNorm, mlp


@dataclass
class FastTd3Config:
    buffer_steps: int = 256  # per world
    # The paper's 32768 costs 100 ms per update on the laptop's RTX 4080 with this critic; 8192
    # costs 18 ms, so the laptop arm runs 8192.
    batch_size: int = 8192
    n_step: int = 3
    gamma: float = 0.99
    tau: float = 0.1
    actor_lr: float = 3e-4
    critic_lr: float = 3e-4
    updates_per_step: int = 2
    policy_delay: int = 2
    noise_std_min: float = 0.001
    noise_std_max: float = 0.4
    target_noise: float = 0.001
    target_noise_clip: float = 0.5
    atoms: int = 101
    v_min: float = -80.0
    v_max: float = 80.0
    critic_hidden: tuple[int, ...] = (1024, 512, 256)
    learning_starts: int = 16  # buffer steps of uniform-random actions before updates
    max_grad_norm: float = 0.0


class DistributionalCritic(nn.Module):
    def __init__(self, in_dim: int, hidden: tuple[int, ...], atoms: int) -> None:
        super().__init__()
        self.q1 = mlp([in_dim, *hidden, atoms])
        self.q2 = mlp([in_dim, *hidden, atoms])

    def forward(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        return self.q1(x), self.q2(x)


@LEARNERS.register("fasttd3")
class FastTd3:
    def __init__(
        self, policy: MlpGaussianPolicy, source: VecEnv, cfg: FastTd3Config, seed: int = 0
    ):
        torch.manual_seed(seed)
        self.policy = policy
        self.env = source
        self.cfg = cfg
        self.device = source.device
        spec = source.spec
        w = source.num_worlds
        dev = self.device
        self.policy.to(dev)
        self.target_actor = mlp([spec.obs_dim, *policy.cfg.hidden, spec.action_dim]).to(dev)
        self.target_actor.load_state_dict(policy.actor.state_dict())
        in_dim = spec.obs_dim + spec.privileged_dim + spec.action_dim
        self.critic = DistributionalCritic(in_dim, cfg.critic_hidden, cfg.atoms).to(dev)
        self.target_critic = DistributionalCritic(in_dim, cfg.critic_hidden, cfg.atoms).to(dev)
        self.target_critic.load_state_dict(self.critic.state_dict())
        self.priv_norm = RunningNorm(spec.privileged_dim).to(dev)
        self.actor_opt = torch.optim.Adam(policy.actor.parameters(), lr=cfg.actor_lr)
        self.critic_opt = torch.optim.Adam(self.critic.parameters(), lr=cfg.critic_lr)
        self.support = torch.linspace(cfg.v_min, cfg.v_max, cfg.atoms, device=dev)
        self.delta_z = (cfg.v_max - cfg.v_min) / (cfg.atoms - 1)

        cap = cfg.buffer_steps
        self.cap = cap
        self.buf_obs = torch.zeros(cap, w, spec.obs_dim, device=dev)
        self.buf_priv = torch.zeros(cap, w, spec.privileged_dim, device=dev)
        self.buf_act = torch.zeros(cap, w, spec.action_dim, device=dev)
        self.buf_rew = torch.zeros(cap, w, device=dev)
        self.buf_term = torch.zeros(cap, w, device=dev)
        self.buf_done = torch.zeros(cap, w, device=dev)
        self.buf_next_obs = torch.zeros(cap, w, spec.obs_dim, device=dev)
        self.buf_next_priv = torch.zeros(cap, w, spec.privileged_dim, device=dev)
        self.ptr = 0
        self.filled = 0

        self.noise_std = self._draw_noise(
            torch.ones(w, dtype=torch.bool, device=dev), torch.zeros(w, device=dev)
        )
        self.obs, self.priv = source.reset(seed)
        desync = getattr(source, "desynchronize", None)
        if desync is not None:
            desync()
        self.env_steps = 0
        self.updates = 0
        self.episodes = EpisodeLog()

    def _draw_noise(self, mask: torch.Tensor, current: torch.Tensor) -> torch.Tensor:
        c = self.cfg
        draw = c.noise_std_min + (c.noise_std_max - c.noise_std_min) * torch.rand_like(current)
        return torch.where(mask, draw, current)

    def _critic_in(
        self, obs: torch.Tensor, priv: torch.Tensor, action: torch.Tensor
    ) -> torch.Tensor:
        return torch.cat([self.policy.norm(obs), self.priv_norm(priv), action], dim=-1)

    def _actor(self, net: nn.Module, obs: torch.Tensor) -> torch.Tensor:
        out: torch.Tensor = torch.tanh(net(self.policy.norm(obs)))
        return out

    @torch.no_grad()
    def _collect(self) -> dict[str, float]:
        if self.filled < self.cfg.learning_starts:
            # Uniform actions until learning starts, as TD3 does: a buffer filled by the untrained
            # actor plus small noise holds one action sign per state, and the critic then cannot
            # see what the other sign earns.
            action = torch.rand(self.env.num_worlds, 2, device=self.device) * 2.0 - 1.0
        else:
            action = self._actor(self.policy.actor, self.obs)
            action = (action + self.noise_std[:, None] * torch.randn_like(action)).clamp(-1.0, 1.0)
        step = self.env.step(action)
        i = self.ptr
        self.buf_obs[i] = self.obs
        self.buf_priv[i] = self.priv
        self.buf_act[i] = action
        self.buf_rew[i] = step.reward
        self.buf_term[i] = step.terminated.float()
        self.buf_done[i] = step.done.float()
        self.buf_next_obs[i] = step.final_obs
        self.buf_next_priv[i] = step.final_privileged
        self.ptr = (self.ptr + 1) % self.cap
        self.filled = min(self.filled + 1, self.cap)
        self.episodes.add(step.episode, step.done)
        self.noise_std = self._draw_noise(step.done, self.noise_std)
        self.policy.norm.update(self.obs)
        self.priv_norm.update(self.priv)
        self.obs, self.priv = step.obs, step.privileged
        self.env_steps += self.env.num_worlds
        return {"reward_per_step": float(step.reward.mean())}

    def _sample(self) -> dict[str, torch.Tensor]:
        """n-step transitions: start rows whose n following steps are all in the buffer."""
        c = self.cfg
        w = self.env.num_worlds
        usable = self.filled - c.n_step
        offset = torch.randint(0, usable, (c.batch_size,), device=self.device)
        start = (self.ptr - self.filled + offset) % self.cap
        world = torch.randint(0, w, (c.batch_size,), device=self.device)
        ret = torch.zeros(c.batch_size, device=self.device)
        discount = torch.ones(c.batch_size, device=self.device)
        alive = torch.ones(c.batch_size, device=self.device)
        last = start.clone()
        terminal = torch.zeros(c.batch_size, device=self.device)
        for k in range(c.n_step):
            idx = (start + k) % self.cap
            ret += alive * discount * self.buf_rew[idx, world]
            last = torch.where(alive > 0, idx, last)
            terminal = torch.where(alive > 0, self.buf_term[idx, world], terminal)
            discount = discount * torch.where(
                alive > 0, torch.full_like(discount, c.gamma), torch.ones_like(discount)
            )
            alive = alive * (1.0 - self.buf_done[idx, world])
        return {
            "obs": self.buf_obs[start, world],
            "priv": self.buf_priv[start, world],
            "action": self.buf_act[start, world],
            "ret": ret,
            "discount": discount * (1.0 - terminal),
            "next_obs": self.buf_next_obs[last, world],
            "next_priv": self.buf_next_priv[last, world],
        }

    def _project(
        self, ret: torch.Tensor, discount: torch.Tensor, probs: torch.Tensor
    ) -> torch.Tensor:
        c = self.cfg
        tz = (ret[:, None] + discount[:, None] * self.support[None, :]).clamp(c.v_min, c.v_max)
        b = (tz - c.v_min) / self.delta_z
        lo, hi = b.floor().long(), b.ceil().long()
        lo = torch.where((hi > 0) & (lo == hi), lo - 1, lo)
        hi = torch.where((lo < c.atoms - 1) & (lo == hi), hi + 1, hi)
        out = torch.zeros_like(probs)
        out.scatter_add_(1, lo, probs * (hi.float() - b))
        out.scatter_add_(1, hi, probs * (b - lo.float()))
        return out

    def _update(self) -> dict[str, float]:
        c = self.cfg
        batch = self._sample()
        with torch.no_grad():
            next_a = self._actor(self.target_actor, batch["next_obs"])
            noise = (torch.randn_like(next_a) * c.target_noise).clamp(
                -c.target_noise_clip, c.target_noise_clip
            )
            next_a = (next_a + noise).clamp(-1.0, 1.0)
            l1, l2 = self.target_critic(
                self._critic_in(batch["next_obs"], batch["next_priv"], next_a)
            )
            p1, p2 = functional.softmax(l1, -1), functional.softmax(l2, -1)
            q1 = (p1 * self.support).sum(-1)
            q2 = (p2 * self.support).sum(-1)
            target_probs = torch.where((q1 < q2)[:, None], p1, p2)
            target = self._project(batch["ret"], batch["discount"], target_probs)
        logits1, logits2 = self.critic(
            self._critic_in(batch["obs"], batch["priv"], batch["action"])
        )
        critic_loss = (
            -(target * functional.log_softmax(logits1, -1)).sum(-1).mean()
            - (target * functional.log_softmax(logits2, -1)).sum(-1).mean()
        )
        self.critic_opt.zero_grad()
        critic_loss.backward()
        if c.max_grad_norm > 0:
            nn.utils.clip_grad_norm_(self.critic.parameters(), c.max_grad_norm)
        self.critic_opt.step()
        stats = {"critic_loss": float(critic_loss.detach())}

        if self.updates % c.policy_delay == 0:
            a = self._actor(self.policy.actor, batch["obs"])
            l1, l2 = self.critic(self._critic_in(batch["obs"], batch["priv"], a))
            q = torch.minimum(
                (functional.softmax(l1, -1) * self.support).sum(-1),
                (functional.softmax(l2, -1) * self.support).sum(-1),
            )
            actor_loss = -q.mean()
            self.actor_opt.zero_grad()
            actor_loss.backward()
            if c.max_grad_norm > 0:
                nn.utils.clip_grad_norm_(self.policy.actor.parameters(), c.max_grad_norm)
            self.actor_opt.step()
            stats["actor_loss"] = float(actor_loss.detach())
            stats["q"] = float(q.detach().mean())
            with torch.no_grad():
                for tp, p in zip(self.target_actor.parameters(), self.policy.actor.parameters()):
                    tp.lerp_(p, c.tau)
                for tp, p in zip(self.target_critic.parameters(), self.critic.parameters()):
                    tp.lerp_(p, c.tau)
        self.updates += 1
        return stats

    def iterate(self) -> dict[str, float]:
        """Sixteen environment steps for every world, each followed by `updates_per_step` updates
        once the buffer holds `learning_starts` steps."""
        totals: dict[str, float] = {}
        count = 0
        for _ in range(16):
            stats = self._collect()
            if self.filled >= max(self.cfg.learning_starts, self.cfg.n_step + 1):
                for _ in range(self.cfg.updates_per_step):
                    stats.update(self._update())
            for k, v in stats.items():
                totals[k] = totals.get(k, 0.0) + v
            count += 1
        out = {k: v / count for k, v in totals.items()}
        out["env_steps"] = float(self.env_steps)
        out["std"] = float(self.noise_std.mean())
        out.update(self.episodes.summary())
        return out

    def state_dict(self) -> dict[str, Any]:
        return {
            "policy": self.policy.state_dict(),
            "target_actor": self.target_actor.state_dict(),
            "critic": self.critic.state_dict(),
            "target_critic": self.target_critic.state_dict(),
            "priv_norm": self.priv_norm.state_dict(),
            "actor_opt": self.actor_opt.state_dict(),
            "critic_opt": self.critic_opt.state_dict(),
            "env_steps": self.env_steps,
        }

    def load_state_dict(self, state: dict[str, Any]) -> None:
        self.policy.load_state_dict(state["policy"])
        self.target_actor.load_state_dict(state["target_actor"])
        self.critic.load_state_dict(state["critic"])
        self.target_critic.load_state_dict(state["target_critic"])
        self.priv_norm.load_state_dict(state["priv_norm"])
        self.actor_opt.load_state_dict(state["actor_opt"])
        self.critic_opt.load_state_dict(state["critic_opt"])
        self.env_steps = int(state["env_steps"])
