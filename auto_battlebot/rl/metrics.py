"""Episode metrics shared by the evaluator, the training logs and the lockstep recordings.

Hits landed per hit taken is the primary score (plan requirement 1). It is a ratio of totals over
the scenario set, not a mean of per-episode ratios, so an episode with no contact counts for
nothing instead of dividing by zero.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

import torch


@dataclass
class EpisodeMetrics:
    episodes: int
    success_rate: float  # episodes with at least one strike
    clean_success_rate: float  # first strike before any hit taken
    strikes_per_episode: float
    rear_strikes_per_episode: float
    hits_taken_per_episode: float
    hits_per_hit_taken: float
    median_time_to_first_strike_s: float  # episodes with no strike count as the episode length
    danger_s_per_episode: float
    wall_events_per_episode: float
    holes_per_episode: float = 0.0
    flips_per_episode: float = 0.0
    block_contacts_per_episode: float = 0.0

    def as_dict(self) -> dict[str, float]:
        return {k: float(v) for k, v in asdict(self).items()}


def summarize(episode: dict[str, torch.Tensor], episode_s: float) -> EpisodeMetrics:
    """`episode` holds one row per episode: the `Step.episode` totals."""
    strikes = episode["strikes"].float()
    hits = episode["hits_taken"].float()
    first_strike = episode["first_strike_s"].float()
    first_hit = episode["first_hit_s"].float()
    n = strikes.numel()

    def mean(key: str) -> float:
        return float(episode[key].float().mean()) if key in episode else 0.0

    struck = strikes > 0
    clean = struck & (first_strike < first_hit)
    ttfs = torch.where(
        torch.isfinite(first_strike), first_strike, torch.full_like(first_strike, episode_s)
    )
    return EpisodeMetrics(
        episodes=n,
        success_rate=float(struck.float().mean()),
        clean_success_rate=float(clean.float().mean()),
        strikes_per_episode=float(strikes.mean()),
        rear_strikes_per_episode=mean("rear_strikes"),
        hits_taken_per_episode=float(hits.mean()),
        hits_per_hit_taken=float(strikes.sum() / hits.sum().clamp(min=1.0)),
        median_time_to_first_strike_s=float(ttfs.median()),
        danger_s_per_episode=mean("danger_s"),
        wall_events_per_episode=mean("wall_events"),
        holes_per_episode=mean("holes"),
        flips_per_episode=mean("flips"),
        block_contacts_per_episode=mean("block_contacts"),
    )
