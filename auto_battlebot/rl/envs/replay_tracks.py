"""Human opponent tracks for the `replay` behaviour, sampled into each world's arena.

`simulation/opponent_tracks/replay_tracks.npz` (written by `playground/rl/extract_replay_tracks.py`)
holds positions as fractions of each recording's field half size, at 50 Hz, so a track replays
in any arena by scaling. A world starts its track at a random offset and holds the last pose
once the track ends.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import torch

REPO = Path(__file__).resolve().parents[3]
DEFAULT_PATH = REPO / "simulation/opponent_tracks/replay_tracks.npz"


class ReplayTracks:
    def __init__(
        self, holdout: bool, device: torch.device, dtype: torch.dtype, path: Path = DEFAULT_PATH
    ) -> None:
        data = np.load(path)
        keep = data["holdout"] == holdout
        self.tracks = torch.as_tensor(data["tracks"][keep], device=device, dtype=dtype)
        self.lengths = torch.as_tensor(data["lengths"][keep], device=device)
        self.rate = float(data["rate_hz"])
        self.count = int(keep.sum())
        if self.count == 0:
            raise ValueError(f"{path}: no {'holdout' if holdout else 'training'} tracks")

    def start_span(self, pick: torch.Tensor) -> torch.Tensor:
        """Seconds a world may skip into its track and still leave 20 s to play."""
        seconds = self.lengths[pick].to(self.tracks.dtype) / self.rate
        return (seconds - 20.0).clamp(min=0.0)

    def sample(
        self, pick: torch.Tensor, t: torch.Tensor, half: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Pose at time `t` of each world's track, scaled to the half size `half` [W, 2]."""
        idx = (t * self.rate).long().clamp(min=0)
        idx = torch.minimum(idx, self.lengths[pick] - 1)
        row = self.tracks[pick, idx]
        return row[:, :2] * half, row[:, 2]
