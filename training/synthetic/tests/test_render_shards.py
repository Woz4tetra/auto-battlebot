"""render_shards.py: the resumed-run seed in the docker command."""

from pathlib import Path

import render_shards
from synthgen.shards import ShardRun, plan_shard_runs


def _seed_of(command: list[str]) -> int:
    return int(command[command.index("--seed") + 1])


def _command(run: ShardRun, start: int) -> list[str]:
    return render_shards.render_command(
        run, start, run.end_index - start, "config.toml", Path("/parts/x"), "image"
    )


def test_resume_seed_clear_of_base_seeds() -> None:
    runs = plan_shard_runs(20000, 3, ("pinhole", "rectified", "distorted"), seed_base=0)
    base_seeds = {run.seed for run in runs}
    for run in runs:
        assert _seed_of(_command(run, run.start_index)) == run.seed
        for start in range(run.start_index + 1, run.end_index, 997):
            resumed = _seed_of(_command(run, start))
            assert resumed != run.seed
            assert resumed not in base_seeds
