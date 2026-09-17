"""Estimator tests for training/gpu_queue.py.

`training/` is deliberately not a package, so the module is loaded by path rather than
imported. That is not a sys.path write: nothing else in the tree gains an import route.
"""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
from typing import Any

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("gpu_queue", REPO_ROOT / "training" / "gpu_queue.py")
assert SPEC and SPEC.loader
queue = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(queue)


def make_job(
    job_id: int,
    command: list[str],
    seconds: float | None = None,
    hint: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """A finished job that took `seconds`, or a queued one when seconds is None."""
    job: dict[str, Any] = {
        "id": job_id,
        "name": f"job{job_id}",
        "command": command,
        "hint": hint or {},
        "state": "queued",
        "started_at": None,
        "finished_at": None,
    }
    if seconds is not None:
        job |= {
            "state": "done",
            "started_at": "2026-09-16T00:00:00",
            "finished_at": (
                f"2026-09-16T{int(seconds // 3600):02d}:"
                f"{int(seconds % 3600 // 60):02d}:{int(seconds % 60):02d}"
            ),
        }
    return job


def arm_command(arm: str, model: str) -> list[str]:
    """The shape run_domain_mix_arm.sh jobs are submitted with: env prefix, no .py, no -e."""
    return [
        "env",
        "ARM_DATE=2026-09-13",
        "bash",
        "training/yolo/run_domain_mix_arm.sh",
        "training/data/domain_mix_arms_2026-09-13",
        arm,
        model,
    ]


class TestParseDuration:
    @pytest.mark.parametrize(
        ("text", "expected"),
        [("6h30m", 23400), ("90m", 5400), ("3600s", 3600), ("3600", 3600), ("2h", 7200)],
    )
    def test_accepts(self, text: str, expected: float) -> None:
        assert queue.parse_duration(text) == expected

    @pytest.mark.parametrize("text", ["", "soon", "6j", "h"])
    def test_rejects(self, text: str) -> None:
        with pytest.raises(argparse.ArgumentTypeError):
            queue.parse_duration(text)


class TestScriptName:
    def test_env_prefixed_shell_job_names_the_shell_script(self) -> None:
        """The bug that pooled every arm under "env": no .py in the command."""
        job = make_job(1, arm_command("d40000", "yolo26x-pose"))
        assert queue.script_name(job) == "run_domain_mix_arm.sh"

    def test_python_job_still_names_the_python_script(self) -> None:
        job = make_job(1, ["venv/bin/python", "training/yolo/train.py", "d.yml", "yolo26s-pose"])
        assert queue.script_name(job) == "train.py"


class TestJobProfile:
    def test_declared_profile_wins(self) -> None:
        job = make_job(1, arm_command("d40000", "yolo26x-pose"), hint={"profile": "custom"})
        assert queue.job_profile(job) == "custom"

    def test_falls_back_to_model_and_size(self) -> None:
        job = make_job(1, arm_command("d40000", "yolo26x-pose"))
        assert queue.job_profile(job) == "yolo26x-pose@default"


class TestWorkEstimate:
    def test_scales_a_measured_rate_by_this_job_s_work(self) -> None:
        """Two hours for 1 M units means six for 3 M, which epoch counts alone cannot say."""
        past = make_job(1, arm_command("d20000", "yolo26s-pose"), 7200, {"work": 1_000_000})
        job = make_job(2, arm_command("d40000", "yolo26s-pose"), hint={"work": 3_000_000})
        state = {"jobs": [past, job]}
        seconds, basis = queue.work_estimate(state, job)
        assert seconds == pytest.approx(21600)
        assert "1 past yolo26s-pose@default run" in basis

    def test_refuses_to_borrow_a_rate_across_model_sizes(self) -> None:
        """The 2h18m-for-most-of-a-day miss: an x cannot inherit an s's seconds per unit."""
        past = make_job(1, arm_command("d40000", "yolo26s-pose"), 7200, {"work": 1_000_000})
        job = make_job(2, arm_command("d40000", "yolo26x-pose"), hint={"work": 1_000_000})
        seconds, _ = queue.work_estimate({"jobs": [past, job]}, job)
        assert seconds is None

    def test_no_work_declared_is_not_an_estimate(self) -> None:
        job = make_job(2, arm_command("d40000", "yolo26s-pose"))
        assert queue.work_estimate({"jobs": [job]}, job) == (None, "")


class TestTypicalSeconds:
    def test_eta_beats_history(self) -> None:
        past = make_job(1, arm_command("d20000", "yolo26s-pose"), 7200, {"work": 1_000_000})
        job = make_job(
            2, arm_command("d40000", "yolo26s-pose"), hint={"work": 3_000_000, "eta_seconds": 999}
        )
        seconds, basis = queue.typical_seconds({"jobs": [past, job]}, job)
        assert (seconds, basis) == (999, "as submitted")

    def test_work_beats_the_shape_fallback(self) -> None:
        past = make_job(1, arm_command("d20000", "yolo26s-pose"), 7200, {"work": 1_000_000})
        job = make_job(2, arm_command("d40000", "yolo26s-pose"), hint={"work": 2_000_000})
        seconds, _ = queue.typical_seconds({"jobs": [past, job]}, job)
        assert seconds == pytest.approx(14400)

    def test_declared_work_with_no_rate_says_so_rather_than_guessing(self) -> None:
        past = make_job(1, arm_command("d40000", "yolo26s-pose"), 7200, {"work": 1_000_000})
        job = make_job(2, arm_command("d40000", "yolo26x-pose"), hint={"work": 1_000_000})
        seconds, basis = queue.typical_seconds({"jobs": [past, job]}, job)
        assert seconds is None
        assert basis == "no comparable runs yet"

    def test_jobs_without_hints_keep_the_old_behaviour(self) -> None:
        """Everything already in state.json predates these flags and must still predict."""
        past = make_job(1, arm_command("d20000", "yolo26s-pose"), 7200)
        job = make_job(2, arm_command("d40000", "yolo26s-pose"))
        seconds, basis = queue.typical_seconds({"jobs": [past, job]}, job)
        assert seconds == pytest.approx(7200)
        assert "run_domain_mix_arm.sh" in basis or "yolo26s-pose" in basis
