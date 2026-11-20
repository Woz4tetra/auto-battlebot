"""Estimator tests for auto_battlebot/gpu_queue.py."""

from __future__ import annotations

import argparse
import contextlib
from datetime import datetime
from pathlib import Path
from typing import Any

import pytest

from auto_battlebot import gpu_queue as queue


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


class TestJobEnv:
    def test_job_env_strips_unsafe_inherited_vars(self, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "1")
        monkeypatch.setenv("PYTHONPATH", "/somewhere")
        job = make_job(1, ["true"])
        job["env"] = {"NCCL_P2P_DISABLE": "1"}
        env = queue.job_env(job)
        assert "CUDA_VISIBLE_DEVICES" not in env
        assert "PYTHONPATH" not in env
        assert env["NCCL_P2P_DISABLE"] == "1"


class TestGpuBusy:
    def test_gpu_users_unreadable_smi_is_busy(self, monkeypatch: pytest.MonkeyPatch) -> None:
        def unreadable(query: str) -> list[list[str]]:
            raise OSError("nvidia-smi missing")

        monkeypatch.setattr(queue, "nvidia_smi_rows", unreadable)
        assert queue.gpu_busy_processes(set()) != []


class TestLogProgress:
    def write_log(self, tmp_path: Path, text: str) -> dict[str, Any]:
        log = tmp_path / "job.log"
        log.write_text(text)
        return {"log": str(log)}

    def test_reads_epoch_counter_from_ultralytics_line(self, tmp_path: Path) -> None:
        job = self.write_log(tmp_path, "      Epoch    GPU_mem\n  46/100      12.7G  0.5\n")
        reading = queue.log_progress(job)
        assert reading is not None
        fraction, label = reading
        assert label == "46/100"
        assert fraction == pytest.approx(45 / 100)

    def test_progress_bar_carriage_returns_split_into_lines(self, tmp_path: Path) -> None:
        job = self.write_log(tmp_path, "  1/100      12.7G  0.5\r  2/100      12.7G  0.5\r")
        reading = queue.log_progress(job)
        assert reading is not None
        assert reading[1] == "2/100"


class TestTailOffset:
    def test_tail_offset_ignores_carriage_returns(self, tmp_path: Path) -> None:
        log = tmp_path / "job.log"
        log.write_bytes(b"start\nbar 1\rbar 2\rbar 3\rend\n")
        assert queue.tail_offset(log, 2) == len(b"start\n")


class TestForecast:
    def test_unknown_duration_propagates_to_later_jobs(self) -> None:
        first = make_job(1, ["bash", "run.sh"])
        second = make_job(2, ["bash", "run.sh"])
        plan = queue.forecast({"jobs": [first, second]}, datetime(2026, 9, 16))
        assert plan[1]["finish"] is None
        assert plan[2]["finish"] is None


class TestIdleTick:
    def test_idle_tick_resumes_before_exit(self, monkeypatch: pytest.MonkeyPatch) -> None:
        calls: list[str] = []
        monkeypatch.setattr(
            queue, "run_yield_hooks", lambda action, devices=None: calls.append(action)
        )
        monkeypatch.setattr(queue, "exit_if_idle", lambda: calls.append("exit"))
        args = argparse.Namespace(idle_exit=60.0)
        still_paused = queue.idle_tick(args, 120.0, True)
        assert calls == ["resume", "exit"]
        assert not still_paused


class TestSubmitLiveness:
    def test_submit_and_idle_exit_cannot_orphan_a_job(
        self, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
    ) -> None:
        held: list[bool] = []
        state: dict[str, Any] = {"next_id": 1, "jobs": [], "worker_stop": False}
        lock_held = {"value": False}

        @contextlib.contextmanager
        def fake_locked_state() -> Any:
            lock_held["value"] = True
            try:
                yield state
            finally:
                lock_held["value"] = False

        def fake_worker_alive() -> bool:
            held.append(lock_held["value"])
            return True

        monkeypatch.setattr(queue, "locked_state", fake_locked_state)
        monkeypatch.setattr(queue, "worker_alive", fake_worker_alive)
        args = argparse.Namespace(
            command=["true"],
            devices=[0],
            name="n",
            cwd=str(tmp_path),
            by="t",
            work=None,
            profile=None,
            eta=None,
        )
        assert queue.cmd_submit(args) == 0
        assert held == [True]


class TestSubmitHints:
    def test_submit_without_hints_records_no_hint_keys(
        self, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
    ) -> None:
        state: dict[str, Any] = {"next_id": 1, "jobs": [], "worker_stop": False}

        @contextlib.contextmanager
        def fake_locked_state() -> Any:
            yield state

        monkeypatch.setattr(queue, "locked_state", fake_locked_state)
        monkeypatch.setattr(queue, "worker_alive", lambda: True)
        args = argparse.Namespace(
            command=["true"],
            devices=[0],
            name="n",
            cwd=str(tmp_path),
            by="t",
            work=None,
            profile=None,
            eta=None,
        )
        queue.cmd_submit(args)
        assert state["jobs"][0]["hint"] == {}


class TestWorkerLoop:
    def _run(self, monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> dict[str, Any]:
        import json
        import types

        monkeypatch.setattr(queue, "QUEUE_DIR", tmp_path)
        monkeypatch.setattr(queue, "LOG_DIR", tmp_path / "logs")
        monkeypatch.setattr(queue, "STATE_PATH", tmp_path / "state.json")
        monkeypatch.setattr(queue, "STATE_LOCK", tmp_path / "state.lock")
        monkeypatch.setattr(queue, "WORKER_LOCK", tmp_path / "worker.lock")
        state = {
            "next_id": 3,
            "jobs": [
                make_job(1, ["true"]) | {"cancel_requested": False},
                make_job(2, ["true"]) | {"cancel_requested": False},
            ],
            "worker_stop": False,
        }
        queue.STATE_PATH.write_text(json.dumps(state))
        clock = [0.0]
        seen: dict[str, Any] = {"hooks": [], "idle": []}

        def run_job(job: dict[str, Any]) -> None:
            clock[0] += 1000.0
            with queue.locked_state() as fresh:
                queue.find_job(fresh, job["id"])["state"] = "done"

        def idle_tick(_args: argparse.Namespace, idle: float, yielded: bool) -> bool:
            seen["idle"].append(idle)
            return yielded

        def sleep(seconds: float) -> None:
            clock[0] += seconds
            with queue.locked_state() as fresh:
                fresh["worker_stop"] = True

        fake_time = types.SimpleNamespace(monotonic=lambda: clock[0], sleep=sleep)
        monkeypatch.setattr(queue, "time", fake_time)
        monkeypatch.setattr(queue, "run_job", run_job)
        monkeypatch.setattr(queue, "idle_tick", idle_tick)
        monkeypatch.setattr(queue, "wait_for_gpus", lambda _job_id: True)
        monkeypatch.setattr(
            queue, "run_yield_hooks", lambda action, devices=None: seen["hooks"].append(action)
        )
        assert queue.cmd_worker(argparse.Namespace(idle_exit=0.0)) == 0
        return seen

    def test_tenants_pause_before_every_job(
        self, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
    ) -> None:
        seen = self._run(monkeypatch, tmp_path)
        assert seen["hooks"].count("pause") == 2

    def test_idle_time_counts_from_end_of_last_job(
        self, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
    ) -> None:
        seen = self._run(monkeypatch, tmp_path)
        assert seen["idle"] == [0.0]
