import os
import subprocess
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[2] / "training/synthetic/docker/run_synthetic.sh"
GPU_PROBE_EXIT = 125


def run_with_fake_docker(tmp_path, *args):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    log = tmp_path / "docker.log"
    fake = bin_dir / "docker"
    fake.write_text(
        '#!/bin/sh\necho "$@" >> "$FAKE_DOCKER_LOG"\n'
        'case "$*" in *--gpus*) exit "$FAKE_GPU_EXIT";; esac\nexit 0\n'
    )
    fake.chmod(0o755)
    env = {
        **os.environ,
        "PATH": f"{bin_dir}:{os.environ['PATH']}",
        "HOME": str(tmp_path),
        "FAKE_DOCKER_LOG": str(log),
        "FAKE_GPU_EXIT": str(GPU_PROBE_EXIT),
    }
    result = subprocess.run(
        ["bash", str(SCRIPT), *args, "image"], env=env, capture_output=True, text=True
    )
    return result, log.read_text().splitlines() if log.exists() else []


def test_require_gpu_fails_without_gpu(tmp_path):
    result, calls = run_with_fake_docker(tmp_path, "--require-gpu")
    assert result.returncode == GPU_PROBE_EXIT
    assert all("--gpus" in call for call in calls), "a render ran without a GPU"


def test_gpu_falls_back_to_cpu_without_gpu(tmp_path):
    result, calls = run_with_fake_docker(tmp_path, "--gpu")
    assert result.returncode == 0
    assert any("--gpus" not in call for call in calls)
