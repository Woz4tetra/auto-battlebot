from __future__ import annotations

import os
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "firmware/mr_stabs_mk2/scripts/ota"

FAKE = '#!/bin/sh\necho "$(basename "$0") $*" >> "$FAKE_LOG"\n'


def run_ota(tmp_path: Path, extra_env: dict[str, str]) -> list[str]:
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    for name in ("platformio", "ssh", "scp"):
        fake = bin_dir / name
        fake.write_text(FAKE)
        fake.chmod(0o755)
    home = tmp_path / "home"
    home.mkdir(exist_ok=True)
    log = tmp_path / "log"
    env = {
        "PATH": f"{bin_dir}:/usr/bin:/bin",
        "HOME": str(home),
        "FAKE_LOG": str(log),
        **extra_env,
    }
    subprocess.run([str(SCRIPT)], env=env, check=True, capture_output=True)
    return log.read_text().splitlines()


def test_jetson_key_pins_the_identity_on_ssh_and_scp(tmp_path: Path) -> None:
    lines = run_ota(tmp_path, {"JETSON_KEY": "/keys/jetson", "JETSON_HOST": "orin"})
    scp = next(line for line in lines if line.startswith("scp "))
    ssh = next(line for line in lines if line.startswith("ssh "))
    for line in (scp, ssh):
        assert "-i /keys/jetson -o IdentitiesOnly=yes" in line


def test_key_named_after_the_host_is_picked_up(tmp_path: Path) -> None:
    key = tmp_path / "home/.ssh/orin"
    key.parent.mkdir(parents=True, exist_ok=True)
    key.write_text("")
    lines = run_ota(tmp_path, {"JETSON_HOST": "orin.local"})
    assert all(
        f"-i {key} -o IdentitiesOnly=yes" in line for line in lines if line[:3] in ("ssh", "scp")
    )


def test_no_key_leaves_the_agent_to_choose(tmp_path: Path) -> None:
    lines = run_ota(tmp_path, {"JETSON_HOST": "orin"})
    transport = [line for line in lines if line[:3] in ("ssh", "scp")]
    assert len(transport) == 2
    assert not any("IdentitiesOnly" in line for line in transport)
    assert os.path.exists(SCRIPT)
