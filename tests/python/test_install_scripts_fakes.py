import os
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
BASH = "/bin/bash"


class Sandbox:
    def __init__(self, tmp_path: Path) -> None:
        self.root = tmp_path
        self.bin = tmp_path / "bin"
        self.bin.mkdir()
        self.log = tmp_path / "calls.log"
        self.home = tmp_path / "home"
        self.home.mkdir()

    def fake(self, name: str, body: str = "") -> None:
        path = self.bin / name
        path.write_text(f'#!/bin/bash\necho "{name} $*" >> "$FAKE_LOG"\n{body}\n')
        path.chmod(0o755)

    def run(self, script: str, **env: str) -> subprocess.CompletedProcess[str]:
        full_env = {
            "PATH": f"{self.bin}:/usr/bin:/bin",
            "HOME": str(self.home),
            "USER": "tester",
            "FAKE_LOG": str(self.log),
            "LC_ALL": "C",
            **env,
        }
        return subprocess.run(
            [BASH, "-c", script], env=full_env, capture_output=True, text=True, cwd=self.root
        )

    def calls(self) -> list[str]:
        return self.log.read_text().splitlines() if self.log.exists() else []


@pytest.fixture
def sandbox(tmp_path: Path) -> Sandbox:
    box = Sandbox(tmp_path)
    box.fake("sudo", 'exec "$@"')
    return box


def source(script: str) -> str:
    return f"source {ROOT / script}\n"


def test_playback_compose_runs_tini_because_the_entrypoint_builds_before_exec() -> None:
    compose = yaml.safe_load((ROOT / "docker/docker-compose.playback.yml").read_text())
    service = next(iter(compose["services"].values()))
    assert service["init"] is True
    entrypoint = (ROOT / "scripts/docker/container_entrypoint.sh").read_text().splitlines()
    build = next(i for i, line in enumerate(entrypoint) if line.endswith('scripts/build.sh"'))
    handoff = next(i for i, line in enumerate(entrypoint) if line.startswith("exec "))
    assert build < handoff


def test_playback_compose_volume_names_are_literal_and_match_the_first_run_probe() -> None:
    compose = yaml.safe_load((ROOT / "docker/docker-compose.playback.yml").read_text())
    for key, volume in compose["volumes"].items():
        assert volume["name"] == key
    common = (ROOT / "scripts/docker/docker_common.sh").read_text()
    assert "docker volume inspect auto-battlebot-zed-resources" in common
    assert "auto-battlebot-zed-resources" in compose["volumes"]


def test_device_tree_model_read_drops_the_nul_without_a_bash_warning(tmp_path: Path) -> None:
    model = tmp_path / "model"
    model.write_bytes(b"NVIDIA Jetson Orin NX Engineering Reference Developer Kit\x00")
    result = subprocess.run(
        [
            BASH,
            "-c",
            f"source {ROOT / 'install/check_jetson_orin.sh'}\n"
            f'm=$(read_device_tree_model "{model}"); echo "[$m]"',
        ],
        capture_output=True,
        text=True,
    )
    assert result.stdout.strip() == "[NVIDIA Jetson Orin NX Engineering Reference Developer Kit]"
    assert result.stderr == ""


def test_unfiltered_device_tree_read_would_warn_about_the_nul(tmp_path: Path) -> None:
    model = tmp_path / "model"
    model.write_bytes(b"NVIDIA Orin\x00")
    result = subprocess.run(
        [BASH, "-c", f'm=$(cat "{model}"); echo "$m"'], capture_output=True, text=True
    )
    assert "ignored null byte" in result.stderr


def test_dashboard_cable_profile_turns_mdns_off(sandbox: Sandbox) -> None:
    sandbox.fake("nmcli")
    sandbox.fake("apt-get")
    script = source("install/install_dashboard_network.sh") + (
        "check_dashboard_network_prereqs() { :; }\n"
        "configure_avahi() { :; }\n"
        "install_dashboard_firewall() { :; }\n"
        "enable_dashboard_network_units() { :; }\n"
        "verify_dashboard_network() { :; }\n"
        "install_dashboard_network --interface eth9\n"
    )
    result = sandbox.run(script)
    assert result.returncode == 0, result.stderr
    adds = [c for c in sandbox.calls() if c.startswith("nmcli connection add")]
    assert len(adds) == 1
    assert "ifname eth9" in adds[0]
    assert "connection.mdns no" in adds[0]


def stability_conf(tmp_path: Path) -> Path:
    conf = tmp_path / "extlinux.conf"
    conf.write_text("LABEL primary\n      APPEND ${cbootargs} root=/dev/nvme0n1p1 quiet\n")
    return conf


def test_cgroup_disable_memory_is_appended_once_on_the_5_15_kernel(
    sandbox: Sandbox, tmp_path: Path
) -> None:
    sandbox.fake("uname", 'echo "5.15.148-tegra"')
    conf = stability_conf(tmp_path)
    script = (
        source("install/install_jetson_stability.sh") + f'install_cgroup_disable_memory "{conf}"\n'
    )
    assert sandbox.run(script).returncode == 0
    assert sandbox.run(script).returncode == 0
    text = conf.read_text()
    assert text.count("cgroup_disable=memory") == 1
    assert text.splitlines()[1].endswith("quiet cgroup_disable=memory")
    assert len(list(tmp_path.glob("extlinux.conf.bak.*"))) == 1


def test_cgroup_disable_memory_is_skipped_on_the_6_8_kernel(
    sandbox: Sandbox, tmp_path: Path
) -> None:
    sandbox.fake("uname", 'echo "6.8.12-tegra"')
    conf = stability_conf(tmp_path)
    before = conf.read_text()
    script = (
        source("install/install_jetson_stability.sh") + f'install_cgroup_disable_memory "{conf}"\n'
    )
    result = sandbox.run(script)
    assert result.returncode == 0
    assert conf.read_text() == before
    assert not list(tmp_path.glob("extlinux.conf.bak.*"))
    assert not any(c.startswith("sudo") for c in sandbox.calls())


def test_service_masks_mask_bluetooth_and_link_user_units_to_dev_null(sandbox: Sandbox) -> None:
    sandbox.fake(
        "systemctl",
        'case "$1" in list-unit-files) echo "bluetooth.service enabled";; '
        "is-enabled) echo enabled;; esac",
    )
    result = sandbox.run(
        source("install/install_jetson_stability.sh") + "install_jetson_service_masks\n"
    )
    assert result.returncode == 0, result.stderr
    assert "sudo systemctl mask --no-block bluetooth.service" in sandbox.calls()
    link = sandbox.home / ".config/systemd/user/tracker-miner-fs-3.service"
    assert os.readlink(link) == "/dev/null"


def test_opencv_python_path_replaces_a_stale_bashrc_line_with_the_dist_packages_dir(
    sandbox: Sandbox, tmp_path: Path
) -> None:
    prefix = tmp_path / "prefix"
    (prefix / "lib/python3.12/dist-packages/cv2").mkdir(parents=True)
    bashrc = tmp_path / "bashrc"
    bashrc.write_text(
        f"export FOO=1\nexport PYTHONPATH={prefix}/lib/python3.10/dist-packages:$PYTHONPATH\n"
        f"export PYTHONPATH=/elsewhere:$PYTHONPATH\n"
    )
    script = (
        source("install/install_opencv.sh")
        + f'opencv_register_python_path "{prefix}" 3.12 "{bashrc}"\n'
    )
    assert sandbox.run(script).returncode == 0
    assert sandbox.run(script).returncode == 0
    lines = bashrc.read_text().splitlines()
    assert "export FOO=1" in lines
    assert "export PYTHONPATH=/elsewhere:$PYTHONPATH" in lines
    ours = [line for line in lines if str(prefix) in line]
    assert ours == [f"export PYTHONPATH={prefix}/lib/python3.12/dist-packages:$PYTHONPATH"]


def test_opencv_python_path_falls_back_to_site_packages_and_leaves_bashrc_alone_without_cv2(
    sandbox: Sandbox, tmp_path: Path
) -> None:
    prefix = tmp_path / "prefix"
    bashrc = tmp_path / "bashrc"
    bashrc.write_text("export FOO=1\n")
    script = (
        source("install/install_opencv.sh")
        + f'opencv_register_python_path "{prefix}" 3.12 "{bashrc}"\n'
    )
    result = sandbox.run(script)
    assert "no cv2 package found" in result.stdout
    assert bashrc.read_text() == "export FOO=1\n"
    (prefix / "lib/python3.12/site-packages/cv2").mkdir(parents=True)
    sandbox.run(script)
    assert f"{prefix}/lib/python3.12/site-packages:" in bashrc.read_text()


def test_platformio_install_pins_intelhex_before_platformio(sandbox: Sandbox) -> None:
    sandbox.fake(
        "pip",
        'case "$*" in "install platformio") printf "#!/bin/bash\\necho 6\\n" > "$FAKE_BIN/pio"; '
        'chmod +x "$FAKE_BIN/pio";; esac',
    )
    result = sandbox.run(
        source("install/install_platformio.sh") + "install_platformio\n", FAKE_BIN=str(sandbox.bin)
    )
    assert result.returncode == 0, result.stderr
    assert sandbox.calls()[:2] == ["pip install intelhex", "pip install platformio"]


def test_tensorrt_dist_packages_lookup_handles_both_ubuntu_layouts(tmp_path: Path) -> None:
    def lookup(root: Path, minor: str) -> str:
        script = source("install/install_python_environment.sh") + (
            f'find_system_tensorrt_dist_packages 3 {minor} "{root}"\n'
        )
        return subprocess.run([BASH, "-c", script], capture_output=True, text=True).stdout.strip()

    noble = tmp_path / "noble"
    (noble / "usr/lib/python3/dist-packages/tensorrt").mkdir(parents=True)
    jammy = tmp_path / "jammy"
    (jammy / "usr/lib/python3.10/dist-packages/tensorrt").mkdir(parents=True)
    (jammy / "usr/lib/python3/dist-packages").mkdir(parents=True)
    empty = tmp_path / "empty"
    empty.mkdir()
    assert lookup(noble, "12") == "/usr/lib/python3/dist-packages"
    assert lookup(jammy, "10") == "/usr/lib/python3.10/dist-packages"
    assert lookup(empty, "12") == ""


def test_project_install_on_jetson_constrains_torch_to_the_installed_wheel(
    sandbox: Sandbox,
) -> None:
    sandbox.fake(
        "pip",
        'case "$1" in show) echo "Name: torch"; echo "Version: 2.5.0a0+nv24.08";; '
        'install) echo "constraint=$PIP_CONSTRAINT:$(cat "$PIP_CONSTRAINT")" >> "$FAKE_LOG";; esac',
    )
    script = source("install/install_python_environment.sh") + (
        'pip_install_project_keeping_torch /proj\necho "after=${PIP_CONSTRAINT:-unset}"\n'
    )
    result = sandbox.run(script)
    assert result.returncode == 0, result.stderr
    constraint = next(c for c in sandbox.calls() if c.startswith("constraint="))
    path, content = constraint.removeprefix("constraint=").split(":", 1)
    assert content == "torch==2.5.0a0+nv24.08"
    assert "after=unset" in result.stdout
    assert not Path(path).exists()


def test_jetson_wheel_install_pins_numpy_1_before_the_torch_wheel(sandbox: Sandbox) -> None:
    sandbox.fake("pip")
    script = source("install/install_pytorch_jetson.sh") + (
        "TORCH_INSTALL=https://example.invalid/torch.whl\ninstall_jetson_torch_wheel\n"
    )
    assert sandbox.run(script).returncode == 0
    installs = [c for c in sandbox.calls() if c.startswith("pip install")]
    assert installs == [
        "pip install numpy==1.26.1",
        "pip install --no-cache-dir https://example.invalid/torch.whl",
    ]


def test_cusparselt_download_is_the_linux_aarch64_0_6_3_2_archive(sandbox: Sandbox) -> None:
    sandbox.fake("sudo")
    sandbox.fake("curl")
    sandbox.fake("tar")
    sandbox.fake("ldconfig")
    result = sandbox.run(
        source("install/install_pytorch_jetson.sh") + "install_cusparselt_linux_aarch64\n"
    )
    assert result.returncode == 0, result.stderr
    curl = next(c for c in sandbox.calls() if c.startswith("curl"))
    assert (
        "/libcusparse_lt/linux-aarch64/libcusparse_lt-linux-aarch64-0.6.3.2-archive.tar.xz" in curl
    )
    assert "sbsa" not in curl


def test_udev_rules_tell_modemmanager_to_ignore_the_transmitter(
    sandbox: Sandbox, tmp_path: Path
) -> None:
    rules = tmp_path / "99-auto-battlebot.rules"
    result = sandbox.run(
        source("install/install_udev_rules.sh") + f'RULES_FILE="{rules}"\nwrite_udev_rules\n'
    )
    assert result.returncode == 0, result.stderr
    statements = rules.read_text().replace("\\\n", " ").splitlines()
    transmitter = [s for s in statements if 'idVendor}=="0483"' in s and 'idProduct}=="5740"' in s]
    assert any('ENV{ID_MM_DEVICE_IGNORE}="1"' in s for s in transmitter)


def install_with_sandboxed_paths(sandbox: Sandbox) -> Path:
    fs = sandbox.root / "fs"
    sandbox.fake(
        "sudo",
        'args=(); for a in "$@"; do case "$a" in /etc/*|/usr/*) a="$SANDBOX_FS$a";; esac; '
        'args+=("$a"); done; mkdir -p "$SANDBOX_FS/usr/local/sbin"; '
        'mkdir -p "$SANDBOX_FS/etc/systemd/system"; '
        'exec "${args[@]}"',
    )
    sandbox.fake("systemctl")
    script = source("install/install_uvcvideo_rt.sh") + "install_uvcvideo_rt\n"
    assert sandbox.run(script, SANDBOX_FS=str(fs)).returncode == 0
    return fs / "usr/local/sbin/auto-battlebot-uvcvideo-rt"


def test_uvcvideo_keeper_raises_only_timeshared_uvcvideo_kworkers_to_fifo_50(
    sandbox: Sandbox,
) -> None:
    keeper = install_with_sandboxed_paths(sandbox)
    assert os.access(keeper, os.X_OK)
    sandbox.log.write_text("")
    sandbox.fake(
        "ps",
        "printf '  101 TS kworker/3:1-uvcvideo\\n  102 TS kworker/3:2-events\\n"
        "  103 FF kworker/3:3-uvcvideo\\n  104 TS bash\\n'",
    )
    sandbox.fake("chrt")
    sandbox.fake("sleep", 'kill "$PPID"')
    sandbox.run(f"{BASH} {keeper}")
    assert [c for c in sandbox.calls() if c.startswith("chrt")] == ["chrt -f -p 50 101"]


def test_uvcvideo_service_restarts_the_keeper_forever(sandbox: Sandbox) -> None:
    install_with_sandboxed_paths(sandbox)
    unit = (sandbox.root / "fs/etc/systemd/system/uvcvideo-rt.service").read_text()
    assert "ExecStart=/usr/local/sbin/auto-battlebot-uvcvideo-rt" in unit
    assert "Restart=always" in unit
    assert "systemctl enable uvcvideo-rt.service" in "\n".join(sandbox.calls())


def build_web_tree(sandbox: Sandbox) -> Path:
    tree = sandbox.root / "repo"
    (tree / "scripts").mkdir(parents=True)
    (tree / "web").mkdir()
    shutil.copy(ROOT / "scripts/build_web.sh", tree / "scripts/build_web.sh")
    (tree / "web/package-lock.json").write_text("{}")
    sandbox.fake("node")
    sandbox.fake(
        "npm",
        'case "$1" in ci) rm -rf node_modules; mkdir node_modules; '
        "touch -d 2000-01-01 node_modules;; esac",
    )
    return tree


def test_build_web_reinstalls_only_when_the_lockfile_changes(sandbox: Sandbox) -> None:
    tree = build_web_tree(sandbox)
    script = f"{BASH} {tree / 'scripts/build_web.sh'}"
    for _ in range(3):
        assert sandbox.run(script).returncode == 0
    assert sandbox.calls().count("npm ci --no-audit --no-fund") == 1
    (tree / "web/package-lock.json").write_text('{"changed": true}')
    assert sandbox.run(script).returncode == 0
    assert sandbox.calls().count("npm ci --no-audit --no-fund") == 2
