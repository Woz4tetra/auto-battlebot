import json
import shutil
import subprocess
from pathlib import Path

import pytest

PCBA = Path(__file__).resolve().parents[2] / "hardware" / "mr_stabs_mk2_pcb" / "pcba"
SKILL = (
    Path.home()
    / ".claude"
    / "plugins"
    / "cache"
    / "pcba-skill"
    / "pcba-design"
    / "0.1.0"
    / "skills"
    / "pcba-design"
)
IMAGE = "pcba-kicad:9.0"


def _have_image() -> bool:
    if shutil.which("docker") is None or not (SKILL / "scripts" / "kicad").exists():
        return False
    out = subprocess.run(["docker", "images", "-q", IMAGE], capture_output=True, text=True)
    return bool(out.stdout.strip())


pytestmark = pytest.mark.skipif(not _have_image(), reason="needs docker image pcba-kicad:9.0")


def _kicad(cwd: Path, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [str(SKILL / "scripts" / "kicad"), *args], cwd=cwd, capture_output=True, text=True
    )


@pytest.fixture(scope="module")
def workdir(tmp_path_factory) -> Path:
    work = tmp_path_factory.mktemp("pcba") / "pcba"
    shutil.copytree(
        PCBA, work, ignore=shutil.ignore_patterns("fab", "step", "sim", "*.png", "*.svg", "*.pdf")
    )
    return work


@pytest.fixture(scope="module")
def netlist(workdir: Path) -> Path:
    result = _kicad(workdir, "python3", "design.py")
    assert result.returncode == 0, result.stderr[-2000:]
    assert (workdir / "mk2.net").stat().st_size > 0
    return workdir


@pytest.fixture(scope="module")
def info(netlist: Path) -> Path:
    raw = _kicad(
        netlist,
        "kinet2pcb",
        "-i",
        "mk2.net",
        "-w",
        "-nb",
        "-l",
        f"{netlist}/lib",
        "-o",
        "mk2_raw.kicad_pcb",
    )
    assert raw.returncode == 0, raw.stderr[-2000:]
    out = _kicad(netlist, "python3", str(SKILL / "scripts" / "pcb.py"), "info", "mk2_raw.kicad_pcb")
    assert out.returncode == 0, out.stderr[-2000:]
    (netlist / "info.jsonl").write_text(out.stdout)
    return netlist


def _gen(workdir: Path) -> subprocess.CompletedProcess:
    return subprocess.run(
        [str(SKILL / "scripts" / "run"), "gen_placement.py"],
        cwd=workdir,
        capture_output=True,
        text=True,
    )


def test_design_netlist_builds_with_skidl_nc_from_builtins(netlist):
    assert "NC" in (netlist / "design.py").read_text()


def test_courtyard_overrides_are_only_nano_rx_and_boot(info):
    _gen(info)
    overrides = json.loads((info / "placement.json").read_text())["courtyard_overrides"]
    assert sorted(overrides) == sorted(
        d["ref"]
        for d in map(json.loads, (info / "info.jsonl").read_text().splitlines())
        if d["value"] in ("NANO_RX", "BOOT/RST")
    )


def test_mcu_inside_roof_keepout_fails_generation(info):
    source = (info / "gen_placement.py").read_text()
    (info / "gen_placement.py").write_text(
        source.replace("MCU_XY = (0.0, 0.0)", "MCU_XY = (0.0, 2.0)")
    )
    result = _gen(info)
    (info / "gen_placement.py").write_text(source)
    assert "AssertionError" in result.stderr
