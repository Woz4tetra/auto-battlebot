"""Every analysis CLI, fixture generator, and scene pipeline imports and exposes main()."""

from __future__ import annotations

import importlib
import importlib.util
from pathlib import Path

import pytest

from auto_battlebot.script_loader import REPO_ROOT, load_script

CLI_MODULES = (
    "auto_battlebot.calibration.apriltag.analyze_apriltag_mcap",
    "auto_battlebot.calibration.apriltag.fit_plant_calib",
    "auto_battlebot.calibration.apriltag.make_print_tags",
    "auto_battlebot.calibration.apriltag.make_robot_tag_3d",
    "auto_battlebot.calibration.distill_mujoco_plant",
    "auto_battlebot.calibration.fit_mrs_buff_mk3_interim_plant",
    "auto_battlebot.calibration.fit_mujoco_plant",
    "auto_battlebot.calibration.jig.fit_encoder_scale",
    "auto_battlebot.calibration.jig.fit_gyro_noise",
    "auto_battlebot.calibration.jig.fit_gyro_scale",
    "auto_battlebot.calibration.jig.fit_jig_plant",
    "auto_battlebot.calibration.jig.plot_jig_runs",
    "auto_battlebot.calibration.jig.velocity_jig_drive",
    "auto_battlebot.calibration.match.fit_match_plant",
    "auto_battlebot.calibration.match.render_match_prediction",
    "auto_battlebot.calibration.match.render_match_prediction_camera",
    "auto_battlebot.calibration.sim_stop_test",
    "auto_battlebot.calibration.sysid.check_sysid_recordings",
    "auto_battlebot.calibration.sysid.detect_robot_yolo",
    "auto_battlebot.calibration.jig.find_flip_accel",
    "auto_battlebot.calibration.sysid.plot_sysid_bundle",
    "auto_battlebot.calibration.sysid.smooth_tag_poses",
    "auto_battlebot.calibration.sysid.sysid_coverage",
    "auto_battlebot.calibration.write_plant_configs",
    "auto_battlebot.datasets.annotate_cage_video",
    "auto_battlebot.datasets.download_cage_video",
    "auto_battlebot.eval.cache_gt_depth",
    "auto_battlebot.mujoco_sim.build_mr_stabs_mjcf",
    "auto_battlebot.mujoco_sim.build_mrs_buff_mk3_model",
    "auto_battlebot.mujoco_sim.lump_mr_stabs_mass_properties",
    "auto_battlebot.nav.repro_1025",
    "auto_battlebot.nav.step_test",
    "auto_battlebot.rl.export_checkpoint",
    "auto_battlebot.rl.fit_provisional_ekf_error",
    "auto_battlebot.rl.fit_sensing_model",
    "auto_battlebot.rl.lockstep_eval",
    "auto_battlebot.rl.score_field_session",
)
SCRIPT_GLOBS = (
    "scripts/mcap_auto_percentage.py",
    "tests/fixtures/generators/*.py",
    "training/synthetic/scenes/*/*.py",
)
OPTIONAL_MODULES = {"pyzed": "ZED SDK", "mujoco": "MuJoCo", "torch": "PyTorch"}


def script_paths() -> list[str]:
    found = {p.relative_to(REPO_ROOT).as_posix() for g in SCRIPT_GLOBS for p in REPO_ROOT.glob(g)}
    return sorted(p for p in found if "def main(" in (REPO_ROOT / p).read_text())


def module_path(name: str) -> Path:
    return REPO_ROOT / (name.replace(".", "/") + ".py")


def missing_optional(path: Path) -> str | None:
    text = path.read_text()
    for module, label in OPTIONAL_MODULES.items():
        imported = f"import {module}" in text or f"from {module}" in text
        if imported and importlib.util.find_spec(module) is None:
            return label
    return None


def test_no_python_sources_under_script_areas() -> None:
    nested = sorted(
        p.relative_to(REPO_ROOT).as_posix()
        for p in REPO_ROOT.glob("scripts/*/**/*.py")
        if "tests" not in p.relative_to(REPO_ROOT).parts
    )
    assert nested == []


def test_the_globs_find_the_generators_and_scenes() -> None:
    assert len(script_paths()) >= 10


@pytest.mark.parametrize("name", CLI_MODULES)
def test_cli_module_imports_and_has_main(name: str) -> None:
    label = missing_optional(module_path(name))
    if label is not None:
        pytest.skip(f"{label} not installed")
    module = importlib.import_module(name)
    assert callable(getattr(module, "main", None)), f"{name} has no main()"


@pytest.mark.parametrize("path", script_paths())
def test_script_imports_and_has_main(path: str) -> None:
    label = missing_optional(REPO_ROOT / path)
    if label is not None:
        pytest.skip(f"{label} not installed")
    module = load_script(path)
    assert callable(getattr(module, "main", None)), f"{path} has no main()"
    assert Path(module.__file__ or "").resolve() == (REPO_ROOT / path).resolve()
