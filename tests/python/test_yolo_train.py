"""training/yolo/train.py: cache cleanup never blocks training."""

from __future__ import annotations

from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def train(load_training):
    return load_training("training/yolo/train.py")


def test_cache_cleanup_failure_does_not_abort(
    train, monkeypatch: pytest.MonkeyPatch, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    def fail(*_args: object, **_kwargs: object) -> None:
        raise OSError("disk gone")

    monkeypatch.setattr(train, "sweep", fail)
    dataset = tmp_path / "data.yaml"
    dataset.write_text("path: .\n")
    train.reclaim_stale_caches(str(dataset), 7.0)
    assert "cache cleanup skipped" in capsys.readouterr().out


def test_repeated_model_key_keeps_its_cache(train) -> None:
    config = {"batch": 64, "cache": "ram"}
    first = train.resolve_model_settings(config, 0, 0, 0, None)
    second = train.resolve_model_settings(config, 0, 0, 0, None)
    assert first == second == ("ram", {"batch": 64})
    assert config == {"batch": 64, "cache": "ram"}


def test_cache_precedence_is_flag_then_config_then_default(train) -> None:
    assert train.resolve_model_settings({"cache": "ram"}, 0, 0, 0, "disk")[0] == "disk"
    assert train.resolve_model_settings({"cache": "ram"}, 0, 0, 0, False)[0] is False
    assert train.resolve_model_settings({"cache": "ram"}, 0, 0, 0, None)[0] == "ram"
    assert train.resolve_model_settings({}, 0, 0, 0, None)[0] == train.DEFAULT_CACHE


def test_cache_never_rides_along_in_the_remaining_settings(train) -> None:
    _, settings = train.resolve_model_settings({"cache": "ram", "epochs": 100}, 0, 0, 0, "disk")
    assert "cache" not in settings


def test_positive_overrides_replace_epochs_batch_and_imgsz_only(train) -> None:
    config = {"epochs": 100, "batch": 64, "imgsz": 640}
    _, settings = train.resolve_model_settings(config, 5, 0, 960, None)
    assert settings == {"epochs": 5, "batch": 64, "imgsz": 960}


def test_dataset_directory_resolves_to_its_yaml(train, tmp_path: Path) -> None:
    (tmp_path / "data.yaml").write_text("path: .\n")
    assert train.resolve_dataset(str(tmp_path)) == str(tmp_path / "data.yaml")
    (tmp_path / "data.yml").write_text("path: .\n")
    assert train.resolve_dataset(str(tmp_path)) == str(tmp_path / "data.yml")


def test_dataset_yaml_path_is_unchanged(train, tmp_path: Path) -> None:
    yaml_path = tmp_path / "arm.yml"
    yaml_path.write_text("path: .\n")
    assert train.resolve_dataset(str(yaml_path)) == str(yaml_path)


def test_dataset_directory_without_yaml_exits_before_training(train, tmp_path: Path) -> None:
    with pytest.raises(SystemExit, match="No data.yml or data.yaml"):
        train.resolve_dataset(str(tmp_path))


def test_importing_train_forces_the_agg_backend_despite_a_display() -> None:
    import os
    import subprocess
    import sys

    repo_root = Path(__file__).resolve().parents[2]
    script = (
        "import importlib.util, sys\n"
        "spec = importlib.util.spec_from_file_location('t', sys.argv[1])\n"
        "mod = importlib.util.module_from_spec(spec)\n"
        "spec.loader.exec_module(mod)\n"
        "import matplotlib, os\n"
        "print(os.environ['MPLBACKEND'])\n"
        "print(matplotlib.get_backend())\n"
    )
    env = {key: value for key, value in os.environ.items() if key != "MPLBACKEND"}
    env["DISPLAY"] = os.environ.get("DISPLAY", ":1")
    result = subprocess.run(
        [sys.executable, "-c", script, str(repo_root / "training/yolo/train.py")],
        env=env,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    declared, resolved = result.stdout.strip().splitlines()[-2:]
    assert declared == "Agg"
    assert resolved.lower() == "agg"
