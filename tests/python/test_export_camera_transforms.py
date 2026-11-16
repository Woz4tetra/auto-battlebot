"""training/model_eval/export_camera_transforms.py: field size from /field_markers."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml


@pytest.fixture(scope="module")
def exporter(load_training):
    return load_training("training/model_eval/export_camera_transforms.py")


def _entity(stamp: int, points: list[list[float]]) -> SimpleNamespace:
    return SimpleNamespace(
        namespace="field", stamp_ns=stamp, lines=[SimpleNamespace(points=points)]
    )


def test_field_size_from_border_line_strip(exporter, monkeypatch) -> None:
    corners = [[0.0, 0.0, 0.0], [4.0, 0.0, 0.0], [4.0, 3.0, 0.0], [0.0, 3.0, 0.0]]
    closed = corners + [corners[0], [9.0, 9.0, 9.0]]
    scene = SimpleNamespace(entities=[_entity(7, closed)])
    monkeypatch.setattr(exporter, "decode_scene_update", lambda data: scene)
    assert exporter._field_sizes(b"") == [(7, 4.0, 3.0)]


def test_field_size_uses_latest_markers(exporter, monkeypatch, tmp_path: Path) -> None:
    sub = tmp_path / "rec"
    (sub / "images").mkdir(parents=True)
    (sub / "data.yaml").write_text(yaml.safe_dump({"source_mcap": "x.mcap"}))
    stamp = 5_000
    (sub / "images" / f"{stamp}.png").write_bytes(b"x")
    identity = np.eye(4)
    recording = {
        "image_stamps": np.array([stamp], dtype=np.int64),
        "camera_infos": [
            {k: 0 for k in ("width", "height", "distortion_model", "D", "K", "R", "P")}
        ],
        "dynamic": [(stamp, identity)],
        "field_from_cameraworld": [(4_000, identity), (4_900, identity)],
        "field_sizes": [(1_000, 4.0, 3.0), (6_000, 9.0, 9.0)],
    }
    monkeypatch.setattr(exporter, "read_recording", lambda path: recording)
    exporter.export_sub_dataset(sub, dry_run=False)
    record = json.loads((sub / "camera_transforms" / f"{stamp}.json").read_text())
    assert record["field_size_m"] == {"x": 4.0, "y": 3.0}
