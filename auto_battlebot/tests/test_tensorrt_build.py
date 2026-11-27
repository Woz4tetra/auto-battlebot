from __future__ import annotations

from pathlib import Path

import pytest

from auto_battlebot import tensorrt_build

pytestmark = pytest.mark.jetson


@pytest.fixture(autouse=True)
def fixed_platform(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(tensorrt_build.platform, "machine", lambda: "aarch64")
    monkeypatch.setattr(tensorrt_build, "get_compute_capability", lambda: (8, 7))


def test_engine_name_tags() -> None:
    assert (
        tensorrt_build.engine_path_with_platform_tag(Path("m/yolo.engine")).name
        == "yolo_aarch64_sm87.engine"
    )
    assert (
        tensorrt_build.engine_path_with_platform_tag(Path("m/yolo.engine"), "int8").name
        == "yolo_int8_aarch64_sm87.engine"
    )
    assert tensorrt_build.engine_path_with_platform_tag(Path("yolo")).name == (
        "yolo_aarch64_sm87.engine"
    )
