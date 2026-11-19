"""interpret_context_vs_appearance donor frame choice for the context swap."""

from __future__ import annotations

import importlib.machinery
import sys
from types import ModuleType

import pytest


@pytest.fixture(scope="module")
def interpret(load_training):
    added = []
    for name in (
        "sklearn",
        "sklearn.linear_model",
        "sklearn.model_selection",
        "sklearn.preprocessing",
    ):
        if name not in sys.modules:
            stub = ModuleType(name)
            stub.__spec__ = importlib.machinery.ModuleSpec(name, None)
            stub.LogisticRegression = stub.cross_val_score = stub.StandardScaler = object
            sys.modules[name] = stub
            added.append(name)
    yield load_training("training/model_eval/interpret_context_vs_appearance.py")
    for name in added:
        del sys.modules[name]


def _frames(boxes: list[list[list[float]]]) -> list[dict]:
    return [{"boxes": b} for b in boxes]


def test_donor_is_never_the_frame_itself(interpret) -> None:
    frames = _frames([[[0, 0, 10, 10]]] * 6)
    for fi in range(6):
        assert interpret.donor_index(frames, fi, [0, 0, 10, 10]) != fi


def test_donor_with_robot_under_the_rect_is_skipped(interpret) -> None:
    frames = _frames([[[0, 0, 10, 10]], [[0, 0, 10, 10]], [[100, 100, 110, 110]], [[0, 0, 10, 10]]])
    assert interpret.donor_index(frames, 0, [0, 0, 10, 10]) == 2


def test_search_starts_half_the_list_away(interpret) -> None:
    frames = _frames([[[100, 100, 110, 110]]] * 8)
    assert interpret.donor_index(frames, 0, [0, 0, 10, 10]) == 4


def test_opp_channels_for_2class_excludes_house_bot(interpret, monkeypatch) -> None:
    import torch

    house_bot_only = torch.full((1, 5, 8), -10.0)
    house_bot_only[:, 1, :] = 10.0

    class FakeNet:
        def __call__(self, batch):
            return None, {"one2one": {"scores": house_bot_only}}

    model = object.__new__(interpret.Model)
    model.net = FakeNet()
    model._pre = lambda imgs: imgs
    monkeypatch.setattr(interpret, "OPP_CHANNELS", (0,))
    assert interpret.Model.opp_scores(model, [None]).max() < 0.01
    monkeypatch.setattr(interpret, "OPP_CHANNELS", (0, 1))
    assert interpret.Model.opp_scores(model, [None]).min() > 0.99
