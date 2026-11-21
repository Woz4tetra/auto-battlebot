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


def test_opponent_probability_ignores_unselected_channels(interpret) -> None:
    import torch

    logits = torch.full((1, 2, 3), -10.0)
    logits[0, 1, 1] = 10.0
    only_opponent = interpret.opponent_probability(logits, (0,))
    both = interpret.opponent_probability(logits, (0, 1))
    assert float(only_opponent.max()) < 0.01
    assert float(both[0, 1]) > 0.99


def test_flatten_branch_maps_matches_anchor_order_and_keeps_gradient(interpret) -> None:
    import torch

    maps = [torch.randn(1, 2, n, n, requires_grad=True) for n in (4, 2, 1)]
    flat = interpret.flatten_branch_maps(maps, (0,))
    assert flat.shape == (16 + 4 + 1,)
    expected = torch.sigmoid(maps[1][0, 0]).reshape(-1)
    assert torch.allclose(flat[16:20], expected)
    flat.sum().backward()
    assert all(m.grad is not None for m in maps)


def test_one2one_scores_do_not_reach_the_neck_but_cv3_maps_do() -> None:
    import torch
    from ultralytics import YOLO

    net = YOLO("yolo26n.yaml").model.eval()
    seq = net.model
    reached: list[bool] = []
    seq[16].register_full_backward_hook(lambda m, gi, go: reached.append(True))
    cls_maps: list[torch.Tensor] = []
    for branch in seq[-1].cv3:
        branch.register_forward_hook(lambda m, i, o: cls_maps.append(o))
    x = torch.rand(1, 3, 160, 160, requires_grad=True)
    _, extra = net(x)
    extra["one2one"]["scores"].amax().backward(retain_graph=True)
    assert not reached
    torch.cat([m.flatten() for m in cls_maps]).amax().backward()
    assert reached


@pytest.mark.skipif(not __import__("torch").cuda.is_available(), reason="needs a CUDA device")
def test_gradcam_returns_a_normalised_map_through_the_cv3_hooks(interpret) -> None:
    import numpy as np

    model = interpret.Model("yolo26n.yaml")
    img = np.random.default_rng(0).integers(0, 255, (640, 640, 3), dtype=np.uint8)
    cam = model.gradcam(img, [100.0, 100.0, 300.0, 300.0])
    assert cam.shape == (640, 640)
    assert np.isfinite(cam).all()
    assert 0.0 <= cam.min() and cam.max() <= 1.0 + 1e-4
