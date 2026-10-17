"""PyTorch Dense stacks to RTNeural's JSON model format.

RTNeural's `json_parser::parseJson` reads the TensorFlow-style layout its `python/model_utils.py`
writes: `in_shape` whose last entry is the input size, then one entry per layer with `type`,
`activation` (empty for none), `shape` (last entry the output size) and `weights`. A Dense
layer's `weights` are `[kernel, bias]` with the kernel indexed `[input][output]`, the transpose of
`torch.nn.Linear.weight`.

Only `Linear` followed optionally by one activation is supported, which is all `mlp_gaussian`
uses. The input normalization is folded into the first layer: `W' = W / std` per input column and
`b' = b - W' @ mean`.
"""

from __future__ import annotations

from typing import Any

import numpy as np
from torch import nn

ACTIVATIONS: dict[type[nn.Module], str] = {
    nn.ELU: "elu",
    nn.Tanh: "tanh",
    nn.ReLU: "relu",
    nn.Sigmoid: "sigmoid",
}


def dense_layers(net: nn.Sequential) -> list[tuple[np.ndarray, np.ndarray, str]]:
    """(weight [out, in], bias [out], activation) per Dense layer."""
    layers: list[tuple[np.ndarray, np.ndarray, str]] = []
    modules = list(net)
    i = 0
    while i < len(modules):
        m = modules[i]
        if not isinstance(m, nn.Linear):
            raise ValueError(
                f"unsupported module {type(m).__name__} at {i}; only Linear + activation"
            )
        act = ""
        if i + 1 < len(modules) and type(modules[i + 1]) in ACTIVATIONS:
            act = ACTIVATIONS[type(modules[i + 1])]
            i += 1
        layers.append(
            (m.weight.detach().double().cpu().numpy(), m.bias.detach().double().cpu().numpy(), act)
        )
        i += 1
    return layers


def fold_normalization(
    layers: list[tuple[np.ndarray, np.ndarray, str]], mean: np.ndarray, std: np.ndarray
) -> list[tuple[np.ndarray, np.ndarray, str]]:
    w, b, act = layers[0]
    w2 = w / std[None, :]
    b2 = b - w2 @ mean
    return [(w2, b2, act), *layers[1:]]


def to_json(layers: list[tuple[np.ndarray, np.ndarray, str]]) -> dict[str, Any]:
    in_dim = int(layers[0][0].shape[1])
    return {
        "in_shape": [None, None, in_dim],
        "layers": [
            {
                "type": "dense",
                "activation": act,
                "shape": [None, None, int(w.shape[0])],
                "weights": [w.T.tolist(), b.tolist()],
            }
            for w, b, act in layers
        ],
    }


def run_json(model: dict[str, Any], x: np.ndarray) -> np.ndarray:
    """Evaluate an RTNeural JSON model in numpy, the way RTNeural reads it. The exporter's
    self-check: a transposed kernel or a missed activation fails here, before C++ sees it."""
    out = np.asarray(x, dtype=np.float64)
    for layer in model["layers"]:
        kernel = np.asarray(layer["weights"][0], dtype=np.float64)
        bias = np.asarray(layer["weights"][1], dtype=np.float64)
        out = out @ kernel + bias
        act = layer["activation"]
        if act == "elu":
            out = np.where(out > 0, out, np.expm1(out))
        elif act == "tanh":
            out = np.tanh(out)
        elif act == "relu":
            out = np.maximum(out, 0.0)
        elif act == "sigmoid":
            out = 1.0 / (1.0 + np.exp(-out))
        elif act:
            raise ValueError(f"unknown activation {act!r}")
    return out
