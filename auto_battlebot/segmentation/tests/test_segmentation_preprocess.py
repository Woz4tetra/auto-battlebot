import cv2
import numpy as np
import pytest
import torch

from auto_battlebot.segmentation.load_deeplabv3 import common_transforms
from auto_battlebot.segmentation.model_config import ModelConfig
from auto_battlebot.segmentation.preprocess import (
    DEPLOYED_UNTIL_2026_11,
    IMAGENET_MEAN,
    IMAGENET_STD,
    TRAINING,
    ChannelOrder,
    Geometry,
    Normalization,
    Preprocess,
    Resolution,
    model_input,
    output_labels,
)

CONFIG = ModelConfig(backbone="r50", image_size=48, pad_size=4, num_classes=2, decoder="v3plus")


def frame(height: int = 30, width: int = 52) -> np.ndarray:
    rng = np.random.default_rng(0)
    return rng.integers(0, 256, size=(height, width, 3), dtype=np.uint8)


def test_training_input_matches_the_training_transforms() -> None:
    bgr = frame()
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(
        rgb, (CONFIG.image_size, CONFIG.image_size), interpolation=cv2.INTER_NEAREST
    )
    expected = common_transforms(pad_size=CONFIG.pad_size)(resized)
    torch.testing.assert_close(model_input(bgr, CONFIG, TRAINING), expected, atol=1e-5, rtol=0)


def test_deployed_input_keeps_bgr_and_imagenet_stats_like_the_cpp_path() -> None:
    bgr = frame()
    pad = CONFIG.pad_size
    padded = cv2.copyMakeBorder(bgr, pad, pad, pad, pad, cv2.BORDER_REPLICATE)
    resized = cv2.resize(padded, (CONFIG.input_size, CONFIG.input_size)).astype(np.float32) / 255
    channel = 0
    expected = (resized[:, :, channel] - IMAGENET_MEAN[channel]) / IMAGENET_STD[channel]
    actual = model_input(bgr, CONFIG, DEPLOYED_UNTIL_2026_11)
    assert actual.shape == (3, CONFIG.input_size, CONFIG.input_size)
    np.testing.assert_allclose(actual[channel].numpy(), expected, atol=1e-5)


def test_training_output_at_model_resolution_is_the_cropped_label_map() -> None:
    labels = np.arange(CONFIG.input_size**2, dtype=np.int64).reshape(CONFIG.input_size, -1) % 7
    labels = labels.astype(np.uint8)
    out = output_labels(labels, (30, 52), CONFIG, TRAINING, Resolution.MODEL)
    pad = CONFIG.pad_size
    np.testing.assert_array_equal(out, labels[pad:-pad, pad:-pad])


@pytest.mark.parametrize("geometry", list(Geometry))
def test_native_output_has_the_frame_shape(geometry: Geometry) -> None:
    preprocess = Preprocess(ChannelOrder.RGB, Normalization.DATASET, geometry)
    labels = np.ones((CONFIG.input_size, CONFIG.input_size), np.uint8)
    out = output_labels(labels, (30, 52), CONFIG, preprocess, Resolution.NATIVE)
    assert out.shape == (30, 52)
    assert out.all()


def test_deployed_output_drops_the_padded_border() -> None:
    labels = np.zeros((CONFIG.input_size, CONFIG.input_size), np.uint8)
    labels[: CONFIG.pad_size, :] = 1
    out = output_labels(
        labels,
        (CONFIG.image_size, CONFIG.image_size),
        CONFIG,
        DEPLOYED_UNTIL_2026_11,
        Resolution.NATIVE,
    )
    assert not out.any()


def test_spec_round_trips_and_rejects_typos() -> None:
    assert Preprocess.parse(DEPLOYED_UNTIL_2026_11.spec) == DEPLOYED_UNTIL_2026_11
    with pytest.raises(ValueError):
        Preprocess.parse("bgr,imagenett,deployed")
    with pytest.raises(ValueError):
        Preprocess.parse("bgr,imagenet")
