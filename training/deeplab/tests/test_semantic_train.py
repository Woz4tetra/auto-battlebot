"""training/deeplab/semantic_train.py: validation ordering and the per-field IoU definition."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import torch
from PIL import Image
from torch.utils.data import SequentialSampler


@pytest.fixture(scope="module")
def trainer(load_training):
    return load_training("training/deeplab/semantic_train.py")


def _write_split(directory: Path, count: int) -> None:
    directory.mkdir(parents=True)
    for index in range(count):
        Image.fromarray(np.zeros((8, 8, 3), dtype=np.uint8)).save(directory / f"f{index}.jpg")
        Image.fromarray(np.zeros((8, 8), dtype=np.uint8)).save(directory / f"f{index}_mask.png")


def test_field_iou_valid_loader_unshuffled(trainer, tmp_path: Path) -> None:
    _write_split(tmp_path / "train", 3)
    _write_split(tmp_path / "val", 3)
    _, valid_loader = trainer.get_dataset(
        tmp_path, batch_size=2, num_workers=0, image_size=8, pad_size=0
    )
    assert isinstance(valid_loader.sampler, SequentialSampler)


def _one_hot(plane: np.ndarray) -> torch.Tensor:
    field = torch.from_numpy(plane.astype(np.float32))
    return torch.stack([1 - field, field])[None]


def test_field_iou_matches_score_masks(trainer) -> None:
    assert trainer.FIELD_CHANNEL == 1
    truth = np.zeros((12, 12), dtype=bool)
    truth[4:8, 4:8] = True
    prediction = truth.copy()
    prediction[0, :] = True
    prediction[:, 0] = True
    cropped = trainer.FieldIoU(["cage3"], pad_size=2)
    cropped.update(_one_hot(prediction), _one_hot(truth))
    assert cropped.summary()["iou/cage3"] == pytest.approx(1.0)
    uncropped = trainer.FieldIoU(["cage3"], pad_size=0)
    uncropped.update(_one_hot(prediction), _one_hot(truth))
    assert uncropped.summary()["iou/cage3"] < 1.0


class _DrawsRandom(torch.utils.data.Dataset):
    def __len__(self) -> int:
        return 8

    def __getitem__(self, index: int) -> float:
        import random

        return random.random()


def test_train_loader_workers_draw_distinct_python_random_streams(trainer, tmp_path: Path) -> None:
    _write_split(tmp_path / "train", 4)
    _write_split(tmp_path / "val", 2)
    train_loader, _ = trainer.get_dataset(
        tmp_path, batch_size=2, num_workers=2, image_size=8, pad_size=0
    )
    assert train_loader.worker_init_fn is None
    probe = torch.utils.data.DataLoader(
        _DrawsRandom(),
        batch_size=4,
        num_workers=train_loader.num_workers,
        worker_init_fn=train_loader.worker_init_fn,
        multiprocessing_context="fork",
    )
    draws = [value for batch in probe for value in batch.tolist()]
    assert len(set(draws)) == len(draws)
