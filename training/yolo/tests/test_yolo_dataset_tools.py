"""training/yolo dataset builders: eval label rewrites, uniform aspect view, box-only pose rows."""

from __future__ import annotations

import json
from collections import Counter
from pathlib import Path

import numpy as np
import pytest
from PIL import Image
from ultralytics.data.utils import verify_image_label


@pytest.fixture(scope="module")
def arm(load_training):
    return load_training("training/yolo/make_all_keypoints_arm.py")


@pytest.fixture(scope="module")
def subset(load_training):
    return load_training("training/yolo/make_uniform_aspect_subset.py")


@pytest.fixture(scope="module")
def boxonly(load_training):
    return load_training("training/yolo/seg_to_boxonly_pose.py")


@pytest.fixture(scope="module")
def plotter(load_training):
    return load_training("training/yolo/plot_results.py")


def _image(path: Path, width: int = 32, height: int = 32) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.zeros((height, width, 3), dtype=np.uint8)).save(path)


def _verify(image: Path, label: Path) -> int:
    result = verify_image_label((str(image), str(label), "", True, 4, 2, 3, False))
    return int(result[8])


def test_ultralytics_discards_a_frame_with_an_out_of_range_keypoint(arm, tmp_path: Path) -> None:
    image = tmp_path / "images" / "f.jpg"
    _image(image)
    label = tmp_path / "labels" / "f.txt"
    label.parent.mkdir()
    label.write_text("0 0.5 0.5 0.2 0.2 1.4 0.5 2 0.5 0.5 2\n")
    assert _verify(image, label) == 1
    fixed, count = arm.fix_row(label.read_text().strip())
    assert count == 1
    label.write_text(fixed + "\n")
    assert _verify(image, label) == 0


def _eval_set(root: Path, rows: list[str]) -> None:
    _image(root / "images" / "f.jpg")
    (root / "labels").mkdir(parents=True)
    (root / "labels" / "f.txt").write_text("".join(f"{row}\n" for row in rows))
    (root / "validation_state.json").write_text(json.dumps({"images/f.jpg": "pass"}))


def test_eval_labels_are_rewritten_beside_a_symlink_and_originals_untouched(
    arm, tmp_path: Path
) -> None:
    root = tmp_path / "eval_a"
    original = ["0 0.5 0.5 0.2 0.2 1.4 0.5 2 0.5 0.5 2", "4 0.1 0.1 0.1 0.1 0.1 0.1 2 0.2 0.2 2"]
    _eval_set(root, original)
    out = tmp_path / "out"
    dropped: Counter = Counter()
    unlabelled: Counter = Counter()
    frames = arm.eval_frames(root, out, dropped, unlabelled)
    assert len(frames) == 1
    link = frames[0]
    assert link.is_symlink()
    assert link.resolve() == (root / "images" / "f.jpg").resolve()
    rewritten = (link.parent.parent / "labels" / "f.txt").read_text().splitlines()
    assert rewritten == ["0 0.5 0.5 0.2 0.2 0.000000 0.000000 0 0.5 0.5 2"]
    assert dropped == Counter({"eval_a": 1})
    assert unlabelled == Counter({"eval_a": 1})
    assert (root / "labels" / "f.txt").read_text().splitlines() == original
    assert _verify(link, link.parent.parent / "labels" / "f.txt") == 0


def test_clean_eval_frame_is_used_in_place(arm, tmp_path: Path) -> None:
    root = tmp_path / "eval_b"
    _eval_set(root, ["0 0.5 0.5 0.2 0.2 0.4 0.5 2 0.5 0.5 2"])
    out = tmp_path / "out"
    frames = arm.eval_frames(root, out, Counter(), Counter())
    assert frames == [(root / "images" / "f.jpg").resolve()]
    assert not (out / "eval_labels").exists()


def _split(root: Path, sizes: dict[str, tuple[int, int]]) -> None:
    for name, (width, height) in sizes.items():
        _image(root / "train" / "images" / f"{name}.jpg", width, height)
        (root / "train" / "labels").mkdir(parents=True, exist_ok=True)
        (root / "train" / "labels" / f"{name}.txt").write_text("0 0.5 0.5 0.1 0.1\n")


def test_uniform_aspect_view_drops_odd_frames_so_rect_batches_share_a_shape(
    subset, tmp_path: Path
) -> None:
    src = tmp_path / "src"
    _split(src, {"wide_a": (160, 90), "wide_b": (320, 180), "odd": (192, 88)})
    kept, dropped = subset.link_split(src, tmp_path / "dst", "train", 16 / 9)
    assert (kept, dropped) == (2, 1)
    names = sorted(p.name for p in (tmp_path / "dst" / "train" / "images").iterdir())
    assert names == ["wide_a.jpg", "wide_b.jpg"]
    assert (tmp_path / "dst" / "train" / "labels" / "wide_a.txt").is_symlink()


def test_no_target_aspect_keeps_everything(subset, tmp_path: Path) -> None:
    src = tmp_path / "src"
    _split(src, {"wide": (160, 90), "odd": (192, 88)})
    assert subset.link_split(src, tmp_path / "dst", "train", None) == (2, 0)


def test_ultralytics_disables_shuffle_for_mixed_rect_batches() -> None:
    import inspect

    from ultralytics.models.yolo.detect import train

    source = inspect.getsource(train.DetectionTrainer.get_dataloader)
    assert "dataset.batch_shapes == dataset.batch_shapes[0]" in source
    assert "shuffle = False" in source


KEEP = 1
EXCLUDE = {3, 4}
SUFFIX = " 0 0 0 0 0 0"


def test_frame_with_our_robot_class_is_dropped_whole(boxonly) -> None:
    text = "1 0.2 0.2 0.4 0.2 0.4 0.4\n3 0.5 0.5 0.1 0.1 0.2 0.2\n"
    assert boxonly.convert_label(text, KEEP, EXCLUDE, 2, SUFFIX) is None


def test_opponent_rows_get_visibility_zero_keypoints(boxonly) -> None:
    text = "1 0.1 0.1 0.5 0.1 0.5 0.5 0.1 0.5\n0 0.3 0.3 0.4 0.4 0.4 0.3\n"
    rows = boxonly.convert_label(text, KEEP, EXCLUDE, 2, SUFFIX)
    assert rows == ["2 0.300000 0.300000 0.400000 0.400000 0 0 0 0 0 0"]


def test_masked_keypoints_survive_ultralytics_verification(boxonly, tmp_path: Path) -> None:
    rows = boxonly.convert_label("1 0.1 0.1 0.5 0.1 0.5 0.5\n", KEEP, EXCLUDE, 0, " 0 0 0 0 0 0")
    assert rows is not None
    image = tmp_path / "images" / "f.jpg"
    _image(image)
    label = tmp_path / "labels" / "f.txt"
    label.parent.mkdir()
    label.write_text("\n".join(rows) + "\n")
    assert _verify(image, label) == 0


def test_padded_results_csv_headers_are_stripped(plotter, tmp_path: Path) -> None:
    csv = tmp_path / "results.csv"
    csv.write_text("                  epoch,         train/box_loss\n1,0.5\n2,0.4\n")
    frame = plotter.load_csv(csv)
    assert list(frame.columns) == ["epoch", "train/box_loss"]
    assert frame["train/box_loss"].tolist() == [0.5, 0.4]
