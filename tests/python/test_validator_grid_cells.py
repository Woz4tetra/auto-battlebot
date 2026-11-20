"""Tk grid cells in the two dataset validators: photo reference, fixed geometry, minimal config."""

from __future__ import annotations

import gc
import tkinter as tk
from types import SimpleNamespace

import pytest
from PIL import Image, ImageTk

SCRIPTS = ["training/deeplab/validate_segmask_dataset.py", "training/yolo/validate_yolo_dataset.py"]
PAIRS_ATTR = {
    "training/deeplab/validate_segmask_dataset.py": "image_mask_pairs",
    "training/yolo/validate_yolo_dataset.py": "image_annotation_pairs",
}


@pytest.fixture(scope="module")
def root():
    try:
        window = tk.Tk()
    except tk.TclError:
        pytest.skip("no display for Tk")
    yield window
    window.destroy()


def _owner(module):
    return next(
        value
        for value in vars(module).values()
        if isinstance(value, type)
        and "paint_cell" in vars(value)
        and value.__module__ == module.__name__
    )


def _fake_validator(module, owner, root, script, columns: int = 2, rows: int = 1):
    from pathlib import Path

    frame = tk.Frame(root)
    frame.pack()
    thumb_px = 64
    fake = SimpleNamespace(
        grid_frame=frame,
        columns=columns,
        rows=rows,
        page_size=columns * rows,
        thumb_px=thumb_px,
        cells=[],
        selected_index=0,
        select_cell=lambda *_a, **_k: None,
    )
    for name in ("PASS_COLOR", "FAIL_COLOR", "UNSET_COLOR", "SELECT_COLOR", "IDLE_COLOR"):
        setattr(fake, name, getattr(owner, name))
    fake.thumb_box = lambda: owner.thumb_box(fake)
    setattr(fake, PAIRS_ATTR[script], [(Path(f"img_{i}.jpg"), None) for i in range(8)])
    fake.status_of = lambda index: None
    fake.photos = {}

    def photo_for(index: int):
        if index not in fake.photos:
            fake.photos[index] = ImageTk.PhotoImage(Image.new("RGB", (300, 200), "red"))
        return (index, fake.photos[index])

    fake.photo_for = photo_for
    return fake


@pytest.mark.parametrize("script", SCRIPTS)
def test_thumbnail_and_caption_geometry_is_fixed(load_training, root, script: str) -> None:
    module = load_training(script)
    owner = _owner(module)
    fake = _fake_validator(module, owner, root, script)
    owner.rebuild_grid(fake)
    width, height = fake.thumb_box()
    cell = fake.cells[0]
    long_name = "x" * 200
    cell["caption"].config(text=long_name)
    cell["image"].config(image=fake.photo_for(0)[1])
    root.update_idletasks()
    assert cell["image"].winfo_reqwidth() == width
    assert cell["image"].winfo_reqheight() == height
    assert cell["caption"].winfo_reqwidth() < 20
    assert tk.Label(root, text=long_name).winfo_reqwidth() > 100


@pytest.mark.parametrize("script", SCRIPTS)
def test_unsized_label_reflows_around_its_image(load_training, root, script: str) -> None:
    label = tk.Label(root, bd=0)
    label.config(image=ImageTk.PhotoImage(Image.new("RGB", (300, 200))))
    root.update_idletasks()
    assert (label.winfo_reqwidth(), label.winfo_reqheight()) == (300, 200)


@pytest.mark.parametrize("script", SCRIPTS)
def test_cell_keeps_its_photo_alive_after_the_cache_drops_it(
    load_training, root, script: str
) -> None:
    module = load_training(script)
    owner = _owner(module)
    fake = _fake_validator(module, owner, root, script)
    owner.rebuild_grid(fake)
    cell = fake.cells[0]
    owner.paint_cell(fake, cell, 0)
    name = str(cell["photo"])
    fake.photos.clear()
    gc.collect()
    assert name in root.tk.call("image", "names")

    loose = tk.Label(root)
    orphan = ImageTk.PhotoImage(Image.new("RGB", (8, 8)))
    orphan_name = str(orphan)
    loose.config(image=orphan)
    del orphan
    gc.collect()
    assert orphan_name not in root.tk.call("image", "names")


class _Spy:
    def __init__(self, widget) -> None:
        self.widget = widget
        self.calls = 0

    def config(self, **options) -> None:
        self.calls += 1
        self.widget.config(**options)

    def grid(self) -> None:
        self.widget.grid()


@pytest.mark.parametrize("script", SCRIPTS)
def test_repainting_an_unchanged_cell_touches_no_widget_option(
    load_training, root, script: str
) -> None:
    module = load_training(script)
    owner = _owner(module)
    fake = _fake_validator(module, owner, root, script)
    owner.rebuild_grid(fake)
    cell = fake.cells[0]
    owner.paint_cell(fake, cell, 0)
    spies = [_Spy(cell[name]) for name in ("image", "caption", "frame")]
    for name, spy in zip(("image", "caption", "frame"), spies):
        cell[name] = spy
    owner.paint_cell(fake, cell, 0)
    assert [spy.calls for spy in spies] == [0, 0, 0]
    owner.paint_cell(fake, cell, 1)
    assert [spy.calls for spy in spies] == [1, 1, 1]
