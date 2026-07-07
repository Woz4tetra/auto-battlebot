#!/usr/bin/env python3
"""YOLO label editor for correcting model pre-labels into ground truth.

Like training/yolo/validate_yolo_dataset.py, but edits annotations instead of
pass/fail marking: move, resize, add, delete, and reclass boxes, and drag pose
keypoints. Detect rows (class cx cy w h) and pose rows (class cx cy w h kx ky v ...)
are editable; any other rows (seg) are preserved verbatim.

Usage:
    python training/model_eval/edit_labels.py data/eval/<svo_name>

Editing is click-move-click (no dragging needed): one click grabs a handle, move
the mouse to position it, and a second click drops it. A press-move-release drag
still finishes in one gesture, so both styles work.

Controls:
    click empty space          start a new box (current class; inherits keypoint
                               count from existing pose boxes, centered); move to
                               size it, click to place
    click a box body           pick it up; move to reposition, click to drop
    click a corner handle      grab it; move to resize, click to release
    click a keypoint circle    pick it up; move to reposition, click to drop
    1-9, 0                     set class of selected box / current draw class
    Delete or x                delete selected box
    a/d or left/right          previous / next image (auto-saves)
    space                      mark reviewed + jump to next unreviewed
    n                          jump to next unreviewed (without marking)
    +/- or ctrl+wheel          zoom in / out at the cursor; 0 (numpad) resets to fit
    ctrl+s                     save
"""

from __future__ import annotations

import argparse
import json
import tkinter as tk
from dataclasses import dataclass, field
from pathlib import Path
from tkinter import ttk

import yaml
from natsort import natsorted
from PIL import Image, ImageTk

HANDLE_PX = 8
MIN_BOX_PX = 4
DRAG_THRESHOLD_PX = 4  # press-to-release travel above this counts as a drag, not a click


@dataclass
class Box:
    """One editable detect/pose annotation, in image-pixel coordinates.

    keypoints holds [x_px, y_px, visibility] triplets (empty for detect rows)."""

    class_id: int
    x1: float
    y1: float
    x2: float
    y2: float
    keypoints: list[list[float]] = field(default_factory=list)

    def normalized_row(self, img_w: int, img_h: int) -> str:
        cx = (self.x1 + self.x2) / 2.0 / img_w
        cy = (self.y1 + self.y2) / 2.0 / img_h
        w = abs(self.x2 - self.x1) / img_w
        h = abs(self.y2 - self.y1) / img_h
        row = f"{self.class_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}"
        for kx, ky, visibility in self.keypoints:
            row += f" {kx / img_w:.6f} {ky / img_h:.6f} {visibility:g}"
        return row


def _parse_row(parts: list[str], img_w: int, img_h: int) -> Box | None:
    """Parse one detect (5 values) or pose (5 + 3k values) row; None if neither."""
    if len(parts) < 5 or (len(parts) - 5) % 3 != 0:
        return None
    try:
        class_id = int(float(parts[0]))
        values = [float(v) for v in parts[1:]]
    except ValueError:
        return None
    cx, cy, w, h = values[:4]
    keypoints = [
        [values[i] * img_w, values[i + 1] * img_h, values[i + 2]] for i in range(4, len(values), 3)
    ]
    return Box(
        class_id=class_id,
        x1=(cx - w / 2) * img_w,
        y1=(cy - h / 2) * img_h,
        x2=(cx + w / 2) * img_w,
        y2=(cy + h / 2) * img_h,
        keypoints=keypoints,
    )


def parse_label_file(label_path: Path, img_w: int, img_h: int) -> tuple[list[Box], list[str]]:
    """Split a YOLO label file into editable detect/pose boxes and preserved other rows."""
    boxes: list[Box] = []
    preserved: list[str] = []
    if not label_path.exists():
        return boxes, preserved
    for line in label_path.read_text().splitlines():
        parts = line.split()
        if not parts:
            continue
        box = _parse_row(parts, img_w, img_h)
        if box is not None:
            boxes.append(box)
        elif line.strip():
            preserved.append(line)
    return boxes, preserved


def _find_data_yaml(dataset_path: Path) -> Path | None:
    """The dataset's data.yaml, or one from an immediate subdataset when dataset_path is a
    parent holding per-recording subdirs (opening the parent labels them all in one pass)."""
    direct = dataset_path / "data.yaml"
    if direct.exists():
        return direct
    if dataset_path.is_dir():
        for child in sorted(dataset_path.iterdir()):
            candidate = child / "data.yaml"
            if candidate.exists():
                return candidate
    return None


def load_class_info(dataset_path: Path) -> tuple[list[str], list[str]]:
    """Read names and colors from the dataset's data.yaml."""
    fallback_colors = [
        "#FF0000",
        "#00FF00",
        "#0000FF",
        "#FFFF00",
        "#FF00FF",
        "#00FFFF",
        "#FFA500",
        "#800080",
        "#008000",
        "#FFC0CB",
    ]
    data_yaml = _find_data_yaml(dataset_path)
    if data_yaml is None:
        return [], fallback_colors
    data = yaml.safe_load(data_yaml.read_text()) or {}
    names = list(data.get("names", []))
    colors = list(data.get("colors", []))
    while len(colors) < len(names):
        colors.append(fallback_colors[len(colors) % len(fallback_colors)])
    return names, colors or fallback_colors


def load_kpt_count(dataset_path: Path) -> int:
    """Keypoints per box from data.yaml kpt_shape (0 if absent).

    Lets a freshly drawn box get keypoints even when the dataset has no pose boxes yet,
    which is the case when labeling a pose dataset from scratch."""
    data_yaml = _find_data_yaml(dataset_path)
    if data_yaml is None:
        return 0
    data = yaml.safe_load(data_yaml.read_text()) or {}
    shape = data.get("kpt_shape")
    if isinstance(shape, (list, tuple)) and shape:
        return int(shape[0])
    return 0


def _label_path_for(img_path: Path) -> Path:
    """Label file for an image: swap the nearest images/ path component for labels/.

    Works for a single dataset (dataset/images/x.png -> dataset/labels/x.txt) and for a
    parent of subdatasets (parent/rec/images/x.png -> parent/rec/labels/x.txt), so labels
    always land in the labels/ dir score.py reads, not next to the image."""
    parts = img_path.parts
    for i in range(len(parts) - 1, -1, -1):
        if parts[i] == "images":
            return Path(*parts[:i], "labels", *parts[i + 1 :]).with_suffix(".txt")
    return img_path.with_suffix(".txt")


def find_image_label_pairs(dataset_path: Path) -> list[tuple[Path, Path]]:
    """Pair each image with its labels/<stem>.txt, creating label paths as needed.

    Searches recursively, so pointing at a parent of per-recording subdatasets labels them
    all in one session."""
    image_extensions = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"}
    images_dir = dataset_path / "images"
    search_dir = images_dir if images_dir.is_dir() else dataset_path
    pairs = []
    for img_path in natsorted(
        p for p in search_dir.rglob("*") if p.suffix.lower() in image_extensions
    ):
        pairs.append((img_path, _label_path_for(img_path)))
    return pairs


class LabelEditor:
    def __init__(self, dataset_path: Path) -> None:
        self.dataset_path = dataset_path
        self.pairs = find_image_label_pairs(dataset_path)
        if not self.pairs:
            raise SystemExit(f"No images found under {dataset_path}")
        self.class_names, self.class_colors = load_class_info(dataset_path)
        self.default_kpt_count = load_kpt_count(dataset_path)
        self.state_file = dataset_path / ".edit_state.json"
        self.reviewed: set[str] = set()
        self._load_state()

        self.index = 0
        self.image: Image.Image | None = None
        self.boxes: list[Box] = []
        self.preserved: list[str] = []
        self.selected: int | None = None
        self.current_class = 0
        self.dirty = False
        self.zoom = 1.0
        # Click-move-click editing state. active_mode is the currently grabbed
        # handle ('body' | 'corner:<which>' | 'kp:<k>') or None when nothing is
        # held. A grab persists across mouse-up until the next click drops it.
        self.active_mode: str | None = None
        self.drag_anchor = (0.0, 0.0)
        self.drawing_new = False
        self.press_xy = (0.0, 0.0)  # canvas coords of the last press, for click-vs-drag

        self.root = tk.Tk()
        self.root.title(f"Label Editor - {dataset_path.name}")
        self.root.geometry("1500x900")
        self.photo: ImageTk.PhotoImage | None = None
        self.canvas_image_id: int | None = None
        self.status_var = tk.StringVar()
        self._build_ui()
        first_unreviewed = self._next_unreviewed_index(0)  # resume where labeling left off
        if first_unreviewed is not None:
            self.index = first_unreviewed
        self._load_current()

    # ------------------------------------------------------------------ state

    def _load_state(self) -> None:
        if self.state_file.exists():
            data = json.loads(self.state_file.read_text())
            self.reviewed = set(data.get("reviewed", []))

    def _save_state(self) -> None:
        self.state_file.write_text(json.dumps({"reviewed": sorted(self.reviewed)}, indent=2))

    def _image_key(self) -> str:
        return str(self.pairs[self.index][0].relative_to(self.dataset_path))

    # ------------------------------------------------------------------ ui

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True)

        canvas_frame = ttk.Frame(main)
        canvas_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=8, pady=8)
        self.canvas = tk.Canvas(canvas_frame, background="#111111", highlightthickness=0)
        x_scroll = ttk.Scrollbar(canvas_frame, orient=tk.HORIZONTAL, command=self.canvas.xview)
        y_scroll = ttk.Scrollbar(canvas_frame, orient=tk.VERTICAL, command=self.canvas.yview)
        self.canvas.configure(xscrollcommand=x_scroll.set, yscrollcommand=y_scroll.set)
        x_scroll.pack(side=tk.BOTTOM, fill=tk.X)
        y_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        sidebar = ttk.Frame(main, width=260)
        sidebar.pack(side=tk.RIGHT, fill=tk.Y)
        sidebar.pack_propagate(False)

        class_section = ttk.LabelFrame(sidebar, text="Classes (key sets selection)", padding="8")
        class_section.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)
        self.class_list = tk.Listbox(class_section, exportselection=False)
        for i, name in enumerate(self.class_names or [f"class_{i}" for i in range(10)]):
            key = (i + 1) % 10
            self.class_list.insert(tk.END, f"[{key}] {name}")
            self.class_list.itemconfig(i, foreground=self._color(i))
        self.class_list.bind("<<ListboxSelect>>", self._on_class_pick)
        self.class_list.pack(fill=tk.BOTH, expand=True)
        self.class_list.selection_set(0)

        nav = ttk.LabelFrame(sidebar, text="Navigation", padding="8")
        nav.pack(fill=tk.X, padx=8, pady=4)
        row = ttk.Frame(nav)
        row.pack(fill=tk.X)
        ttk.Button(row, text="< (a)", command=self.prev_image).pack(
            side=tk.LEFT, expand=True, fill=tk.X
        )
        ttk.Button(row, text="(d) >", command=self.next_image).pack(
            side=tk.LEFT, expand=True, fill=tk.X
        )
        ttk.Button(nav, text="Reviewed + next (space)", command=self.mark_reviewed_next).pack(
            fill=tk.X, pady=4
        )
        ttk.Button(nav, text="Next unreviewed (n)", command=self.jump_next_unreviewed).pack(
            fill=tk.X
        )
        ttk.Button(nav, text="Save (ctrl+s)", command=self.save).pack(fill=tk.X, pady=4)
        ttk.Button(nav, text="Delete box (x)", command=self.delete_selected).pack(fill=tk.X, pady=4)

        ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN).pack(
            side=tk.BOTTOM, fill=tk.X
        )

        self.canvas.bind("<ButtonPress-1>", self._on_press)
        self.canvas.bind("<Motion>", self._on_move)
        self.canvas.bind("<B1-Motion>", self._on_move)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.root.bind("a", lambda _e: self.prev_image())
        self.root.bind("d", lambda _e: self.next_image())
        self.root.bind("<Left>", lambda _e: self.prev_image())
        self.root.bind("<Right>", lambda _e: self.next_image())
        self.root.bind("<space>", lambda _e: self.mark_reviewed_next())
        self.root.bind("n", lambda _e: self.jump_next_unreviewed())
        self.root.bind("x", lambda _e: self.delete_selected())
        self.root.bind("<Delete>", lambda _e: self.delete_selected())
        self.root.bind("<Control-s>", lambda _e: self.save())
        for digit in range(10):
            self.root.bind(str(digit), self._on_digit)
        self.root.bind("<plus>", lambda _e: self._zoom_by(1.25))
        self.root.bind("<equal>", lambda _e: self._zoom_by(1.25))
        self.root.bind("<minus>", lambda _e: self._zoom_by(1 / 1.25))
        self.root.bind("<KP_0>", lambda _e: self._set_zoom(1.0))
        self.root.bind("<Control-Button-4>", lambda e: self._zoom_at(self.zoom * 1.1, e.x, e.y))
        self.root.bind("<Control-Button-5>", lambda e: self._zoom_at(self.zoom / 1.1, e.x, e.y))
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _color(self, class_id: int) -> str:
        return self.class_colors[class_id % len(self.class_colors)]

    def _class_name(self, class_id: int) -> str:
        if 0 <= class_id < len(self.class_names):
            return self.class_names[class_id]
        return f"class_{class_id}"

    # ------------------------------------------------------------------ io

    def _load_current(self) -> None:
        img_path, label_path = self.pairs[self.index]
        self.image = Image.open(img_path)
        self.boxes, self.preserved = parse_label_file(label_path, *self.image.size)
        self.selected = None
        self.active_mode = None
        self.drawing_new = False
        self.dirty = False
        self.zoom = 1.0  # reset to fit when moving to a new image
        self._render()
        self.canvas.xview_moveto(0.0)
        self.canvas.yview_moveto(0.0)

    def save(self) -> None:
        img_path, label_path = self.pairs[self.index]
        assert self.image is not None
        rows = [box.normalized_row(*self.image.size) for box in self.boxes]
        label_path.parent.mkdir(parents=True, exist_ok=True)
        label_path.write_text(
            "\n".join(rows + self.preserved) + "\n" if rows or self.preserved else ""
        )
        self.dirty = False
        self._update_status(f"Saved {label_path.name}")

    def _autosave(self) -> None:
        if self.dirty:
            self.save()

    def _on_close(self) -> None:
        self._autosave()
        self._save_state()
        self.root.destroy()

    # ------------------------------------------------------------------ navigation

    def prev_image(self) -> None:
        self._autosave()
        if self.index > 0:
            self.index -= 1
            self._load_current()

    def next_image(self) -> None:
        self._autosave()
        if self.index < len(self.pairs) - 1:
            self.index += 1
            self._load_current()

    def _next_unreviewed_index(self, start_offset: int) -> int | None:
        """Index of the next unreviewed image, scanning from self.index + start_offset and
        wrapping. start_offset=0 includes the current image; 1 skips it."""
        n = len(self.pairs)
        for offset in range(start_offset, start_offset + n):
            i = (self.index + offset) % n
            if str(self.pairs[i][0].relative_to(self.dataset_path)) not in self.reviewed:
                return i
        return None

    def jump_next_unreviewed(self) -> None:
        self._autosave()
        target = self._next_unreviewed_index(1)
        if target is None:
            self._update_status("All images reviewed")
            return
        self.index = target
        self._load_current()

    def mark_reviewed_next(self) -> None:
        self._autosave()
        self.reviewed.add(self._image_key())
        self._save_state()
        target = self._next_unreviewed_index(1)
        if target is None:
            self._update_status("All images reviewed")
            return
        self.index = target
        self._load_current()

    # ------------------------------------------------------------------ zoom + transforms

    def _fit_scale(self) -> float:
        assert self.image is not None
        cw = max(1, self.canvas.winfo_width())
        ch = max(1, self.canvas.winfo_height())
        return min(cw / self.image.width, ch / self.image.height)

    def _scale(self) -> float:
        return self._fit_scale() * self.zoom

    def _set_zoom(self, zoom: float) -> None:
        self.zoom = max(0.25, min(8.0, zoom))
        self._render()

    def _zoom_at(self, zoom: float, cx: float, cy: float) -> None:
        """Zoom so the image point under canvas widget coords (cx, cy) stays put."""
        if self.image is None:
            return
        s_old = self._scale()
        ix = self.canvas.canvasx(cx) / s_old
        iy = self.canvas.canvasy(cy) / s_old
        self.zoom = max(0.25, min(8.0, zoom))
        self._render()
        s_new = self._scale()
        w = max(1, int(self.image.width * s_new))
        h = max(1, int(self.image.height * s_new))
        self.canvas.xview_moveto(max(0.0, min(1.0, (ix * s_new - cx) / w)))
        self.canvas.yview_moveto(max(0.0, min(1.0, (iy * s_new - cy) / h)))

    def _pointer_on_canvas(self) -> tuple[float, float]:
        """Mouse position in canvas widget coords, or the canvas center if off-canvas."""
        px = self.canvas.winfo_pointerx() - self.canvas.winfo_rootx()
        py = self.canvas.winfo_pointery() - self.canvas.winfo_rooty()
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        if 0 <= px <= cw and 0 <= py <= ch:
            return float(px), float(py)
        return cw / 2, ch / 2

    def _zoom_by(self, factor: float) -> None:
        """Keyboard zoom, centered on the mouse (or the canvas center if off-canvas)."""
        cx, cy = self._pointer_on_canvas()
        self._zoom_at(self.zoom * factor, cx, cy)

    def _to_image(self, cx: float, cy: float) -> tuple[float, float]:
        s = self._scale()
        return self.canvas.canvasx(cx) / s, self.canvas.canvasy(cy) / s

    # ------------------------------------------------------------------ mouse editing

    def _hit_test(self, ix: float, iy: float) -> tuple[str, int] | None:
        """Return ('kp:<k>'|'corner:<which>'|'body', box index) for the hit box, if any."""
        handle = HANDLE_PX / self._scale()
        best: tuple[float, tuple[str, int]] | None = None
        for i, box in enumerate(self.boxes):
            for k, (kx, ky, _v) in enumerate(box.keypoints):
                if abs(ix - kx) <= handle and abs(iy - ky) <= handle:
                    return (f"kp:{k}", i)
            corners = {
                "nw": (box.x1, box.y1),
                "ne": (box.x2, box.y1),
                "sw": (box.x1, box.y2),
                "se": (box.x2, box.y2),
            }
            for which, (px, py) in corners.items():
                if abs(ix - px) <= handle and abs(iy - py) <= handle:
                    return (f"corner:{which}", i)
            if box.x1 <= ix <= box.x2 and box.y1 <= iy <= box.y2:
                area = (box.x2 - box.x1) * (box.y2 - box.y1)
                if best is None or area < best[0]:
                    best = (area, ("body", i))
        return best[1] if best else None

    def _on_press(self, event: tk.Event) -> None:
        """A click grabs a handle when nothing is held, or drops it when one is."""
        self.press_xy = (event.x, event.y)
        if self.active_mode is not None:
            # Something is already grabbed: this click drops it in place.
            self._commit_active()
            return
        ix, iy = self._to_image(event.x, event.y)
        hit = self._hit_test(ix, iy)
        if hit is None:
            # Start a new box; it grows with the mouse until the next click.
            self.boxes.append(Box(self.current_class, ix, iy, ix, iy))
            self.selected = len(self.boxes) - 1
            self.active_mode = "corner:se"
            self.drag_anchor = (ix, iy)
            self.drawing_new = True
        else:
            self.active_mode, self.selected = hit
            box = self.boxes[self.selected]
            self.drag_anchor = (ix - box.x1, iy - box.y1)
            self.drawing_new = False
        self._render()

    def _on_move(self, event: tk.Event) -> None:
        """Position the grabbed handle to follow the mouse (button up or down)."""
        if self.active_mode is None or self.selected is None:
            return
        ix, iy = self._to_image(event.x, event.y)
        box = self.boxes[self.selected]
        if self.active_mode.startswith("kp:"):
            kp = box.keypoints[int(self.active_mode.split(":")[1])]
            kp[0] = ix
            kp[1] = iy
        elif self.active_mode == "body":
            w = box.x2 - box.x1
            h = box.y2 - box.y1
            dx = ix - self.drag_anchor[0] - box.x1
            dy = iy - self.drag_anchor[1] - box.y1
            box.x1 += dx
            box.y1 += dy
            box.x2 = box.x1 + w
            box.y2 = box.y1 + h
            for kp in box.keypoints:
                kp[0] += dx
                kp[1] += dy
        else:
            which = self.active_mode.split(":")[1]
            if "n" in which:
                box.y1 = iy
            if "s" in which:
                box.y2 = iy
            if "w" in which:
                box.x1 = ix
            if "e" in which:
                box.x2 = ix
        self.dirty = True
        self._render()

    def _on_release(self, event: tk.Event) -> None:
        """End the interaction if this was a drag; a stationary click keeps the
        grab so the mouse can be moved freely before the next click drops it."""
        if self.active_mode is None:
            return
        moved = (
            abs(event.x - self.press_xy[0]) > DRAG_THRESHOLD_PX
            or abs(event.y - self.press_xy[1]) > DRAG_THRESHOLD_PX
        )
        if moved:
            self._commit_active()

    def _commit_active(self) -> None:
        """Release the grabbed handle: normalize the box, drop a degenerate new
        box, and seed keypoints on a freshly drawn one."""
        if self.active_mode is not None and self.selected is not None:
            box = self.boxes[self.selected]
            # Normalize corners and discard degenerate click-drawn boxes.
            box.x1, box.x2 = sorted((box.x1, box.x2))
            box.y1, box.y2 = sorted((box.y1, box.y2))
            if box.x2 - box.x1 < MIN_BOX_PX or box.y2 - box.y1 < MIN_BOX_PX:
                if self.drawing_new:
                    self.boxes.pop(self.selected)
                    self.selected = None
            elif self.drawing_new:
                self._init_new_box_keypoints(box)
        self.active_mode = None
        self.drawing_new = False
        self._render()

    def _init_new_box_keypoints(self, box: Box) -> None:
        """Pose datasets: give a freshly drawn box the same keypoint count as its peers,
        spread along the box's vertical midline, ready to drag into place."""
        count = max((len(b.keypoints) for b in self.boxes if b is not box), default=0)
        if count == 0:  # no pose peers yet (e.g. labeling from scratch): use the schema
            count = self.default_kpt_count
        if count == 0 or box.keypoints:
            return
        cx = (box.x1 + box.x2) / 2
        for k in range(count):
            ky = box.y1 + (box.y2 - box.y1) * (k + 1) / (count + 1)
            box.keypoints.append([cx, ky, 2.0])

    # ------------------------------------------------------------------ classes

    def _on_digit(self, event: tk.Event) -> None:
        class_id = (int(event.char) - 1) % 10
        self._set_class(class_id)

    def _on_class_pick(self, _event: tk.Event) -> None:
        selection = self.class_list.curselection()
        if selection:
            self._set_class(int(selection[0]))

    def _set_class(self, class_id: int) -> None:
        self.current_class = class_id
        self.class_list.selection_clear(0, tk.END)
        if class_id < self.class_list.size():
            self.class_list.selection_set(class_id)
        if self.selected is not None:
            self.boxes[self.selected].class_id = class_id
            self.dirty = True
        self._render()

    def delete_selected(self) -> None:
        if self.selected is not None:
            self.boxes.pop(self.selected)
            self.selected = None
            self.active_mode = None
            self.drawing_new = False
            self.dirty = True
            self._render()

    # ------------------------------------------------------------------ rendering

    def _render(self) -> None:
        if self.image is None:
            return
        s = self._scale()
        w = max(1, int(self.image.width * s))
        h = max(1, int(self.image.height * s))
        resized = self.image.resize((w, h), Image.Resampling.BILINEAR)
        self.photo = ImageTk.PhotoImage(resized)
        if self.canvas_image_id is None:
            self.canvas_image_id = self.canvas.create_image(0, 0, anchor=tk.NW, image=self.photo)
        else:
            self.canvas.itemconfig(self.canvas_image_id, image=self.photo)
        self.canvas.configure(scrollregion=(0, 0, w, h))

        self.canvas.delete("box")
        for i, box in enumerate(self.boxes):
            color = self._color(box.class_id)
            width = 3 if i == self.selected else 2
            self.canvas.create_rectangle(
                box.x1 * s,
                box.y1 * s,
                box.x2 * s,
                box.y2 * s,
                outline=color,
                width=width,
                tags="box",
            )
            self.canvas.create_text(
                box.x1 * s + 4,
                box.y1 * s - 10,
                text=self._class_name(box.class_id),
                fill=color,
                anchor=tk.W,
                tags="box",
            )
            for k, (kx, ky, _v) in enumerate(box.keypoints):
                # Keypoint order follows the model's kpt layout (0=front, 1=back).
                self.canvas.create_oval(
                    kx * s - 5,
                    ky * s - 5,
                    kx * s + 5,
                    ky * s + 5,
                    outline=color,
                    width=2,
                    tags="box",
                )
                self.canvas.create_text(
                    kx * s + 7, ky * s - 7, text=str(k), fill=color, anchor=tk.W, tags="box"
                )
            if i == self.selected:
                for px, py in (
                    (box.x1, box.y1),
                    (box.x2, box.y1),
                    (box.x1, box.y2),
                    (box.x2, box.y2),
                ):
                    self.canvas.create_rectangle(
                        px * s - 4, py * s - 4, px * s + 4, py * s + 4, fill=color, tags="box"
                    )
        self._update_status()

    def _update_status(self, note: str = "") -> None:
        img_path, _ = self.pairs[self.index]
        reviewed = "reviewed" if self._image_key() in self.reviewed else "unreviewed"
        dirty = " *" if self.dirty else ""
        self.status_var.set(
            f"{self.index + 1}/{len(self.pairs)}  {img_path.name}{dirty}  [{reviewed}]  "
            f"boxes: {len(self.boxes)}  class: {self._class_name(self.current_class)}  "
            f"reviewed: {len(self.reviewed)}/{len(self.pairs)}  {note}"
        )

    def run(self) -> None:
        self.root.after(100, self._render)
        self.root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument(
        "dataset", type=Path, help="dataset directory (images/ + labels/ + data.yaml)"
    )
    args = parser.parse_args()
    LabelEditor(args.dataset).run()


if __name__ == "__main__":
    main()
