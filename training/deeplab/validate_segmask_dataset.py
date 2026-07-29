#!/usr/bin/env python3
"""
Segmentation Mask Dataset Validation UI

This script provides a GUI for manually validating segmentation mask dataset
annotations. Images are shown as a page of thumbnails with their mask
overlays; arrow keys move the selection, y/n pass or fail it, and t/b judge
every image on screen at once. Press Z to zoom the selected frame.
The validation state is saved and can be resumed later.

Mask convention: each image foo.jpg has a corresponding foo_mask.png where
pixel values are integer class IDs (same format used by semantic_train.py).
"""

import argparse
import json
import os
import re
import threading
import tkinter as tk
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml
from natsort import natsorted
from PIL import Image, ImageDraw, ImageFont, ImageTk

NAMED_COLORS: Dict[str, List[int]] = {
    # Primary colors
    "red": [255, 0, 0],
    "green": [0, 255, 0],
    "blue": [0, 0, 255],
    # Secondary colors
    "yellow": [255, 255, 0],
    "cyan": [0, 255, 255],
    "magenta": [255, 0, 255],
    # Common colors
    "orange": [255, 165, 0],
    "pink": [255, 192, 203],
    "purple": [128, 0, 128],
    "lime": [50, 255, 50],
    "teal": [0, 128, 128],
    "navy": [0, 0, 128],
    "maroon": [128, 0, 0],
    "olive": [128, 128, 0],
    "aqua": [0, 255, 255],
    "fuchsia": [255, 0, 255],
    # Grayscale
    "white": [255, 255, 255],
    "gray": [128, 128, 128],
    "grey": [128, 128, 128],
    "black": [0, 0, 0],
    "lightgray": [211, 211, 211],
    "lightgrey": [211, 211, 211],
    "darkgray": [169, 169, 169],
    "darkgrey": [169, 169, 169],
    "dimgray": [105, 105, 105],
    "dimgrey": [105, 105, 105],
    # Light variants
    "lightred": [255, 102, 102],
    "lightblue": [173, 216, 230],
    "lightgreen": [144, 238, 144],
    "lightpink": [255, 182, 193],
    "lightyellow": [255, 255, 224],
    "lightcyan": [224, 255, 255],
    "lightorange": [255, 200, 128],
    "lightpurple": [200, 162, 200],
    "lightcoral": [240, 128, 128],
    "lightsalmon": [255, 160, 122],
    "lightseagreen": [32, 178, 170],
    "lightskyblue": [135, 206, 250],
    "lightsteelblue": [176, 196, 222],
    # Dark variants
    "darkred": [139, 0, 0],
    "darkgreen": [0, 100, 0],
    "darkblue": [0, 0, 139],
    "darkcyan": [0, 139, 139],
    "darkmagenta": [139, 0, 139],
    "darkorange": [255, 140, 0],
    "darkviolet": [148, 0, 211],
    "darkturquoise": [0, 206, 209],
    "darksalmon": [233, 150, 122],
    "darkseagreen": [143, 188, 143],
    "darkslateblue": [72, 61, 139],
    "darkslategray": [47, 79, 79],
    "darkgoldenrod": [184, 134, 11],
    "darkolivegreen": [85, 107, 47],
    "darkorchid": [153, 50, 204],
    "darkkhaki": [189, 183, 107],
    # Reds and pinks
    "crimson": [220, 20, 60],
    "firebrick": [178, 34, 34],
    "indianred": [205, 92, 92],
    "tomato": [255, 99, 71],
    "hotpink": [255, 105, 180],
    "deeppink": [255, 20, 147],
    "mediumvioletred": [199, 21, 133],
    "palevioletred": [219, 112, 147],
    "rosybrown": [188, 143, 143],
    # Oranges and yellows
    "gold": [255, 215, 0],
    "coral": [255, 127, 80],
    "salmon": [250, 128, 114],
    "peachpuff": [255, 218, 185],
    "moccasin": [255, 228, 181],
    "papayawhip": [255, 239, 213],
    "lemonchiffon": [255, 250, 205],
    "khaki": [240, 230, 140],
    "palegoldenrod": [238, 232, 170],
    "goldenrod": [218, 165, 32],
    # Greens
    "limegreen": [50, 205, 50],
    "forestgreen": [34, 139, 34],
    "seagreen": [46, 139, 87],
    "springgreen": [0, 255, 127],
    "mediumseagreen": [60, 179, 113],
    "mediumspringgreen": [0, 250, 154],
    "palegreen": [152, 251, 152],
    "greenyellow": [173, 255, 47],
    "chartreuse": [127, 255, 0],
    "lawngreen": [124, 252, 0],
    "olivedrab": [107, 142, 35],
    "yellowgreen": [154, 205, 50],
    # Blues and cyans
    "skyblue": [135, 206, 235],
    "deepskyblue": [0, 191, 255],
    "dodgerblue": [30, 144, 255],
    "cornflowerblue": [100, 149, 237],
    "steelblue": [70, 130, 180],
    "royalblue": [65, 105, 225],
    "mediumblue": [0, 0, 205],
    "midnightblue": [25, 25, 112],
    "powderblue": [176, 224, 230],
    "cadetblue": [95, 158, 160],
    "slateblue": [106, 90, 205],
    "mediumslateblue": [123, 104, 238],
    "aquamarine": [127, 255, 212],
    "mediumaquamarine": [102, 205, 170],
    "turquoise": [64, 224, 208],
    "mediumturquoise": [72, 209, 204],
    "paleturquoise": [175, 238, 238],
    # Purples and violets
    "violet": [238, 130, 238],
    "indigo": [75, 0, 130],
    "blueviolet": [138, 43, 226],
    "orchid": [218, 112, 214],
    "plum": [221, 160, 221],
    "mediumorchid": [186, 85, 211],
    "mediumpurple": [147, 112, 219],
    "thistle": [216, 191, 216],
    "lavender": [230, 230, 250],
    # Browns and neutrals
    "brown": [139, 69, 19],
    "saddlebrown": [139, 69, 19],
    "sienna": [160, 82, 45],
    "chocolate": [210, 105, 30],
    "peru": [205, 133, 63],
    "sandybrown": [244, 164, 96],
    "burlywood": [222, 184, 135],
    "tan": [210, 180, 140],
    "wheat": [245, 222, 179],
    "navajowhite": [255, 222, 173],
    "bisque": [255, 228, 196],
    "blanchedalmond": [255, 235, 205],
    "cornsilk": [255, 248, 220],
    "beige": [245, 245, 220],
    "antiquewhite": [250, 235, 215],
    "linen": [250, 240, 230],
    "oldlace": [253, 245, 230],
    "ivory": [255, 255, 240],
    "floralwhite": [255, 250, 240],
    "honeydew": [240, 255, 240],
    "mintcream": [245, 255, 250],
    "azure": [240, 255, 255],
    "aliceblue": [240, 248, 255],
    "ghostwhite": [248, 248, 255],
    "snow": [255, 250, 250],
    "seashell": [255, 245, 238],
    "mistyrose": [255, 228, 225],
    "lavenderblush": [255, 240, 245],
    "silver": [192, 192, 192],
    "gainsboro": [220, 220, 220],
    "whitesmoke": [245, 245, 245],
    "slategray": [112, 128, 144],
    "slategrey": [112, 128, 144],
}

# Default fallback colors for classes when no config is provided
DEFAULT_CLASS_COLORS = [
    [255, 0, 0],
    [0, 255, 0],
    [0, 0, 255],
    [255, 255, 0],
    [255, 0, 255],
    [0, 255, 255],
    [255, 165, 0],
    [128, 0, 128],
    [0, 128, 0],
    [255, 192, 203],
]


class SegmaskDatasetValidator:
    """Contact-sheet validator: page through a grid of thumbnails, judge with the keyboard."""

    THUMB_STEPS = [120, 160, 200, 260, 320, 400, 500, 620]

    PASS_COLOR = "#2E7D32"
    FAIL_COLOR = "#C62828"
    UNSET_COLOR = "#9E9E9E"
    SELECT_COLOR = "#1565C0"
    IDLE_COLOR = "#ECECEC"

    def __init__(self, dataset_path: Path, class_labels_path: Optional[Path]):
        self.dataset_path = dataset_path
        self.state_file = self.dataset_path / "validation_state.json"
        self.class_labels_path = class_labels_path

        # Data structures
        self.image_mask_pairs: List[Tuple[Path, Path]] = []
        self.validation_state: Dict[str, str] = {}  # path -> 'pass'/'fail'
        self.class_info: Dict[int, Dict] = {}  # class_id -> {name, color}
        self.show_overlay = True
        self.overlay_alpha = 0.45

        # Grid state
        self.thumb_step = 3
        self.selected_index = 0
        self.page_start = 0
        self.columns = 1
        self.rows = 1
        self.cells: List[Dict] = []
        self.photo_refs: Dict[int, ImageTk.PhotoImage] = {}

        # Thumbnails are decoded off the main thread and cached, so paging stays snappy
        self.thumb_cache: "OrderedDict[Tuple, Image.Image]" = OrderedDict()
        self.cache_limit = 800
        self.cache_lock = threading.Lock()
        self.executor = ThreadPoolExecutor(max_workers=min(8, (os.cpu_count() or 4)))
        self._layout_job: Optional[str] = None

        # Zoom overlay
        self.zoom_window: Optional[tk.Toplevel] = None
        self.zoom_label: Optional[tk.Label] = None
        self.zoom_photo: Optional[ImageTk.PhotoImage] = None

        # UI components
        self.root = tk.Tk()
        self.root.title("Segmentation Mask Dataset Validator")
        self.root.geometry("1600x1000")

        # Status
        self.status_var = tk.StringVar()
        self.page_info_var = tk.StringVar()
        self.selection_var = tk.StringVar()
        self.show_overlay_var = tk.BooleanVar(value=True)

        # Initialize
        self.load_dataset()
        self.load_class_info(self.class_labels_path)
        self.load_state()
        self.setup_ui()
        self.jump_to_next_unvalidated()

    # ------------------------------------------------------------------ data

    def load_dataset(self):
        """Recursively find all image and mask pairs in the dataset."""
        print(f"Loading dataset from {self.dataset_path}")

        image_extensions = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"}

        image_files = []
        for ext in image_extensions:
            for p in self.dataset_path.rglob(f"*{ext}"):
                # Exclude mask files themselves
                if "_mask" not in p.stem:
                    image_files.append(p)
            for p in self.dataset_path.rglob(f"*{ext.upper()}"):
                if "_mask" not in p.stem:
                    image_files.append(p)

        for img_path in natsorted(image_files):
            mask_path = img_path.with_name(img_path.stem + "_mask.png")
            if mask_path.exists():
                self.image_mask_pairs.append((img_path, mask_path))

        print(f"Found {len(self.image_mask_pairs)} image-mask pairs")

        if not self.image_mask_pairs:
            messagebox.showerror("Error", "No image-mask pairs found in dataset")
            self.root.quit()

    def load_class_info(self, class_labels_path: Optional[Path]):
        """Load class information from YAML file if present."""
        if class_labels_path is None or not class_labels_path.exists():
            print("No class info file found, using default colors and names")
            return

        print(f"Loading class information from {class_labels_path}")
        try:
            with open(class_labels_path, "r") as f:
                data = yaml.safe_load(f)

            for class_id, (class_name, color_hex) in enumerate(zip(data["names"], data["colors"])):
                self.class_info[class_id] = {
                    "name": class_name,
                    "color": color_hex,
                }

            print(f"Loaded {len(self.class_info)} class definitions")
        except Exception as e:
            print(f"Warning: Failed to load class info from {class_labels_path}: {e}")

    def load_state(self):
        """Load validation state from JSON file."""
        if self.state_file.exists():
            with open(self.state_file, "r") as f:
                self.validation_state = json.load(f)
            print(f"Loaded validation state: {len(self.validation_state)} entries")
        else:
            self.validation_state = {}

    def save_state(self):
        """Save validation state to JSON file."""
        tmp = self.state_file.with_suffix(".json.tmp")
        with open(tmp, "w") as f:
            json.dump(self.validation_state, f, indent=2)
        # Atomic replace so an interrupted write cannot truncate existing work
        tmp.replace(self.state_file)

    def key_for(self, index: int) -> str:
        img_path, _ = self.image_mask_pairs[index]
        return str(img_path.relative_to(self.dataset_path))

    def status_of(self, index: int) -> str:
        return self.validation_state.get(self.key_for(index), "unvalidated")

    # -------------------------------------------------------------- rendering

    def color_name_to_rgb(self, color: str) -> Tuple[int, int, int]:
        """Convert a color name or hex string to an (R, G, B) tuple."""
        if color.startswith("#"):
            h = color.lstrip("#")
            return tuple(int(h[i : i + 2], 16) for i in (0, 2, 4))
        rgb = NAMED_COLORS.get(color.lower())
        if rgb:
            return tuple(rgb)
        return (255, 0, 0)

    def get_class_color(self, class_id: int) -> Tuple[int, int, int]:
        """Return the (R, G, B) color for a given class ID."""
        if class_id in self.class_info:
            return self.color_name_to_rgb(self.class_info[class_id]["color"])
        fallback = DEFAULT_CLASS_COLORS[class_id % len(DEFAULT_CLASS_COLORS)]
        return tuple(fallback)

    def get_class_name(self, class_id: int) -> str:
        """Return the name for a given class ID."""
        if class_id in self.class_info:
            return self.class_info[class_id]["name"]
        return f"Class {class_id}"

    def compose(self, index: int, box: Tuple[int, int], legend: bool) -> Image.Image:
        """Composite image + mask overlay, decoded no larger than `box`.

        Runs on worker threads, so it must not touch tkinter.
        """
        img_path, mask_path = self.image_mask_pairs[index]

        image = Image.open(img_path)
        # draft() lets libjpeg decode at a reduced DCT scale: the single biggest
        # win for grid paging, since full 1080p decodes dominate otherwise.
        image.draft("RGB", box)
        image = image.convert("RGB")
        image.thumbnail(box, Image.Resampling.BILINEAR)

        mask_img = Image.open(mask_path)
        if mask_img.mode != "L":
            mask_arr = np.array(mask_img)
            if mask_arr.ndim == 3:
                mask_arr = mask_arr[:, :, 0]
            mask_img = Image.fromarray(mask_arr.astype(np.uint8), mode="L")
        if mask_img.size != image.size:
            mask_img = mask_img.resize(image.size, Image.Resampling.NEAREST)

        mask_arr = np.array(mask_img)
        unique_ids = np.unique(mask_arr)

        if self.show_overlay:
            lut = np.zeros((256, 3), dtype=np.uint8)
            for class_id in unique_ids:
                lut[int(class_id)] = self.get_class_color(int(class_id))
            tint = lut[mask_arr]
            base = np.asarray(image, dtype=np.float32)
            blended = base * (1.0 - self.overlay_alpha) + tint * self.overlay_alpha
            result = Image.fromarray(blended.astype(np.uint8), mode="RGB")
        else:
            result = image

        if legend:
            draw = ImageDraw.Draw(result)
            font = ImageFont.load_default(size=18)
            legend_x = 10
            legend_y = 10
            swatch = 16
            pad = 4

            for class_id in sorted(unique_ids):
                name = self.get_class_name(int(class_id))
                r, g, b = self.get_class_color(int(class_id))
                hex_color = f"#{r:02x}{g:02x}{b:02x}"

                draw.rectangle(
                    [legend_x, legend_y, legend_x + swatch, legend_y + swatch],
                    fill=hex_color,
                    outline="#000000",
                )
                text_x = legend_x + swatch + pad
                text_bbox = draw.textbbox((text_x, legend_y), name, font=font)
                draw.rectangle(text_bbox, fill="#000000")
                draw.text((text_x, legend_y), name, fill="white", font=font)
                legend_y += swatch + pad

        return result

    def thumb_size(self) -> int:
        return self.THUMB_STEPS[self.thumb_step]

    def render_thumb(self, index: int) -> Image.Image:
        """Return a cached thumbnail for `index`, rendering it if needed."""
        size = self.thumb_size()
        cache_key = (index, size, round(self.overlay_alpha, 3), self.show_overlay)

        with self.cache_lock:
            hit = self.thumb_cache.get(cache_key)
            if hit is not None:
                self.thumb_cache.move_to_end(cache_key)
                return hit

        thumb = self.compose(index, (size, size), legend=False)

        with self.cache_lock:
            self.thumb_cache[cache_key] = thumb
            while len(self.thumb_cache) > self.cache_limit:
                self.thumb_cache.popitem(last=False)
        return thumb

    # ------------------------------------------------------------------- grid

    @property
    def page_size(self) -> int:
        return max(1, self.columns * self.rows)

    def setup_ui(self):
        """Create the user interface."""
        main_container = ttk.Frame(self.root)
        main_container.pack(fill=tk.BOTH, expand=True)

        self.grid_frame = tk.Frame(main_container, bg="#FFFFFF")
        self.grid_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=6, pady=6)
        self.grid_frame.bind("<Configure>", self._on_grid_configure)

        sidebar = ttk.Frame(main_container, relief=tk.RIDGE, borderwidth=2)
        sidebar.pack(side=tk.RIGHT, fill=tk.Y, padx=(0, 8), pady=8)

        # === PAGE SECTION ===
        page_section = ttk.LabelFrame(sidebar, text="Page", padding="12")
        page_section.pack(fill=tk.X, padx=10, pady=(10, 5))

        ttk.Label(page_section, textvariable=self.page_info_var, font=("Arial", 11, "bold")).pack(
            pady=(0, 4)
        )
        ttk.Label(
            page_section, textvariable=self.selection_var, font=("Courier", 8), wraplength=200
        ).pack(pady=(0, 6))

        page_buttons = ttk.Frame(page_section)
        page_buttons.pack(fill=tk.X)
        ttk.Button(page_buttons, text="⏮", command=self.jump_to_start, width=4).pack(
            side=tk.LEFT, padx=1, expand=True, fill=tk.X
        )
        ttk.Button(page_buttons, text="◀ Page", command=self.previous_page, width=6).pack(
            side=tk.LEFT, padx=1, expand=True, fill=tk.X
        )
        ttk.Button(page_buttons, text="Page ▶", command=self.next_page, width=6).pack(
            side=tk.LEFT, padx=1, expand=True, fill=tk.X
        )
        ttk.Button(page_buttons, text="⏭", command=self.jump_to_end, width=4).pack(
            side=tk.LEFT, padx=1, expand=True, fill=tk.X
        )

        ttk.Button(
            page_section, text="⏩ Next Unvalidated (U)", command=self.jump_to_next_unvalidated
        ).pack(fill=tk.X, pady=(6, 0))

        # === VALIDATION SECTION ===
        validation_section = ttk.LabelFrame(sidebar, text="Validate", padding="12")
        validation_section.pack(fill=tk.X, padx=10, pady=5)

        ttk.Button(
            validation_section, text="✓ Pass selected (Y)", command=lambda: self.validate("pass")
        ).pack(fill=tk.X, pady=2)
        ttk.Button(
            validation_section, text="✗ Fail selected (N)", command=lambda: self.validate("fail")
        ).pack(fill=tk.X, pady=2)

        ttk.Separator(validation_section, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=8)

        ttk.Button(
            validation_section,
            text="✓✓ PASS ALL on screen (T)",
            command=lambda: self.validate_page("pass"),
        ).pack(fill=tk.X, pady=2)
        ttk.Button(
            validation_section,
            text="✗✗ FAIL ALL on screen (B)",
            command=lambda: self.validate_page("fail"),
        ).pack(fill=tk.X, pady=2)

        ttk.Button(
            validation_section, text="↺ Clear selected (C)", command=lambda: self.validate(None)
        ).pack(fill=tk.X, pady=(8, 2))

        # === DISPLAY OPTIONS SECTION ===
        options_section = ttk.LabelFrame(sidebar, text="Display Options", padding="12")
        options_section.pack(fill=tk.X, padx=10, pady=5)

        ttk.Button(options_section, text="🔍 Zoom selected (Z)", command=self.open_zoom).pack(
            fill=tk.X, pady=(0, 8)
        )

        ttk.Label(options_section, text="Thumbnail size:").pack(anchor=tk.W)
        self.size_scale = ttk.Scale(
            options_section,
            from_=0,
            to=len(self.THUMB_STEPS) - 1,
            orient=tk.HORIZONTAL,
            value=self.thumb_step,
            command=self._on_size_change,
        )
        self.size_scale.pack(fill=tk.X, pady=(0, 8))

        ttk.Checkbutton(
            options_section,
            text="Show Overlay (O)",
            variable=self.show_overlay_var,
            command=self.toggle_overlay,
        ).pack(anchor=tk.W)

        ttk.Label(options_section, text="Overlay Alpha:").pack(anchor=tk.W, pady=(6, 0))
        self.alpha_scale = ttk.Scale(
            options_section,
            from_=0.0,
            to=1.0,
            orient=tk.HORIZONTAL,
            value=self.overlay_alpha,
            command=self._on_alpha_change,
        )
        self.alpha_scale.pack(fill=tk.X, pady=(0, 5))

        # === JUMP SECTION ===
        jump_section = ttk.LabelFrame(sidebar, text="Jump to Frame", padding="12")
        jump_section.pack(fill=tk.X, padx=10, pady=5)
        jump_frame = ttk.Frame(jump_section)
        jump_frame.pack(fill=tk.X)
        self.jump_entry = ttk.Entry(jump_frame, width=10)
        self.jump_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        ttk.Button(jump_frame, text="Go", command=self._submit_jump, width=5).pack(side=tk.RIGHT)

        # === KEYBOARD SHORTCUTS SECTION ===
        shortcuts_section = ttk.LabelFrame(sidebar, text="Keyboard Shortcuts", padding="12")
        shortcuts_section.pack(fill=tk.X, padx=10, pady=5)

        shortcuts_text = (
            "← → ↑ ↓ - Move selection\n"
            "Y / N     - Pass / fail selected\n"
            "T / B     - Pass / fail whole page\n"
            "C         - Clear selected\n"
            "Z / Enter - Zoom selected\n"
            "+ / -     - Thumbnail size\n"
            "PgUp/PgDn - Previous / next page\n"
            "U         - Next unvalidated\n"
            "O         - Toggle overlay\n"
            "Home/End  - First / last page"
        )
        ttk.Label(
            shortcuts_section, text=shortcuts_text, font=("Courier", 9), justify=tk.LEFT
        ).pack(anchor=tk.W)

        # Status bar at bottom
        status_frame = ttk.Frame(self.root)
        status_frame.pack(side=tk.BOTTOM, fill=tk.X)
        ttk.Label(status_frame, textvariable=self.status_var, relief=tk.SUNKEN).pack(fill=tk.X)

        # Keyboard bindings
        self.bind_key("<Left>", lambda: self.move_selection(-1))
        self.bind_key("<Right>", lambda: self.move_selection(1))
        self.bind_key("<Up>", lambda: self.move_selection(-self.columns))
        self.bind_key("<Down>", lambda: self.move_selection(self.columns))
        self.bind_key("y", lambda: self.validate("pass"))
        self.bind_key("n", lambda: self.validate("fail"))
        self.bind_key("t", lambda: self.validate_page("pass"))
        self.bind_key("b", lambda: self.validate_page("fail"))
        self.bind_key("c", lambda: self.validate(None))
        self.bind_key("z", self.open_zoom)
        self.bind_key("<Return>", self.open_zoom)
        self.bind_key("u", self.jump_to_next_unvalidated)
        self.bind_key("o", lambda: self.toggle_overlay(flip=True))
        self.bind_key("<Prior>", self.previous_page)
        self.bind_key("<Next>", self.next_page)
        self.bind_key("<Home>", self.jump_to_start)
        self.bind_key("<End>", self.jump_to_end)
        self.bind_key("<plus>", lambda: self.change_thumb_size(1))
        self.bind_key("<equal>", lambda: self.change_thumb_size(1))
        self.bind_key("<minus>", lambda: self.change_thumb_size(-1))

        # Typing in the jump box must not trigger shortcuts; Enter submits it.
        self.jump_entry.bind("<Return>", self._submit_jump)

    def bind_key(self, sequence: str, action):
        """Bind a global shortcut that stays inert while the jump box has focus."""

        def handler(_event=None):
            if self.root.focus_get() is self.jump_entry:
                return None
            action()
            return "break"

        self.root.bind(sequence, handler)

    def _submit_jump(self, _event=None):
        self.jump_to_frame()
        self.root.focus_set()
        return "break"

    def _on_grid_configure(self, _event=None):
        """Recompute the grid shape when the window resizes (debounced)."""
        if self._layout_job is not None:
            self.root.after_cancel(self._layout_job)
        self._layout_job = self.root.after(120, self.relayout)

    def relayout(self):
        """Size the grid to the available space and redraw."""
        self._layout_job = None
        size = self.thumb_size()
        avail_w = max(self.grid_frame.winfo_width(), size)
        avail_h = max(self.grid_frame.winfo_height(), size)

        # Cells are laid out for 16:9 source frames plus a caption strip.
        cell_w = size + 14
        cell_h = int(size * 9 / 16) + 34

        columns = max(1, avail_w // cell_w)
        rows = max(1, avail_h // cell_h)

        if (columns, rows) != (self.columns, self.rows):
            self.columns, self.rows = columns, rows
            self.rebuild_grid()

        self.page_start = (self.selected_index // self.page_size) * self.page_size
        self.update_display()

    def rebuild_grid(self):
        """Recreate cell widgets after a shape change."""
        for cell in self.cells:
            cell["frame"].destroy()
        self.cells = []

        for row in range(self.rows):
            self.grid_frame.rowconfigure(row, weight=1)
        for col in range(self.columns):
            self.grid_frame.columnconfigure(col, weight=1)

        for position in range(self.columns * self.rows):
            row, col = divmod(position, self.columns)
            frame = tk.Frame(
                self.grid_frame,
                bg=self.UNSET_COLOR,
                highlightthickness=3,
                highlightbackground=self.IDLE_COLOR,
                highlightcolor=self.IDLE_COLOR,
            )
            frame.grid(row=row, column=col, padx=2, pady=2)
            image_label = tk.Label(frame, bd=0, bg="#000000")
            image_label.pack(padx=3, pady=(3, 0))
            caption = tk.Label(
                frame, text="", font=("Courier", 8), bg=self.UNSET_COLOR, fg="#FFFFFF", anchor=tk.W
            )
            caption.pack(fill=tk.X, padx=3, pady=(1, 3))

            cell = {"frame": frame, "image": image_label, "caption": caption, "index": None}
            for widget in (frame, image_label, caption):
                widget.bind("<Button-1>", lambda e, c=cell: self.select_cell(c))
                widget.bind("<Double-Button-1>", lambda e, c=cell: self.select_cell(c, zoom=True))
            self.cells.append(cell)

    def select_cell(self, cell: Dict, zoom: bool = False):
        if cell["index"] is None:
            return
        self.selected_index = cell["index"]
        self.update_display()
        if zoom:
            self.open_zoom()

    def update_display(self):
        """Redraw the current page of thumbnails."""
        if not self.image_mask_pairs or not self.cells:
            return

        total = len(self.image_mask_pairs)
        self.selected_index = min(max(self.selected_index, 0), total - 1)
        self.page_start = min(max(self.page_start, 0), max(0, total - 1))

        indices = [i for i in range(self.page_start, min(self.page_start + len(self.cells), total))]

        # Decode everything missing from the cache in parallel before touching tk.
        pending = [i for i in indices if self._cached(i) is None]
        if pending:
            list(self.executor.map(self.render_thumb, pending))

        for position, cell in enumerate(self.cells):
            if position < len(indices):
                index = indices[position]
                cell["index"] = index
                photo = ImageTk.PhotoImage(self.render_thumb(index))
                self.photo_refs[position] = photo
                cell["image"].config(image=photo)

                status = self.status_of(index)
                color = {
                    "pass": self.PASS_COLOR,
                    "fail": self.FAIL_COLOR,
                }.get(status, self.UNSET_COLOR)
                glyph = {"pass": "✓", "fail": "✗"}.get(status, "○")
                name = self.image_mask_pairs[index][0].name
                cell["frame"].config(bg=color)
                cell["caption"].config(bg=color, text=f"{glyph} {index + 1}  {name[-22:]}")
                selected = index == self.selected_index
                cell["frame"].config(
                    highlightbackground=self.SELECT_COLOR if selected else self.IDLE_COLOR,
                    highlightcolor=self.SELECT_COLOR if selected else self.IDLE_COLOR,
                )
                cell["frame"].grid()
            else:
                cell["index"] = None
                cell["frame"].grid_remove()

        page_number = self.page_start // self.page_size + 1
        page_count = (total + self.page_size - 1) // self.page_size
        self.page_info_var.set(f"Page {page_number} / {page_count}  ({self.columns}×{self.rows})")
        self.selection_var.set(
            f"#{self.selected_index + 1} {self.image_mask_pairs[self.selected_index][0].name[-30:]}"
        )

        validated = sum(1 for v in self.validation_state.values() if v in ("pass", "fail"))
        passed = sum(1 for v in self.validation_state.values() if v == "pass")
        failed = sum(1 for v in self.validation_state.values() if v == "fail")
        self.status_var.set(
            f"Dataset: {self.dataset_path.name} | Frames: {total} | "
            f"Validated: {validated} | Pass: {passed} | Fail: {failed}"
        )

        self.prefetch_next_page()
        self.refresh_zoom()

    def _cached(self, index: int) -> Optional[Image.Image]:
        key = (index, self.thumb_size(), round(self.overlay_alpha, 3), self.show_overlay)
        with self.cache_lock:
            return self.thumb_cache.get(key)

    def prefetch_next_page(self):
        """Warm the cache for the following page so paging feels instant."""
        start = self.page_start + self.page_size
        end = min(start + self.page_size, len(self.image_mask_pairs))
        for index in range(start, end):
            if self._cached(index) is None:
                self.executor.submit(self.render_thumb, index)

    # ------------------------------------------------------------ interaction

    def move_selection(self, delta: int):
        total = len(self.image_mask_pairs)
        self.selected_index = min(max(self.selected_index + delta, 0), total - 1)
        self.page_start = (self.selected_index // self.page_size) * self.page_size
        self.update_display()

    def validate(self, result: Optional[str]):
        """Mark the selected image, then advance the selection."""
        if not self.image_mask_pairs:
            return

        key = self.key_for(self.selected_index)
        if result is None:
            self.validation_state.pop(key, None)
        else:
            self.validation_state[key] = result
        self.save_state()

        if self.selected_index < len(self.image_mask_pairs) - 1:
            self.move_selection(1)
        else:
            self.update_display()

    def validate_page(self, result: str):
        """Mark every image currently on screen. Stays on the page so you can fix outliers."""
        indices = [cell["index"] for cell in self.cells if cell["index"] is not None]
        if not indices:
            return
        for index in indices:
            self.validation_state[self.key_for(index)] = result
        self.save_state()
        self.update_display()

    def next_page(self):
        if self.page_start + self.page_size < len(self.image_mask_pairs):
            self.page_start += self.page_size
            self.selected_index = self.page_start
        self.update_display()

    def previous_page(self):
        if self.page_start > 0:
            self.page_start = max(0, self.page_start - self.page_size)
            self.selected_index = self.page_start
        self.update_display()

    def jump_to_start(self):
        self.selected_index = 0
        self.page_start = 0
        self.update_display()

    def jump_to_end(self):
        self.selected_index = len(self.image_mask_pairs) - 1
        self.page_start = (self.selected_index // self.page_size) * self.page_size
        self.update_display()

    def jump_to_frame(self):
        """Jump to specific frame number."""
        try:
            frame_num = int(self.jump_entry.get())
            if 1 <= frame_num <= len(self.image_mask_pairs):
                self.selected_index = frame_num - 1
                self.page_start = (self.selected_index // self.page_size) * self.page_size
                self.update_display()
            else:
                messagebox.showerror(
                    "Error",
                    f"Frame number must be between 1 and {len(self.image_mask_pairs)}",
                )
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid frame number")

    def jump_to_next_unvalidated(self):
        """Select the next image with no verdict, wrapping around."""
        total = len(self.image_mask_pairs)
        order = list(range(self.selected_index, total)) + list(range(0, self.selected_index))
        for index in order:
            if self.key_for(index) not in self.validation_state:
                self.selected_index = index
                self.page_start = (index // self.page_size) * self.page_size
                self.update_display()
                return

        messagebox.showinfo("Complete", "All images have been validated!")
        self.update_display()

    def toggle_overlay(self, flip: bool = False):
        """Toggle mask overlay visibility."""
        if flip:
            self.show_overlay_var.set(not self.show_overlay_var.get())
        self.show_overlay = self.show_overlay_var.get()
        self.update_display()

    def _on_alpha_change(self, value):
        """Handle alpha slider change."""
        self.overlay_alpha = float(value)
        self.update_display()

    def _on_size_change(self, value):
        """Handle thumbnail size slider change."""
        step = int(round(float(value)))
        if step != self.thumb_step:
            self.thumb_step = step
            self.relayout()

    def change_thumb_size(self, delta: int):
        step = min(max(self.thumb_step + delta, 0), len(self.THUMB_STEPS) - 1)
        if step != self.thumb_step:
            self.thumb_step = step
            self.size_scale.set(step)
            self.relayout()

    # ------------------------------------------------------------------ zoom

    def open_zoom(self):
        """Show the selected frame full size in its own window."""
        if self.zoom_window is not None and self.zoom_window.winfo_exists():
            self.close_zoom()
            return

        window = tk.Toplevel(self.root)
        window.title("Zoom")
        self.zoom_window = window
        self.zoom_label = tk.Label(window, bd=0, bg="#000000")
        self.zoom_label.pack(fill=tk.BOTH, expand=True)

        window.bind("<Escape>", lambda e: self.close_zoom())
        window.bind("z", lambda e: self.close_zoom())
        window.bind("<Return>", lambda e: self.close_zoom())
        window.bind("y", lambda e: self.validate("pass"))
        window.bind("n", lambda e: self.validate("fail"))
        window.bind("c", lambda e: self.validate(None))
        window.bind("<Left>", lambda e: self.move_selection(-1))
        window.bind("<Right>", lambda e: self.move_selection(1))
        window.protocol("WM_DELETE_WINDOW", self.close_zoom)

        self.refresh_zoom()
        window.focus_set()

    def refresh_zoom(self):
        """Repaint the zoom window for the current selection, if it is open."""
        if self.zoom_window is None or not self.zoom_window.winfo_exists():
            return

        max_w = int(self.root.winfo_screenwidth() * 0.9)
        max_h = int(self.root.winfo_screenheight() * 0.85)
        image = self.compose(self.selected_index, (max_w, max_h), legend=True)

        self.zoom_photo = ImageTk.PhotoImage(image)
        if self.zoom_label is not None:
            self.zoom_label.config(image=self.zoom_photo)

        status = self.status_of(self.selected_index)
        glyph = {"pass": "✓ PASS", "fail": "✗ FAIL"}.get(status, "○ unvalidated")
        name = self.image_mask_pairs[self.selected_index][0].name
        self.zoom_window.title(f"[{glyph}]  #{self.selected_index + 1}  {name}")

    def close_zoom(self):
        if self.zoom_window is not None and self.zoom_window.winfo_exists():
            self.zoom_window.destroy()
        self.zoom_window = None
        self.zoom_label = None
        self.zoom_photo = None
        self.root.focus_set()

    def run(self):
        """Start the UI main loop."""
        self.root.mainloop()
        self.executor.shutdown(wait=False)


def main():
    parser = argparse.ArgumentParser(description="Segmentation Mask Dataset Validation UI")
    parser.add_argument("dataset_path", nargs="?", help="Path to segmask dataset root directory")
    parser.add_argument(
        "-c",
        "--classes",
        type=str,
        help="Path to class label info YAML file (with 'names' and 'colors' keys).",
    )
    args = parser.parse_args()

    if args.dataset_path:
        dataset_path_str = args.dataset_path
    else:
        root = tk.Tk()
        root.withdraw()
        dataset_path_str = filedialog.askdirectory(
            title="Select Segmentation Mask Dataset Directory"
        )
        root.destroy()

        if not dataset_path_str:
            print("No dataset selected. Exiting.")
            return

    dataset_path = Path(dataset_path_str)

    classes_path: Optional[Path] = None
    if args.classes:
        classes_path = Path(args.classes)
    else:
        for path in dataset_path.iterdir():
            if re.search(r".*\.(ya?ml)$", str(path)):
                classes_path = dataset_path / path.name
                break

    validator = SegmaskDatasetValidator(dataset_path, classes_path)
    validator.run()


if __name__ == "__main__":
    main()
