#!/usr/bin/env python3
"""
YOLO Dataset Validation UI

This script provides a GUI for manually validating YOLO dataset annotations.
Images are shown as a page of grid thumbnails with their bounding boxes,
polygons and keypoints drawn on. Verdicts are keyboard-driven, one image at a
time or a whole page at once. The validation state is saved and can be resumed
later.
"""

import argparse
import json
import os
import threading
import tkinter as tk
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Dict, List, Optional, Tuple

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


class YOLODatasetValidator:
    """Contact-sheet validator: page through grid thumbnails, judge from the keyboard."""

    # How many frames a page holds. The thumbnail size follows from the window
    # size, so this control is really "how many fit on screen".
    PAGE_STEPS = [1, 2, 4, 6, 9, 12, 16, 20, 30, 42, 56]
    DEFAULT_PAGE = 4
    MIN_THUMB = 80

    PASS_COLOR = "#2E7D32"
    FAIL_COLOR = "#C62828"
    UNSET_COLOR = "#9E9E9E"
    SELECT_COLOR = "#1565C0"
    IDLE_COLOR = "#ECECEC"

    def __init__(self, dataset_path: Path, class_labels_path: Path):
        self.dataset_path = dataset_path
        self.state_file = self.dataset_path / "validation_state.json"
        self.class_labels_path = class_labels_path

        # Data structures
        self.image_annotation_pairs: List[Tuple[Path, Path]] = []
        self.validation_state: Dict[str, str] = {}  # path -> 'pass'/'fail'
        self.class_info: Dict[int, Dict] = {}  # class_id -> {name, color}
        self.show_labels = True

        # Grid state
        self.page_step = self.PAGE_STEPS.index(self.DEFAULT_PAGE)
        self.thumb_px = 400
        self.selected_index = 0
        self.page_start = 0
        self.columns = 1
        self.rows = 1
        self.cells: List[Dict] = []

        # Thumbnails are decoded off the main thread and cached, so paging stays
        # snappy. The PhotoImage cache is separate because those must be built on
        # the main thread; keeping them lets a redraw skip cells whose picture has
        # not changed.
        self.thumb_cache: "OrderedDict[Tuple, Image.Image]" = OrderedDict()
        self.cache_limit = 800
        self.photo_cache: "OrderedDict[Tuple, ImageTk.PhotoImage]" = OrderedDict()
        self.photo_limit = 200
        self.cache_lock = threading.Lock()
        self.executor = ThreadPoolExecutor(max_workers=min(8, (os.cpu_count() or 4)))
        self._layout_job: Optional[str] = None

        # Zoom overlay
        self.zoom_window: Optional[tk.Toplevel] = None
        self.zoom_label: Optional[tk.Label] = None
        self.zoom_photo: Optional[ImageTk.PhotoImage] = None

        # UI components
        self.root = tk.Tk()
        self.root.title("YOLO Dataset Validator")
        self.root.geometry("1600x1000")

        # Status
        self.status_var = tk.StringVar()
        self.page_info_var = tk.StringVar()
        self.selection_var = tk.StringVar()
        self.show_labels_var = tk.BooleanVar(value=True)

        # Fallback colors for classes with no color in the dataset yaml
        self.colors = [
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

        # Initialize
        self.load_dataset()
        self.load_class_info(self.class_labels_path)
        self.load_state()
        self.setup_ui()
        self.jump_to_next_unvalidated()

    # ------------------------------------------------------------------ data

    def load_dataset(self):
        """Recursively find all image and annotation pairs in the dataset."""
        print(f"Loading dataset from {self.dataset_path}")

        # Common image extensions
        image_extensions = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"}

        # Find all images
        image_files = []
        for ext in image_extensions:
            image_files.extend(self.dataset_path.rglob(f"*{ext}"))
            image_files.extend(self.dataset_path.rglob(f"*{ext.upper()}"))

        # Match with annotation files
        for img_path in natsorted(image_files):
            # Try to find corresponding label file
            # YOLO convention: images/xxx.jpg -> labels/xxx.txt
            img_str = str(img_path)

            # Check if in an 'images' directory
            if "/images/" in img_str or "\\images\\" in img_str:
                label_str = img_str.replace("/images/", "/labels/").replace(
                    "\\images\\", "\\labels\\"
                )
            else:
                # Assume labels are in same directory or parallel structure
                label_str = img_str

            label_path = Path(label_str).with_suffix(".txt")

            if label_path.exists():
                self.image_annotation_pairs.append((img_path, label_path))

        print(f"Found {len(self.image_annotation_pairs)} image-annotation pairs")

        if not self.image_annotation_pairs:
            messagebox.showerror("Error", "No image-annotation pairs found in dataset")
            self.root.quit()

    def load_class_info(self, class_labels_path: Path):
        """Load class names/colors from dataset yaml if present."""

        class_labels_path = Path(class_labels_path)
        if not class_labels_path.exists():
            print("No class info file found, using default colors and names")
            return

        print(f"Loading class information from {class_labels_path}")
        try:
            with open(class_labels_path, "r") as f:
                data = yaml.safe_load(f) or {}

            names_data = data.get("names", [])
            colors_data = data.get("colors", [])

            if isinstance(names_data, dict):
                # YOLO can store names as {0: "a", 1: "b", ...}
                normalized_names = []
                for key in sorted(names_data.keys(), key=lambda x: int(x)):
                    normalized_names.append(str(names_data[key]))
                names_data = normalized_names
            elif isinstance(names_data, list):
                names_data = [str(name) for name in names_data]
            else:
                names_data = []

            if not names_data:
                print("No 'names' field found in class yaml; using default class labels")
                return

            for class_id, class_name in enumerate(names_data):
                if class_id < len(colors_data):
                    color_hex = colors_data[class_id]
                else:
                    color_hex = self.colors[class_id % len(self.colors)]
                self.class_info[class_id] = {
                    "name": class_name,
                    "color": color_hex,
                }

            print(f"Loaded {len(self.class_info)} class definitions")
            return
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
        """Save validation state to the JSON file."""
        tmp = self.state_file.with_suffix(".json.tmp")
        with open(tmp, "w") as f:
            json.dump(self.validation_state, f, indent=2)
        # Atomic replace so an interrupted write cannot truncate existing work
        tmp.replace(self.state_file)

    def key_for(self, index: int) -> str:
        img_path, _ = self.image_annotation_pairs[index]
        return str(img_path.relative_to(self.dataset_path))

    def status_of(self, index: int) -> str:
        return self.validation_state.get(self.key_for(index), "unvalidated")

    # ------------------------------------------------------------------- grid

    @property
    def page_size(self) -> int:
        return self.PAGE_STEPS[self.page_step]

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

        ttk.Label(options_section, text="Images per page (+ / -):").pack(anchor=tk.W)
        self.size_scale = ttk.Scale(
            options_section,
            from_=0,
            to=len(self.PAGE_STEPS) - 1,
            orient=tk.HORIZONTAL,
            value=self.page_step,
            command=self._on_size_change,
        )
        self.size_scale.pack(fill=tk.X, pady=(0, 8))

        ttk.Checkbutton(
            options_section,
            text="Show Labels (L)",
            variable=self.show_labels_var,
            command=self.toggle_labels,
        ).pack(anchor=tk.W)

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
            "+ / -     - Fewer / more per page\n"
            "PgUp/PgDn - Previous / next page\n"
            "U         - Next unvalidated\n"
            "L         - Toggle labels\n"
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
        self.bind_key("l", lambda: self.toggle_labels(flip=True))
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
        """Pick the grid shape that shows `page_size` frames as large as possible."""
        self._layout_job = None
        target = self.page_size
        avail_w = max(self.grid_frame.winfo_width(), 320)
        avail_h = max(self.grid_frame.winfo_height(), 240)

        # Each cell is a 16:9 thumbnail plus padding and a caption strip.
        pad_w, pad_h = 14, 34

        size, columns, rows = float(self.MIN_THUMB), target, 1
        best_score = (0.0, 0)
        for candidate_columns in range(1, target + 1):
            candidate_rows = -(-target // candidate_columns)  # ceil
            width = avail_w / candidate_columns - pad_w
            height = avail_h / candidate_rows - pad_h
            candidate = min(width, height * 16 / 9)
            if candidate <= 0:
                continue
            # Prefer the biggest thumbnail; break ties toward fewer empty slots.
            score = (candidate, -(candidate_columns * candidate_rows - target))
            if score > best_score:
                best_score = score
                size, columns, rows = candidate, candidate_columns, candidate_rows

        # Quantize so a few pixels of window resize does not churn the whole grid
        thumb_px = max(self.MIN_THUMB, int(size) // 8 * 8)

        if (columns, rows) != (self.columns, self.rows):
            self.columns, self.rows, self.thumb_px = columns, rows, thumb_px
            self.rebuild_grid()
        elif thumb_px != self.thumb_px:
            self.thumb_px = thumb_px
            self.resize_cells()

        self.page_start = (self.selected_index // self.page_size) * self.page_size
        self.update_display()

    def resize_cells(self):
        """Resize existing cells in place rather than tearing the grid down."""
        width, height = self.thumb_box()
        for cell in self.cells:
            cell["image"].config(width=width, height=height)
            cell["photo_key"] = None

    def rebuild_grid(self):
        """Recreate the cell widgets after a shape change."""
        for cell in self.cells:
            cell["frame"].destroy()
        self.cells = []

        for row in range(self.rows):
            self.grid_frame.rowconfigure(row, weight=1)
        for col in range(self.columns):
            self.grid_frame.columnconfigure(col, weight=1)

        width, height = self.thumb_box()
        for position in range(self.page_size):
            row, col = divmod(position, self.columns)
            frame = tk.Frame(
                self.grid_frame,
                bg=self.UNSET_COLOR,
                highlightthickness=3,
                highlightbackground=self.IDLE_COLOR,
                highlightcolor=self.IDLE_COLOR,
            )
            frame.grid(row=row, column=col, padx=2, pady=2)
            # Fixed pixel size: the label must not resize itself around whatever
            # image it is handed, or every cell reflows as a page loads.
            image_label = tk.Label(frame, bd=0, bg="#000000", width=width, height=height)
            image_label.pack(padx=3, pady=(3, 0))
            # width=1 char keeps a long filename from widening the cell
            caption = tk.Label(
                frame,
                text="",
                font=("Courier", 8),
                bg=self.UNSET_COLOR,
                fg="#FFFFFF",
                anchor=tk.W,
                width=1,
            )
            caption.pack(fill=tk.X, padx=3, pady=(1, 3))

            cell = {
                "frame": frame,
                "image": image_label,
                "caption": caption,
                "index": None,
                "photo_key": None,
                "photo": None,
                "color": None,
                "outline": None,
                "text": None,
                "visible": True,
            }
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

    # --------------------------------------------------------------- drawing

    def color_name_to_hex(self, color_name: str) -> str:
        """Convert color name to hex code."""
        # Check if already hex
        if color_name.startswith("#"):
            return color_name

        # Convert to lowercase and return mapped color or default
        color_rgb = NAMED_COLORS.get(
            color_name.lower(), color_name if color_name.startswith("#") else (0, 0, 0)
        )
        return f"#{color_rgb[0]:02x}{color_rgb[1]:02x}{color_rgb[2]:02x}"

    @staticmethod
    def _parse_detect_pose_row(class_id: int, values: List[float]) -> Optional[Dict]:
        """Parse a standard YOLO detect/pose row into a bbox annotation.

        Row layout: class cx cy w h [kpt_x kpt_y kpt_v]...
        Returns None when the row is not a valid detect/pose row so the caller
        can fall back to seg parsing.
        """
        if len(values) < 4:
            return None

        kp_values = values[4:]
        if len(kp_values) % 3 != 0:
            return None

        parsed_keypoints = []
        for i in range(0, len(kp_values), 3):
            v_raw = kp_values[i + 2]
            v_int = int(round(v_raw))
            if abs(v_raw - v_int) > 1e-6 or v_int not in (0, 1, 2):
                return None
            parsed_keypoints.append({"x": kp_values[i], "y": kp_values[i + 1], "v": v_int})

        return {
            "type": "bbox",
            "class_id": class_id,
            "center_x": values[0],
            "center_y": values[1],
            "width": values[2],
            "height": values[3],
            "keypoints": parsed_keypoints,
        }

    @staticmethod
    def _parse_seg_row(class_id: int, values: List[float]) -> Optional[Dict]:
        """Parse a YOLO-seg polygon row into a seg annotation.

        Row layout: class x1 y1 x2 y2 ... xn yn
        Returns None when the row does not form a valid polygon.
        """
        if len(values) < 6 or len(values) % 2 != 0:
            return None

        polygon = []
        for i in range(0, len(values), 2):
            polygon.append({"x": values[i], "y": values[i + 1]})

        if len(polygon) < 3:
            return None

        xs = [p["x"] for p in polygon]
        ys = [p["y"] for p in polygon]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        return {
            "type": "seg",
            "class_id": class_id,
            "center_x": (min_x + max_x) / 2.0,
            "center_y": (min_y + max_y) / 2.0,
            "width": max_x - min_x,
            "height": max_y - min_y,
            "polygon": polygon,
            "keypoints": [],
        }

    def parse_yolo_annotation(self, label_path: Path) -> List[Dict]:
        """Parse YOLO annotation file for bbox/pose and yolo-seg polygon rows."""
        annotations = []
        with open(label_path, "r") as f:
            for line in f:
                parts = line.strip().split()
                if not parts:
                    continue

                try:
                    class_id = int(float(parts[0]))
                    values = [float(v) for v in parts[1:]]
                except ValueError:
                    continue

                # Parse standard YOLO detect/pose rows first.
                bbox = self._parse_detect_pose_row(class_id, values)
                if bbox is not None:
                    annotations.append(bbox)
                    continue

                # Fall back to YOLO-seg polygon rows.
                seg = self._parse_seg_row(class_id, values)
                if seg is not None:
                    annotations.append(seg)
        return annotations

    def class_style(self, class_id: int) -> Tuple[str, str]:
        """Return the (name, hex color) a class draws with."""
        if class_id in self.class_info:
            return (
                self.class_info[class_id]["name"],
                self.color_name_to_hex(self.class_info[class_id]["color"]),
            )
        return f"Class {class_id}", self.colors[class_id % len(self.colors)]

    def compose(self, index: int, box: Tuple[int, int], detail: bool) -> Image.Image:
        """Draw an image's annotations, decoded no larger than `box`.

        Runs on worker threads, so it must not touch tkinter. Line widths, dot
        radii and text size are derived from the rendered size, so a 200px
        thumbnail and a full-screen zoom stay equally legible.
        """
        img_path, label_path = self.image_annotation_pairs[index]

        image = Image.open(img_path)
        # draft() lets libjpeg decode at a reduced DCT scale: the single biggest
        # win for grid paging, since full 1080p decodes dominate otherwise.
        image.draft("RGB", box)
        image = image.convert("RGBA")
        image.thumbnail(box, Image.Resampling.BILINEAR)

        img_width, img_height = image.size
        short_side = min(img_width, img_height)

        draw = ImageDraw.Draw(image)
        seg_overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
        seg_overlay_draw = ImageDraw.Draw(seg_overlay, "RGBA")

        annotations = self.parse_yolo_annotation(label_path)

        line_width = max(1, round(short_side / 180))
        radius = max(2, round(short_side * 0.008))
        font_px = max(9, round(short_side / 18))
        font = ImageFont.load_default(size=font_px)
        # Text is unreadable on a small thumbnail and just obscures the boxes.
        draw_text = self.show_labels and (detail or short_side >= 220)

        kp_colors = [
            "#FF4444",
            "#44FF44",
            "#4444FF",
            "#FFFF44",
            "#FF44FF",
            "#44FFFF",
            "#FF8800",
            "#8844FF",
        ]

        for ann in annotations:
            class_id = ann["class_id"]
            cx = ann["center_x"] * img_width
            cy = ann["center_y"] * img_height
            w = ann["width"] * img_width
            h = ann["height"] * img_height

            x1 = cx - w / 2
            y1 = cy - h / 2
            x2 = cx + w / 2
            y2 = cy + h / 2

            class_name, color = self.class_style(class_id)

            if ann.get("type") == "seg":
                polygon_points = [
                    (p["x"] * img_width, p["y"] * img_height) for p in ann.get("polygon", [])
                ]
                if len(polygon_points) >= 3:
                    color_str = color.lstrip("#")
                    try:
                        color_rgb = tuple(int(color_str[i : i + 2], 16) for i in (0, 2, 4))
                    except ValueError:
                        color_rgb = (255, 0, 0)
                    seg_overlay_draw.polygon(
                        polygon_points,
                        fill=(color_rgb[0], color_rgb[1], color_rgb[2], 35),
                    )
                    draw.line(
                        polygon_points + [polygon_points[0]],
                        fill=color,
                        width=line_width,
                    )
            else:
                # Always draw the bounding box for detect/pose annotations
                draw.rectangle([x1, y1, x2, y2], outline=color, width=line_width)

            if draw_text:
                text_y = max(0.0, y1 - font_px - 4)
                bbox = draw.textbbox((x1, text_y), class_name, font=font)
                draw.rectangle(bbox, fill=color)
                draw.text((x1, text_y), class_name, fill="white", font=font)

            for kp_idx, kp in enumerate(ann.get("keypoints", [])):
                if kp["v"] == 0:
                    continue
                kx = kp["x"] * img_width
                ky = kp["y"] * img_height
                kp_color = kp_colors[kp_idx % len(kp_colors)]
                # Filled circle with a dark outline
                draw.ellipse(
                    [kx - radius - 1, ky - radius - 1, kx + radius + 1, ky + radius + 1],
                    fill="#000000",
                )
                draw.ellipse(
                    [kx - radius, ky - radius, kx + radius, ky + radius],
                    fill=kp_color,
                )
                if draw_text:
                    kp_label = str(kp_idx)
                    anchor = (kx + radius + 2, ky - radius)
                    draw.rectangle(draw.textbbox(anchor, kp_label, font=font), fill="#000000")
                    draw.text(anchor, kp_label, fill=kp_color, font=font)

        image = Image.alpha_composite(image, seg_overlay)
        return image.convert("RGB")

    def thumb_box(self) -> Tuple[int, int]:
        """Exact pixel size every grid thumbnail is padded to."""
        return (self.thumb_px, max(1, round(self.thumb_px * 9 / 16)))

    def thumb_key(self, index: int) -> Tuple:
        return (index, self.thumb_box(), self.show_labels)

    def render_thumb(self, index: int) -> Image.Image:
        """Return a cached thumbnail for `index`, rendering it if needed.

        Every thumbnail is letterboxed to the identical `thumb_box()` size. Cells
        then never change shape when a new image lands in them, which is what made
        the grid jump around while a page was loading.
        """
        box = self.thumb_box()
        cache_key = self.thumb_key(index)

        with self.cache_lock:
            hit = self.thumb_cache.get(cache_key)
            if hit is not None:
                self.thumb_cache.move_to_end(cache_key)
                return hit

        fitted = self.compose(index, box, detail=False)
        thumb = Image.new("RGB", box, (0, 0, 0))
        thumb.paste(fitted, ((box[0] - fitted.width) // 2, (box[1] - fitted.height) // 2))

        with self.cache_lock:
            self.thumb_cache[cache_key] = thumb
            while len(self.thumb_cache) > self.cache_limit:
                self.thumb_cache.popitem(last=False)
        return thumb

    def photo_for(self, index: int) -> Tuple[Tuple, ImageTk.PhotoImage]:
        """Return the cached (key, PhotoImage) pair for `index`.

        Reusing the PhotoImage is what keeps arrow-key navigation quiet: moving the
        selection only recolors borders, it does not re-upload every image to X.
        """
        key = self.thumb_key(index)
        photo = self.photo_cache.get(key)
        if photo is None:
            photo = ImageTk.PhotoImage(self.render_thumb(index))
            self.photo_cache[key] = photo
            while len(self.photo_cache) > self.photo_limit:
                self.photo_cache.popitem(last=False)
        else:
            self.photo_cache.move_to_end(key)
        return key, photo

    def _cached(self, index: int) -> Optional[Image.Image]:
        with self.cache_lock:
            return self.thumb_cache.get(self.thumb_key(index))

    def prefetch_next_page(self):
        """Warm the cache for the following page so paging feels instant."""
        start = self.page_start + self.page_size
        end = min(start + self.page_size, len(self.image_annotation_pairs))
        for index in range(start, end):
            if self._cached(index) is None:
                self.executor.submit(self.render_thumb, index)

    # ------------------------------------------------------------ interaction

    def update_display(self):
        """Redraw the current page of thumbnails."""
        if not self.image_annotation_pairs or not self.cells:
            return

        total = len(self.image_annotation_pairs)
        self.selected_index = min(max(self.selected_index, 0), total - 1)
        self.page_start = min(max(self.page_start, 0), max(0, total - 1))

        indices = [i for i in range(self.page_start, min(self.page_start + len(self.cells), total))]

        # Decode everything missing from the cache in parallel before touching tk.
        pending = [i for i in indices if self._cached(i) is None]
        if pending:
            list(self.executor.map(self.render_thumb, pending))

        for position, cell in enumerate(self.cells):
            if position < len(indices):
                self.paint_cell(cell, indices[position])
            elif cell["visible"]:
                cell["index"] = None
                cell["visible"] = False
                cell["frame"].grid_remove()

        page_number = self.page_start // self.page_size + 1
        page_count = (total + self.page_size - 1) // self.page_size
        self.page_info_var.set(
            f"Page {page_number} / {page_count}   {len(indices)} on screen "
            f"({self.columns}×{self.rows})"
        )
        self.selection_var.set(
            f"#{self.selected_index + 1} "
            f"{self.image_annotation_pairs[self.selected_index][0].name[-30:]}"
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

    def paint_cell(self, cell: Dict, index: int):
        """Push `index` into `cell`, touching only the widget options that changed.

        Every config() call repaints, so a blind rewrite of every cell on every
        keystroke is what the flicker was.
        """
        key, photo = self.photo_for(index)
        if cell["photo_key"] != key:
            cell["photo"] = photo  # keep a strong reference; tk does not
            cell["photo_key"] = key
            cell["image"].config(image=photo)

        status = self.status_of(index)
        color = {"pass": self.PASS_COLOR, "fail": self.FAIL_COLOR}.get(status, self.UNSET_COLOR)
        glyph = {"pass": "✓", "fail": "✗"}.get(status, "○")
        text = f"{glyph} {index + 1}  {self.image_annotation_pairs[index][0].name[-22:]}"
        outline = self.SELECT_COLOR if index == self.selected_index else self.IDLE_COLOR

        if cell["color"] != color:
            cell["color"] = color
            cell["frame"].config(bg=color)
            cell["caption"].config(bg=color)
        if cell["text"] != text:
            cell["text"] = text
            cell["caption"].config(text=text)
        if cell["outline"] != outline:
            cell["outline"] = outline
            cell["frame"].config(highlightbackground=outline, highlightcolor=outline)

        cell["index"] = index
        if not cell["visible"]:
            cell["visible"] = True
            cell["frame"].grid()

    def move_selection(self, delta: int):
        total = len(self.image_annotation_pairs)
        self.selected_index = min(max(self.selected_index + delta, 0), total - 1)
        self.page_start = (self.selected_index // self.page_size) * self.page_size
        self.update_display()

    def validate(self, result: Optional[str]):
        """Mark the selected image, then advance the selection."""
        if not self.image_annotation_pairs:
            return

        key = self.key_for(self.selected_index)
        if result is None:
            self.validation_state.pop(key, None)
        else:
            self.validation_state[key] = result
        self.save_state()

        if self.selected_index < len(self.image_annotation_pairs) - 1:
            self.move_selection(1)
        else:
            self.update_display()

    def validate_page(self, result: str):
        """Mark every image on screen. Stays on the page so you can fix outliers."""
        indices = [cell["index"] for cell in self.cells if cell["index"] is not None]
        if not indices:
            return
        for index in indices:
            self.validation_state[self.key_for(index)] = result
        self.save_state()
        self.update_display()

    def toggle_labels(self, flip: bool = False):
        """Toggle class-name text on the annotations."""
        if flip:
            self.show_labels_var.set(not self.show_labels_var.get())
        self.show_labels = self.show_labels_var.get()
        self.update_display()

    def _on_size_change(self, value):
        """Handle the images-per-page slider."""
        self.set_page_step(int(round(float(value))))

    def change_thumb_size(self, delta: int):
        """`+` enlarges thumbnails, which means fewer of them per page."""
        self.set_page_step(self.page_step - delta, sync_scale=True)

    def set_page_step(self, step: int, sync_scale: bool = False):
        step = min(max(step, 0), len(self.PAGE_STEPS) - 1)
        if step == self.page_step:
            return
        self.page_step = step
        if sync_scale:
            self.size_scale.set(step)
        self.relayout()

    def next_page(self):
        if self.page_start + self.page_size < len(self.image_annotation_pairs):
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
        self.selected_index = len(self.image_annotation_pairs) - 1
        self.page_start = (self.selected_index // self.page_size) * self.page_size
        self.update_display()

    def jump_to_frame(self):
        """Jump to a specific frame number."""
        try:
            frame_num = int(self.jump_entry.get())
            if 1 <= frame_num <= len(self.image_annotation_pairs):
                self.selected_index = frame_num - 1
                self.page_start = (self.selected_index // self.page_size) * self.page_size
                self.update_display()
            else:
                messagebox.showerror(
                    "Error",
                    f"Frame number must be between 1 and {len(self.image_annotation_pairs)}",
                )
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid frame number")

    def jump_to_next_unvalidated(self):
        """Select the next image with no verdict, wrapping around."""
        total = len(self.image_annotation_pairs)
        order = list(range(self.selected_index, total)) + list(range(0, self.selected_index))
        for index in order:
            if self.key_for(index) not in self.validation_state:
                self.selected_index = index
                self.page_start = (index // self.page_size) * self.page_size
                self.update_display()
                return

        messagebox.showinfo("Complete", "All images have been validated!")
        self.update_display()

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
        image = self.compose(self.selected_index, (max_w, max_h), detail=True)

        self.zoom_photo = ImageTk.PhotoImage(image)
        if self.zoom_label is not None:
            self.zoom_label.config(image=self.zoom_photo)

        status = self.status_of(self.selected_index)
        glyph = {"pass": "✓ PASS", "fail": "✗ FAIL"}.get(status, "○ unvalidated")
        name = self.image_annotation_pairs[self.selected_index][0].name
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


def find_class_yaml(dataset_path: Path) -> Optional[Path]:
    """Locate the data yaml describing the dataset's classes.

    The root is checked first, then the whole tree, because a tree of per-scene exports keeps one
    yaml inside each export rather than a single one at the top. Those per-scene yamls only agree
    when the scenes share a vocabulary; when they do not, a single class list would mislabel
    everything drawn from the other scenes, so say so instead of picking one silently.
    """
    candidates = sorted(dataset_path.glob("*.y*ml")) or sorted(dataset_path.rglob("*.y*ml"))
    candidates = [p for p in candidates if p.suffix in (".yml", ".yaml")]
    if not candidates:
        return None

    vocabularies = set()
    for path in candidates:
        try:
            with open(path) as f:
                names = yaml.safe_load(f).get("names")
        except (OSError, ValueError, AttributeError, yaml.YAMLError):
            continue
        if names:
            vocabularies.add(tuple(names) if isinstance(names, list) else tuple(sorted(names)))

    if len(vocabularies) > 1:
        print(
            f"WARNING: {len(candidates)} data yamls under {dataset_path} declare "
            f"{len(vocabularies)} different class lists. Class names and colors are read from "
            f"{candidates[0]} and will be wrong for the scenes using another vocabulary. "
            "Remap them to a shared vocabulary first, or pass -c explicitly."
        )
    return candidates[0]


def main():
    parser = argparse.ArgumentParser(description="YOLO Dataset Validation UI")
    parser.add_argument("dataset_path", nargs="?", help="Path to YOLO dataset root directory")
    parser.add_argument("-c", "--classes", type=str, help="Path to class label info JSON file.")
    args = parser.parse_args()

    if args.dataset_path:
        dataset_path_str = args.dataset_path
    else:
        # Ask user to select directory
        root = tk.Tk()
        root.withdraw()
        dataset_path_str = filedialog.askdirectory(title="Select YOLO Dataset Directory")
        root.destroy()

        if not dataset_path_str:
            print("No dataset selected. Exiting.")
            return

    dataset_path = Path(dataset_path_str)

    if args.classes:
        classes_path = Path(args.classes)
    else:
        classes_path = find_class_yaml(dataset_path)

    if classes_path is None:
        raise ValueError(f"Failed to find a YOLO data yaml under {dataset_path}")

    validator = YOLODatasetValidator(dataset_path, classes_path)
    validator.run()


if __name__ == "__main__":
    main()
