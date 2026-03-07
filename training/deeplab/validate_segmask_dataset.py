#!/usr/bin/env python3
"""
Segmentation Mask Dataset Validation UI

This script provides a GUI for manually validating segmentation mask dataset
annotations. Users can review images with their mask overlays and class
legends, marking them as pass or fail. The validation state is saved and
can be resumed later.

Mask convention: each image foo.jpg has a corresponding foo_mask.png where
pixel values are integer class IDs (same format used by semantic_train.py).
"""

import re
import yaml
import json
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from PIL import Image, ImageDraw, ImageFont, ImageTk
from pathlib import Path
from typing import List, Dict, Tuple, Optional
import argparse
import numpy as np
from natsort import natsorted

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
    def __init__(self, dataset_path: Path, class_labels_path: Optional[Path]):
        self.dataset_path = dataset_path
        self.state_file = self.dataset_path / "validation_state.json"
        self.class_labels_path = class_labels_path

        # Data structures
        self.image_mask_pairs: List[Tuple[Path, Path]] = []
        self.validation_state: Dict[str, str] = {}  # path -> 'pass'/'fail'
        self.current_index = 0
        self.class_info: Dict[int, Dict] = {}  # class_id -> {name, color}
        self.show_overlay = True
        self.overlay_alpha = 0.45

        # UI components
        self.root = tk.Tk()
        self.root.title("Segmentation Mask Dataset Validator")
        self.root.geometry("1300x900")

        self.image_label = None
        self.current_photo = None
        self.status_icon_label = None

        # Status
        self.status_var = tk.StringVar()
        self.frame_info_var = tk.StringVar()
        self.show_overlay_var = tk.BooleanVar(value=True)

        # Feedback overlay
        self.showing_feedback = False

        # Initialize
        self.load_dataset()
        self.load_class_info(self.class_labels_path)
        self.load_state()
        self.setup_ui()
        self.jump_to_next_unvalidated()

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

            for class_id, (class_name, color_hex) in enumerate(
                zip(data["names"], data["colors"])
            ):
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
        with open(self.state_file, "w") as f:
            json.dump(self.validation_state, f, indent=2)

    def setup_ui(self):
        """Create the user interface."""
        main_container = ttk.Frame(self.root)
        main_container.pack(fill=tk.BOTH, expand=True)

        # Image display area (left side)
        image_frame = ttk.Frame(main_container)
        image_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.image_label = ttk.Label(image_frame)
        self.image_label.pack(fill=tk.BOTH, expand=True)

        # Sidebar (right side)
        sidebar = ttk.Frame(main_container, relief=tk.RIDGE, borderwidth=2)
        sidebar.pack(side=tk.RIGHT, fill=tk.Y, padx=(0, 10), pady=10)

        # === STATUS SECTION ===
        status_section = ttk.LabelFrame(sidebar, text="Validation Status", padding="15")
        status_section.pack(fill=tk.X, padx=10, pady=(10, 5))

        self.status_icon_label = tk.Label(
            status_section,
            text="○",
            font=("Arial", 64, "bold"),
            bg="#F5F5F5",
            fg="#888888",
            width=3,
            height=1,
        )
        self.status_icon_label.pack(pady=10)

        ttk.Label(
            status_section,
            textvariable=self.frame_info_var,
            font=("Arial", 11, "bold"),
            anchor=tk.CENTER,
        ).pack(pady=5)

        # === VALIDATION BUTTONS SECTION ===
        validation_section = ttk.LabelFrame(sidebar, text="Validate", padding="15")
        validation_section.pack(fill=tk.X, padx=10, pady=5)

        pass_btn = ttk.Button(
            validation_section,
            text="✓ PASS\n(Y)",
            command=lambda: self.validate("pass"),
            width=15,
        )
        pass_btn.pack(fill=tk.X, pady=5)

        fail_btn = ttk.Button(
            validation_section,
            text="✗ FAIL\n(N)",
            command=lambda: self.validate("fail"),
            width=15,
        )
        fail_btn.pack(fill=tk.X, pady=5)

        # === NAVIGATION SECTION ===
        nav_section = ttk.LabelFrame(sidebar, text="Navigation", padding="15")
        nav_section.pack(fill=tk.X, padx=10, pady=5)

        nav_buttons_frame = ttk.Frame(nav_section)
        nav_buttons_frame.pack(fill=tk.X, pady=(0, 5))

        ttk.Button(
            nav_buttons_frame, text="⏮", command=self.jump_to_start, width=4
        ).pack(side=tk.LEFT, padx=1, expand=True, fill=tk.X)
        ttk.Button(
            nav_buttons_frame, text="◀", command=self.previous_image, width=4
        ).pack(side=tk.LEFT, padx=1, expand=True, fill=tk.X)
        ttk.Button(nav_buttons_frame, text="▶", command=self.next_image, width=4).pack(
            side=tk.LEFT, padx=1, expand=True, fill=tk.X
        )
        ttk.Button(nav_buttons_frame, text="⏭", command=self.jump_to_end, width=4).pack(
            side=tk.LEFT, padx=1, expand=True, fill=tk.X
        )

        ttk.Button(
            nav_section,
            text="⏩ Next Unvalidated (U)",
            command=self.jump_to_next_unvalidated,
            width=15,
        ).pack(fill=tk.X, pady=5)

        ttk.Separator(nav_section, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        ttk.Label(nav_section, text="Jump to Frame:").pack(anchor=tk.W, pady=(5, 2))

        jump_frame = ttk.Frame(nav_section)
        jump_frame.pack(fill=tk.X, pady=2)

        self.jump_entry = ttk.Entry(jump_frame, width=10)
        self.jump_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        ttk.Button(jump_frame, text="Go", command=self.jump_to_frame, width=5).pack(
            side=tk.RIGHT
        )

        # === DISPLAY OPTIONS SECTION ===
        options_section = ttk.LabelFrame(sidebar, text="Display Options", padding="15")
        options_section.pack(fill=tk.X, padx=10, pady=5)

        ttk.Checkbutton(
            options_section,
            text="Show Overlay",
            variable=self.show_overlay_var,
            command=self.toggle_overlay,
        ).pack(anchor=tk.W, pady=5)

        ttk.Label(options_section, text="Overlay Alpha:").pack(anchor=tk.W)
        self.alpha_scale = ttk.Scale(
            options_section,
            from_=0.0,
            to=1.0,
            orient=tk.HORIZONTAL,
            value=self.overlay_alpha,
            command=self._on_alpha_change,
        )
        self.alpha_scale.pack(fill=tk.X, pady=(0, 5))

        # === KEYBOARD SHORTCUTS SECTION ===
        shortcuts_section = ttk.LabelFrame(
            sidebar, text="Keyboard Shortcuts", padding="15"
        )
        shortcuts_section.pack(fill=tk.X, padx=10, pady=5)

        shortcuts_text = (
            "Y - Pass\n"
            "N - Fail\n"
            "U - Next Unvalidated\n"
            "← - Previous\n"
            "→ - Next\n"
            "Space - Next Unvalidated\n"
            "Home - Start\n"
            "End - End"
        )
        ttk.Label(
            shortcuts_section, text=shortcuts_text, font=("Courier", 9), justify=tk.LEFT
        ).pack(anchor=tk.W)

        # Status bar at bottom
        status_frame = ttk.Frame(self.root)
        status_frame.pack(side=tk.BOTTOM, fill=tk.X)
        ttk.Label(status_frame, textvariable=self.status_var, relief=tk.SUNKEN).pack(
            fill=tk.X
        )

        # Keyboard bindings
        self.root.bind("y", lambda e: self.validate("pass"))
        self.root.bind("n", lambda e: self.validate("fail"))
        self.root.bind("<Left>", lambda e: self.previous_image())
        self.root.bind("a", lambda e: self.previous_image())
        self.root.bind("<Right>", lambda e: self.next_image())
        self.root.bind("d", lambda e: self.next_image())
        self.root.bind("u", lambda e: self.jump_to_next_unvalidated())
        self.root.bind("<Home>", lambda e: self.jump_to_start())
        self.root.bind("<End>", lambda e: self.jump_to_end())
        self.root.bind("<space>", lambda e: self.jump_to_next_unvalidated())

        self.update_display()

    def _on_alpha_change(self, value):
        """Handle alpha slider change."""
        self.overlay_alpha = float(value)
        self.update_display()

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

    def draw_image_with_mask(self, img_path: Path, mask_path: Path) -> Image.Image:
        """Composite the image with a coloured, semi-transparent mask overlay."""
        image = Image.open(img_path).convert("RGB")
        img_w, img_h = image.size

        mask_img = Image.open(mask_path)
        # If the mask is multi-channel, take the first channel
        if mask_img.mode != "L":
            mask_arr = np.array(mask_img)
            if mask_arr.ndim == 3:
                mask_arr = mask_arr[:, :, 0]
            mask_img = Image.fromarray(mask_arr.astype(np.uint8), mode="L")

        # Resize mask to match image if needed
        if mask_img.size != image.size:
            mask_img = mask_img.resize((img_w, img_h), Image.Resampling.NEAREST)

        mask_arr = np.array(mask_img)
        unique_ids = np.unique(mask_arr)

        if self.show_overlay:
            # Build a coloured RGBA overlay
            overlay_arr = np.zeros((img_h, img_w, 4), dtype=np.uint8)
            for class_id in unique_ids:
                r, g, b = self.get_class_color(int(class_id))
                alpha_val = int(self.overlay_alpha * 255)
                pixels = mask_arr == class_id
                overlay_arr[pixels, 0] = r
                overlay_arr[pixels, 1] = g
                overlay_arr[pixels, 2] = b
                overlay_arr[pixels, 3] = alpha_val

            overlay = Image.fromarray(overlay_arr, mode="RGBA")
            result = Image.alpha_composite(image.convert("RGBA"), overlay).convert(
                "RGB"
            )
        else:
            result = image

        # Draw legend for classes present in this mask
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

            # Swatch background
            draw.rectangle(
                [legend_x, legend_y, legend_x + swatch, legend_y + swatch],
                fill=hex_color,
                outline="#000000",
            )
            text_x = legend_x + swatch + pad
            text_bbox = draw.textbbox((text_x, legend_y), name, font=font)
            draw.rectangle(text_bbox, fill="#000000CC" if False else "#000000")
            draw.text((text_x, legend_y), name, fill="white", font=font)
            legend_y += swatch + pad

        return result

    def draw_feedback_overlay(self, image: Image.Image, passed: bool) -> Image.Image:
        """Draw checkmark or X overlay on image."""
        img_width, img_height = image.size

        overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
        overlay_draw = ImageDraw.Draw(overlay)

        if passed:
            color = (0, 255, 0, 180)
            cx, cy = img_width // 2, img_height // 2
            size = min(img_width, img_height) // 4
            points = [
                (cx - size // 2, cy),
                (cx - size // 6, cy + size // 2),
                (cx + size // 2, cy - size // 2),
            ]
            overlay_draw.line(points, fill=color, width=20)
        else:
            color = (255, 0, 0, 180)
            cx, cy = img_width // 2, img_height // 2
            size = min(img_width, img_height) // 4
            overlay_draw.line(
                [(cx - size // 2, cy - size // 2), (cx + size // 2, cy + size // 2)],
                fill=color,
                width=20,
            )
            overlay_draw.line(
                [(cx - size // 2, cy + size // 2), (cx + size // 2, cy - size // 2)],
                fill=color,
                width=20,
            )

        result = Image.alpha_composite(image.convert("RGBA"), overlay)
        return result.convert("RGB")

    def update_display(self):
        """Update the display with current image."""
        if not self.image_mask_pairs:
            return

        if self.current_index < 0:
            self.current_index = 0
        elif self.current_index >= len(self.image_mask_pairs):
            self.current_index = len(self.image_mask_pairs) - 1

        img_path, mask_path = self.image_mask_pairs[self.current_index]

        image = self.draw_image_with_mask(img_path, mask_path)

        display_width = 1100
        display_height = 700
        image.thumbnail((display_width, display_height), Image.Resampling.LANCZOS)

        self.current_photo = ImageTk.PhotoImage(image)
        self.image_label.config(image=self.current_photo)

        img_key = str(img_path.relative_to(self.dataset_path))
        status = self.validation_state.get(img_key, "unvalidated")

        validated_count = sum(
            1 for v in self.validation_state.values() if v in ["pass", "fail"]
        )
        pass_count = sum(1 for v in self.validation_state.values() if v == "pass")
        fail_count = sum(1 for v in self.validation_state.values() if v == "fail")

        self.frame_info_var.set(
            f"Frame {self.current_index + 1} / {len(self.image_mask_pairs)}"
        )

        if status == "pass":
            self.status_icon_label.config(text="✓", fg="#00AA00", bg="#E8F5E9")
        elif status == "fail":
            self.status_icon_label.config(text="✗", fg="#CC0000", bg="#FFEBEE")
        else:
            self.status_icon_label.config(text="○", fg="#888888", bg="#F5F5F5")

        self.status_var.set(
            f"Dataset: {self.dataset_path.name} | "
            f"Validated: {validated_count} | Pass: {pass_count} | Fail: {fail_count} | "
            f"File: {img_path.name}"
        )

    def validate(self, result: str):
        """Mark current image as pass or fail."""
        if not self.image_mask_pairs:
            return

        img_path, _ = self.image_mask_pairs[self.current_index]
        img_key = str(img_path.relative_to(self.dataset_path))

        self.validation_state[img_key] = result
        self.save_state()
        self.next_image()

    def toggle_overlay(self):
        """Toggle mask overlay visibility."""
        self.show_overlay = self.show_overlay_var.get()
        self.update_display()

    def next_image(self):
        """Navigate to next image."""
        self.current_index += 1
        if self.current_index >= len(self.image_mask_pairs):
            self.current_index = len(self.image_mask_pairs) - 1
            messagebox.showinfo("End", "Reached end of dataset")
        self.update_display()

    def previous_image(self):
        """Navigate to previous image."""
        self.current_index -= 1
        if self.current_index < 0:
            self.current_index = 0
            messagebox.showinfo("Start", "Already at start of dataset")
        self.update_display()

    def jump_to_start(self):
        """Jump to first image."""
        self.current_index = 0
        self.update_display()

    def jump_to_end(self):
        """Jump to last image."""
        self.current_index = len(self.image_mask_pairs) - 1
        self.update_display()

    def jump_to_frame(self):
        """Jump to specific frame number."""
        try:
            frame_num = int(self.jump_entry.get())
            if 1 <= frame_num <= len(self.image_mask_pairs):
                self.current_index = frame_num - 1
                self.update_display()
            else:
                messagebox.showerror(
                    "Error",
                    f"Frame number must be between 1 and {len(self.image_mask_pairs)}",
                )
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid frame number")

    def jump_to_next_unvalidated(self):
        """Jump to next unvalidated image."""
        start_index = self.current_index

        for i in range(self.current_index, len(self.image_mask_pairs)):
            img_path, _ = self.image_mask_pairs[i]
            img_key = str(img_path.relative_to(self.dataset_path))
            if img_key not in self.validation_state:
                self.current_index = i
                self.update_display()
                return

        for i in range(0, start_index):
            img_path, _ = self.image_mask_pairs[i]
            img_key = str(img_path.relative_to(self.dataset_path))
            if img_key not in self.validation_state:
                self.current_index = i
                self.update_display()
                return

        messagebox.showinfo("Complete", "All images have been validated!")
        self.update_display()

    def run(self):
        """Start the UI main loop."""
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(
        description="Segmentation Mask Dataset Validation UI"
    )
    parser.add_argument(
        "dataset_path", nargs="?", help="Path to segmask dataset root directory"
    )
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
