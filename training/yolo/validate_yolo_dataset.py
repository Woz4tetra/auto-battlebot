#!/usr/bin/env python3
"""
YOLO Dataset Validation UI

This script provides a GUI for manually validating YOLO dataset annotations.
Users can review images with their bounding boxes and labels, marking them as
pass or fail. The validation state is saved and can be resumed later.
"""

import re
import yaml
import json
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from PIL import Image, ImageDraw, ImageFont, ImageTk
from pathlib import Path
from typing import List, Dict, Tuple
import argparse
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


class YOLODatasetValidator:
    def __init__(self, dataset_path: Path, class_labels_path: Path):
        self.dataset_path = dataset_path
        self.state_file = self.dataset_path / "validation_state.json"
        self.class_labels_path = class_labels_path

        # Data structures
        self.image_annotation_pairs: List[Tuple[Path, Path]] = []
        self.validation_state: Dict[str, str] = {}  # path -> 'pass'/'fail'/None
        self.current_index = 0
        self.class_info: Dict[int, Dict] = {}  # class_id -> {name, color}
        self.show_labels = True

        # UI components
        self.root = tk.Tk()
        self.root.title("YOLO Dataset Validator")
        self.root.geometry("1300x900")

        # Image display
        self.canvas = None
        self.image_label = None
        self.current_photo = None
        self.status_icon_label = None
        self.status_icon_label = None

        # Status
        self.status_var = tk.StringVar()
        self.frame_info_var = tk.StringVar()
        self.show_labels_var = tk.BooleanVar(value=True)

        # Colors for bounding boxes
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

        # Feedback overlay
        self.showing_feedback = False

        # Initialize
        self.load_dataset()
        self.load_class_info(self.class_labels_path)
        self.load_state()
        self.setup_ui()
        self.jump_to_next_unvalidated()

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
        """Load class information from JSON file if present."""
        # Look for common class info file names

        class_labels_path = Path(class_labels_path)
        if not class_labels_path.exists():
            print("No class info file found, using default colors and names")
            return

        print(f"Loading class information from {class_labels_path}")
        try:
            with open(class_labels_path, "r") as f:
                data = yaml.safe_load(f)

            # Parse class info
            for class_id, (class_name, color_hex) in enumerate(
                zip(data["names"], data["colors"])
            ):
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
        """Save validation state to JSON file."""
        with open(self.state_file, "w") as f:
            json.dump(self.validation_state, f, indent=2)

    def setup_ui(self):
        """Create the user interface."""
        # Main container with image on left and sidebar on right
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

        # Navigation buttons row
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

        # Next unvalidated button (separate)
        ttk.Button(
            nav_section,
            text="⏩ Next Unvalidated (U)",
            command=self.jump_to_next_unvalidated,
            width=15,
        ).pack(fill=tk.X, pady=5)

        # Jump to frame subsection
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

        show_labels_check = ttk.Checkbutton(
            options_section,
            text="Show Labels",
            variable=self.show_labels_var,
            command=self.toggle_labels,
        )
        show_labels_check.pack(anchor=tk.W, pady=5)

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

        # Update display
        self.update_display()

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

                # Parse standard YOLO detect/pose rows first:
                # class cx cy w h [kpt_x kpt_y kpt_v]...
                if len(values) >= 4:
                    kp_values = values[4:]
                    if len(kp_values) % 3 == 0:
                        parsed_keypoints = []
                        keypoints_valid = True
                        for i in range(0, len(kp_values), 3):
                            v_raw = kp_values[i + 2]
                            v_int = int(round(v_raw))
                            if abs(v_raw - v_int) > 1e-6 or v_int not in (0, 1, 2):
                                keypoints_valid = False
                                break
                            parsed_keypoints.append(
                                {"x": kp_values[i], "y": kp_values[i + 1], "v": v_int}
                            )

                        if keypoints_valid:
                            annotations.append(
                                {
                                    "type": "bbox",
                                    "class_id": class_id,
                                    "center_x": values[0],
                                    "center_y": values[1],
                                    "width": values[2],
                                    "height": values[3],
                                    "keypoints": parsed_keypoints,
                                }
                            )
                            continue

                # Parse YOLO-seg rows:
                # class x1 y1 x2 y2 ... xn yn
                if len(values) < 6 or len(values) % 2 != 0:
                    continue

                polygon = []
                for i in range(0, len(values), 2):
                    polygon.append({"x": values[i], "y": values[i + 1]})

                if len(polygon) < 3:
                    continue

                xs = [p["x"] for p in polygon]
                ys = [p["y"] for p in polygon]
                min_x, max_x = min(xs), max(xs)
                min_y, max_y = min(ys), max(ys)

                annotations.append(
                    {
                        "type": "seg",
                        "class_id": class_id,
                        "center_x": (min_x + max_x) / 2.0,
                        "center_y": (min_y + max_y) / 2.0,
                        "width": max_x - min_x,
                        "height": max_y - min_y,
                        "polygon": polygon,
                        "keypoints": [],
                    }
                )
        return annotations

    def draw_image_with_annotations(
        self, img_path: Path, label_path: Path
    ) -> Image.Image:
        """Draw parsed annotations (bbox/pose and seg polygons) on image."""
        # Load image
        image = Image.open(img_path).convert("RGBA")
        draw = ImageDraw.Draw(image)
        seg_overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
        seg_overlay_draw = ImageDraw.Draw(seg_overlay, "RGBA")

        # Get image dimensions
        img_width, img_height = image.size

        # Parse annotations
        annotations = self.parse_yolo_annotation(label_path)

        font = ImageFont.load_default(size=20)

        # Draw each annotation
        for ann in annotations:
            class_id = ann["class_id"]
            cx = ann["center_x"] * img_width
            cy = ann["center_y"] * img_height
            w = ann["width"] * img_width
            h = ann["height"] * img_height

            # Calculate bounding box corners
            x1 = cx - w / 2
            y1 = cy - h / 2
            x2 = cx + w / 2
            y2 = cy + h / 2

            # Get class name and color from class_info if available
            if class_id in self.class_info:
                class_name = self.class_info[class_id]["name"]
                color = self.color_name_to_hex(self.class_info[class_id]["color"])
            else:
                class_name = f"Class {class_id}"
                color = self.colors[class_id % len(self.colors)]

            if ann.get("type") == "seg":
                polygon_points = [
                    (p["x"] * img_width, p["y"] * img_height)
                    for p in ann.get("polygon", [])
                ]
                if len(polygon_points) >= 3:
                    color_str = color.lstrip("#")
                    try:
                        color_rgb = tuple(
                            int(color_str[i : i + 2], 16) for i in (0, 2, 4)
                        )
                    except ValueError:
                        color_rgb = (255, 0, 0)
                    seg_overlay_draw.polygon(
                        polygon_points,
                        fill=(color_rgb[0], color_rgb[1], color_rgb[2], 35),
                    )
                    draw.line(
                        polygon_points + [polygon_points[0]],
                        fill=color,
                        width=3,
                    )
            else:
                # Always draw bounding box for detect/pose annotations
                draw.rectangle([x1, y1, x2, y2], outline=color, width=3)

            # Conditionally draw label text
            if self.show_labels:
                label_text = class_name

                # Draw label background
                bbox = draw.textbbox((x1, y1 - 25), label_text, font=font)
                draw.rectangle(bbox, fill=color)
                draw.text((x1, y1 - 25), label_text, fill="white", font=font)

            # Draw keypoints
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
            r = max(4, int(min(img_width, img_height) * 0.008))
            for kp_idx, kp in enumerate(ann.get("keypoints", [])):
                if kp["v"] == 0:
                    continue
                kx = kp["x"] * img_width
                ky = kp["y"] * img_height
                kp_color = kp_colors[kp_idx % len(kp_colors)]
                # Filled circle with dark outline
                draw.ellipse(
                    [kx - r - 1, ky - r - 1, kx + r + 1, ky + r + 1], fill="#000000"
                )
                draw.ellipse([kx - r, ky - r, kx + r, ky + r], fill=kp_color)
                if self.show_labels:
                    kp_label = str(kp_idx)
                    kp_bbox = draw.textbbox((kx + r + 2, ky - r), kp_label, font=font)
                    draw.rectangle(
                        kp_bbox,
                        fill="#000000AA" if hasattr(draw, "alpha") else "#000000",
                    )
                    draw.text((kx + r + 2, ky - r), kp_label, fill=kp_color, font=font)

        image = Image.alpha_composite(image, seg_overlay)
        return image.convert("RGB")

    def draw_feedback_overlay(self, image: Image.Image, passed: bool) -> Image.Image:
        """Draw checkmark or X overlay on image."""
        img_width, img_height = image.size

        # Semi-transparent overlay
        overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
        overlay_draw = ImageDraw.Draw(overlay)

        if passed:
            # Green checkmark
            color = (0, 255, 0, 180)
            # Draw checkmark
            cx, cy = img_width // 2, img_height // 2
            size = min(img_width, img_height) // 4
            points = [
                (cx - size // 2, cy),
                (cx - size // 6, cy + size // 2),
                (cx + size // 2, cy - size // 2),
            ]
            overlay_draw.line(points, fill=color, width=20)
        else:
            # Red X
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

        # Composite
        result = Image.alpha_composite(image.convert("RGBA"), overlay)
        return result.convert("RGB")

    def update_display(self):
        """Update the display with current image."""
        if not self.image_annotation_pairs:
            return

        if self.current_index < 0:
            self.current_index = 0
        elif self.current_index >= len(self.image_annotation_pairs):
            self.current_index = len(self.image_annotation_pairs) - 1

        img_path, label_path = self.image_annotation_pairs[self.current_index]

        # Draw image with annotations
        image = self.draw_image_with_annotations(img_path, label_path)

        # Resize to fit window
        display_width = 1100
        display_height = 700
        image.thumbnail((display_width, display_height), Image.Resampling.LANCZOS)

        # Convert to PhotoImage
        self.current_photo = ImageTk.PhotoImage(image)
        self.image_label.config(image=self.current_photo)

        # Update info
        img_key = str(img_path.relative_to(self.dataset_path))
        status = self.validation_state.get(img_key, "unvalidated")

        validated_count = sum(
            1 for v in self.validation_state.values() if v in ["pass", "fail"]
        )
        pass_count = sum(1 for v in self.validation_state.values() if v == "pass")
        fail_count = sum(1 for v in self.validation_state.values() if v == "fail")

        self.frame_info_var.set(
            f"Frame {self.current_index + 1} / {len(self.image_annotation_pairs)}"
        )

        # Update status icon
        if status == "pass":
            self.status_icon_label.config(text="✓", fg="#00AA00", bg="#E8F5E9")
        elif status == "fail":
            self.status_icon_label.config(text="✗", fg="#CC0000", bg="#FFEBEE")
        else:  # unvalidated
            self.status_icon_label.config(text="○", fg="#888888", bg="#F5F5F5")

        self.status_var.set(
            f"Dataset: {self.dataset_path.name} | "
            f"Validated: {validated_count} | Pass: {pass_count} | Fail: {fail_count} | "
            f"File: {img_path.name}"
        )

    def validate(self, result: str):
        """Mark current image as pass or fail."""
        if not self.image_annotation_pairs:
            return

        img_path, label_path = self.image_annotation_pairs[self.current_index]
        img_key = str(img_path.relative_to(self.dataset_path))

        # Update state
        self.validation_state[img_key] = result
        self.save_state()

        # Show feedback
        # self.show_feedback(result == "pass")

        # Auto-advance to next unvalidated
        # self.root.after(250, self.next_image)
        self.next_image()

    def toggle_labels(self):
        """Toggle label text visibility."""
        self.show_labels = self.show_labels_var.get()
        self.update_display()

    def show_feedback(self, passed: bool):
        """Show checkmark or X overlay briefly."""
        if not self.image_annotation_pairs:
            return

        img_path, label_path = self.image_annotation_pairs[self.current_index]

        # Draw image with annotations and feedback
        image = self.draw_image_with_annotations(img_path, label_path)

        # Resize to fit window
        display_width = 1100
        display_height = 700
        image.thumbnail((display_width, display_height), Image.Resampling.LANCZOS)

        # Add feedback overlay
        image = self.draw_feedback_overlay(image, passed)

        # Convert to PhotoImage
        feedback_photo = ImageTk.PhotoImage(image)
        self.image_label.config(image=feedback_photo)

        # Keep reference to prevent garbage collection
        self.image_label.feedback_photo = feedback_photo

    def next_image(self):
        """Navigate to next image."""
        self.current_index += 1
        if self.current_index >= len(self.image_annotation_pairs):
            self.current_index = len(self.image_annotation_pairs) - 1
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
        self.current_index = len(self.image_annotation_pairs) - 1
        self.update_display()

    def jump_to_frame(self):
        """Jump to specific frame number."""
        try:
            frame_num = int(self.jump_entry.get())
            if 1 <= frame_num <= len(self.image_annotation_pairs):
                self.current_index = frame_num - 1
                self.update_display()
            else:
                messagebox.showerror(
                    "Error",
                    f"Frame number must be between 1 and {len(self.image_annotation_pairs)}",
                )
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid frame number")

    def jump_to_next_unvalidated(self):
        """Jump to next unvalidated image."""
        start_index = self.current_index

        # Search forward from current position
        for i in range(self.current_index, len(self.image_annotation_pairs)):
            img_path, _ = self.image_annotation_pairs[i]
            img_key = str(img_path.relative_to(self.dataset_path))
            if img_key not in self.validation_state:
                self.current_index = i
                self.update_display()
                return

        # Wrap around and search from beginning
        for i in range(0, start_index):
            img_path, _ = self.image_annotation_pairs[i]
            img_key = str(img_path.relative_to(self.dataset_path))
            if img_key not in self.validation_state:
                self.current_index = i
                self.update_display()
                return

        # All validated
        messagebox.showinfo("Complete", "All images have been validated!")
        self.update_display()

    def run(self):
        """Start the UI main loop."""
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(description="YOLO Dataset Validation UI")
    parser.add_argument(
        "dataset_path", nargs="?", help="Path to YOLO dataset root directory"
    )
    parser.add_argument(
        "-c", "--classes", type=str, help="Path to class label info JSON file."
    )
    args = parser.parse_args()

    if args.dataset_path:
        dataset_path_str = args.dataset_path
    else:
        # Ask user to select directory
        root = tk.Tk()
        root.withdraw()
        dataset_path_str = filedialog.askdirectory(
            title="Select YOLO Dataset Directory"
        )
        root.destroy()

        if not dataset_path_str:
            print("No dataset selected. Exiting.")
            return

    dataset_path = Path(dataset_path_str)

    classes_path_str = args.classes

    if not classes_path_str:
        classes_path = None
        for path in dataset_path.iterdir():
            match = re.search(r".*(ya?ml)", str(path))
            if not match:
                continue
            classes_path = dataset_path / path.name
    else:
        classes_path = Path(classes_path_str)

    if classes_path is None:
        raise ValueError("Failed to find YOLO data file")

    validator = YOLODatasetValidator(dataset_path, classes_path)
    validator.run()


if __name__ == "__main__":
    main()
