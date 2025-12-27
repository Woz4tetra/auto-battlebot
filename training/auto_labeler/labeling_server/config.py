"""Server configuration management."""

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Union

import yaml


# Named colors mapping (name -> RGB)
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


def parse_color(color_value: Union[str, List[int]]) -> List[int]:
    """Parse a color value which can be either a named color string or RGB list."""
    if isinstance(color_value, str):
        color_lower = color_value.lower().strip()
        if color_lower in NAMED_COLORS:
            return NAMED_COLORS[color_lower]
        else:
            raise ValueError(
                f"Unknown color name: '{color_value}'. "
                f"Available colors: {', '.join(sorted(NAMED_COLORS.keys()))}"
            )
    elif isinstance(color_value, list) and len(color_value) == 3:
        return [int(c) for c in color_value]
    else:
        raise ValueError(
            f"Invalid color format: {color_value}. "
            "Use a color name string or [R, G, B] list."
        )


@dataclass
class ObjectLabel:
    """Configuration for a tracked object label."""

    id: int
    name: str
    color: List[int] = field(default_factory=lambda: [255, 0, 0])  # RGB


@dataclass
class ServerConfig:
    """Server configuration loaded from YAML file."""

    # Video settings
    video_path: str

    # Object labels to track
    object_labels: List[ObjectLabel]

    # Output paths
    manual_annotations_dir: str
    generated_annotations_dir: str

    # Propagation settings
    propagate_length: int = 300  # Number of frames to propagate forward

    # Server settings
    host: str = "0.0.0.0"
    port: int = 7547

    # GPU settings
    gpu_id: int = 0

    # Mask overlay settings
    mask_alpha: float = 0.5

    # Inference width - target width in pixels for SAM3 inference
    # Height is calculated automatically to maintain aspect ratio
    # Masks are scaled back to original size after inference
    inference_width: int = 960

    @classmethod
    def from_yaml(cls, config_path: str) -> "ServerConfig":
        """Load configuration from YAML file."""
        with open(config_path, "r") as f:
            data = yaml.safe_load(f)

        # Parse object labels
        object_labels = []
        for label_data in data.get("object_labels", []):
            color_value = label_data.get("color", [255, 0, 0])
            label = ObjectLabel(
                id=label_data["id"],
                name=label_data["name"],
                color=parse_color(color_value),
            )
            object_labels.append(label)

        # Ensure directories exist
        manual_dir = data.get("manual_annotations_dir", "./manual_annotations")
        generated_dir = data.get("generated_annotations_dir", "./generated_annotations")

        return cls(
            video_path=data["video_path"],
            object_labels=object_labels,
            manual_annotations_dir=manual_dir,
            generated_annotations_dir=generated_dir,
            propagate_length=data.get("propagate_length", 300),
            host=data.get("host", "0.0.0.0"),
            port=data.get("port", 7547),
            gpu_id=data.get("gpu_id", 0),
            mask_alpha=data.get("mask_alpha", 0.5),
            inference_width=data.get("inference_width", 960),
        )

    def ensure_directories(self):
        """Create output directories if they don't exist."""
        Path(self.manual_annotations_dir).mkdir(parents=True, exist_ok=True)
        Path(self.generated_annotations_dir).mkdir(parents=True, exist_ok=True)

        # Create subdirectories for images and annotations
        Path(self.manual_annotations_dir, "images").mkdir(exist_ok=True)
        Path(self.manual_annotations_dir, "annotations").mkdir(exist_ok=True)
        Path(self.generated_annotations_dir, "images").mkdir(exist_ok=True)
        Path(self.generated_annotations_dir, "annotations").mkdir(exist_ok=True)
        Path(self.generated_annotations_dir, "masks").mkdir(exist_ok=True)

    def get_label_by_id(self, label_id: int) -> Optional[ObjectLabel]:
        """Get object label by ID."""
        for label in self.object_labels:
            if label.id == label_id:
                return label
        return None

    def get_label_by_name(self, name: str) -> Optional[ObjectLabel]:
        """Get object label by name."""
        for label in self.object_labels:
            if label.name == name:
                return label
        return None

    def to_dict(self) -> Dict:
        """Convert config to dictionary for API responses."""
        return {
            "video_path": self.video_path,
            "object_labels": [
                {"id": l.id, "name": l.name, "color": l.color}
                for l in self.object_labels
            ],
            "propagate_length": self.propagate_length,
            "mask_alpha": self.mask_alpha,
            "inference_width": self.inference_width,
        }
