import cv2
import numpy as np


from pathlib import Path


def load_image_render(image_path: Path) -> np.ndarray:
    if image_path.suffix.lower() == ".png":
        image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
        alpha = image[..., 3] < 30
        image = (image[..., :3].astype(np.float32) * 255.0 / 65535.0).astype(np.uint8)
        image[alpha] = 0
    else:
        image = cv2.imread(image_path)
    return image