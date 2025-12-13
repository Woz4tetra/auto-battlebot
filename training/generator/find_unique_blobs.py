import cv2
import numpy as np

from dataclasses import dataclass
from pathlib import Path

ColorRgbUint8 = tuple[int, int, int]


@dataclass
class Blob:
    contour: list[tuple[int, int]]
    color: ColorRgbUint8


def find_unique_blobs(image: np.ndarray) -> list[Blob]:
    nonzero_mask = (cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) > 0).astype(np.uint8) * 255
    num_labels, labels = cv2.connectedComponents(nonzero_mask, connectivity=8)

    blobs = []
    # Skip label 0 (background)
    for label_id in range(1, num_labels):
        # mask the pixels that make up the blob
        blob_mask = (labels == label_id).astype(np.uint8) * 255

        # Find contours for this blob
        contours, _ = cv2.findContours(
            blob_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            continue

        # Use the largest contour
        contour = max(contours, key=cv2.contourArea)

        # Convert contour to list of (x, y) tuples
        contour_points = [(int(pt[0][0]), int(pt[0][1])) for pt in contour]

        # Get the color of the blob (mean color inside the mask)
        mask = labels == label_id
        if np.any(mask):
            mean_color = tuple([int(np.mean(image[:, :, c][mask])) for c in range(3)])[::-1]
        else:
            mean_color = (0, 0, 0)

        blobs.append(Blob(contour=contour_points, color=mean_color))

    return blobs


def draw_blobs(image: np.ndarray, blobs: list[Blob], labels: list[str] = None) -> None:
    for idx, blob in enumerate(blobs):
        # Convert contour points to the format expected by cv2
        contour_np = np.array(blob.contour, dtype=np.int32).reshape((-1, 1, 2))
        # Draw filled contour
        cv2.drawContours(image, [contour_np], -1, blob.color, thickness=5)
        # Draw label (index or provided label) at the center of the bounding box
        xs = [pt[0] for pt in blob.contour]
        ys = [pt[1] for pt in blob.contour]
        x_center = int((min(xs) + max(xs)) / 2)
        y_center = int((min(ys) + max(ys)) / 2)
        label = str(labels[idx]) if labels and idx < len(labels) else str(idx)
        cv2.putText(
            image,
            label,
            (x_center, y_center),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )


def load_image_render(image_path: Path) -> np.ndarray:
    if image_path.suffix.lower() == ".png":
        image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
        alpha = image[..., 3] < 30
        image = (image[..., :3].astype(np.float32) * 255.0 / 65535.0).astype(np.uint8)
        image[alpha] = 0
    else:
        image = cv2.imread(image_path)
    return image


if __name__ == "__main__":
    import argparse

    def main() -> None:
        parser = argparse.ArgumentParser("find_unique_blobs")
        parser.add_argument("image", type=Path)
        parser.add_argument("-b", "--base", type=Path)
        args = parser.parse_args()

        image_path: Path = args.image
        if not image_path.exists():
            raise FileNotFoundError(f"{image_path} doesn't exist")

        base_path: Path | None = args.base
        if base_path is not None and not base_path.exists():
            raise FileNotFoundError(f"{base_path} doesn't exist")

        image = load_image_render(image_path)

        blobs = find_unique_blobs(image)
        if base_path is not None:
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            nonzero_mask = gray_image > 0
            draw_image = cv2.imread(base_path)
            draw_image[nonzero_mask] = image[nonzero_mask]
        else:
            draw_image = image
        draw_blobs(draw_image, blobs)

        print("Index: R, G, B")
        for index, blob in enumerate(blobs):
            blob_color_str = ", ".join([str(ch) for ch in blob.color])
            print(f"{index}: {blob_color_str}")

        cv2.imshow("Blobs", draw_image)
        cv2.waitKey(-1)

    main()
