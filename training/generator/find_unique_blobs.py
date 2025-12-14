import cv2
import numpy as np

from dataclasses import dataclass
from pathlib import Path

from load_image_render import load_image_render

ColorRgbUint8 = tuple[int, int, int]


@dataclass
class Blob:
    contour: list[tuple[int, int]]
    color: ColorRgbUint8


def get_blob_centroid(blob: Blob) -> tuple[float, float]:
    contour_np = np.array(blob.contour, dtype=np.int32).reshape((-1, 1, 2))
    M = cv2.moments(contour_np)
    if M["m00"] == 0:
        return None
    x_centroid = M["m10"] / M["m00"]
    y_centroid = M["m01"] / M["m00"]
    return (x_centroid, y_centroid)


def get_blob_bounding_rectangle(blob: Blob) -> tuple[float, float, float, float]:
    xs = [pt[0] for pt in blob.contour]
    ys = [pt[1] for pt in blob.contour]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    x_center = (x_min + x_max) / 2
    y_center = (y_min + y_max) / 2
    box_w = x_max - x_min
    box_h = y_max - y_min
    return x_center, y_center, box_w, box_h


def find_unique_blobs(
    image: np.ndarray,
    expected_color: ColorRgbUint8 | None = None,
    color_tolerance: float = 10.0,
) -> list[Blob]:
    if expected_color is None:
        nonzero_mask = (cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) > 0).astype(
            np.uint8
        ) * 255
    else:
        color_range = color_tolerance * 255 / 100
        color_array = np.array(expected_color[::-1])
        lower_color = color_array - color_range
        upper_color = color_array + color_range
        nonzero_mask = cv2.inRange(image, lower_color, upper_color)
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
            mean_color = tuple([int(np.mean(image[:, :, c][mask])) for c in range(3)])
        else:
            mean_color = (0, 0, 0)

        blobs.append(Blob(contour=contour_points, color=mean_color[::-1]))

    return blobs


def find_unique_blobs_with_color_priors(
    image: np.ndarray,
    expected_colors: list[ColorRgbUint8],
    color_tolerance: float = 10.0,
) -> list[Blob]:
    blobs = []
    for expected_color in expected_colors:
        blobs += find_unique_blobs(image, expected_color, color_tolerance)
    return blobs


def draw_blobs(image: np.ndarray, blobs: list[Blob], labels: list[str] = None) -> None:
    for idx, blob in enumerate(blobs):
        # Convert contour points to the format expected by cv2
        contour_np = np.array(blob.contour, dtype=np.int32).reshape((-1, 1, 2))
        # Draw filled contour
        cv2.drawContours(image, [contour_np], -1, blob.color, thickness=5)
        # Draw label (index or provided label) at the center of the bounding box
        x_center, y_center = get_blob_centroid(blob)
        label = str(labels[idx]) if labels and idx < len(labels) else str(idx)
        cv2.putText(
            image,
            label,
            (int(x_center), int(y_center)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )


if __name__ == "__main__":
    import argparse

    def main() -> None:
        parser = argparse.ArgumentParser("find_unique_blobs")
        parser.add_argument("image", type=Path)
        parser.add_argument("-b", "--base", type=Path)
        parser.add_argument("-c", "--color", type=str)
        args = parser.parse_args()

        image_path: Path = args.image
        if not image_path.exists():
            raise FileNotFoundError(f"{image_path} doesn't exist")

        base_path: Path | None = args.base
        if base_path is not None and not base_path.exists():
            raise FileNotFoundError(f"{base_path} doesn't exist")
        
        expected_color_str: str | None = args.color
        if expected_color_str is not None:
            expected_color = [int(ch) for ch in expected_color_str.split(",")]
        else:
            expected_color = None

        image = load_image_render(image_path)

        blobs = find_unique_blobs(image, expected_color)
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
