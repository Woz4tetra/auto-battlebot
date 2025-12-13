import argparse
import cv2
import numpy as np
from pathlib import Path
from typing import List, Tuple
from load_image_render import load_image_render


def parse_yolo_label(label_path: Path) -> List[Tuple]:
    """Parse YOLO format label file with keypoints.

    Format: class_id cx cy w h [kp1_x kp1_y kp1_visible kp2_x kp2_y kp2_visible ...]
    All coordinates are normalized (0-1).
    """
    annotations = []
    if not label_path.exists():
        return annotations

    with open(label_path, "r") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 5:
                continue

            class_id = int(parts[0])
            cx, cy, box_w, box_h = map(float, parts[1:5])

            # Parse keypoints if present
            keypoints = []
            if len(parts) > 5:
                kp_data = parts[5:]
                # Keypoints come in groups of 3: x, y, visibility
                for i in range(0, len(kp_data), 3):
                    if i + 2 < len(kp_data):
                        kp_x = float(kp_data[i])
                        kp_y = float(kp_data[i + 1])
                        kp_visible = int(kp_data[i + 2])
                        keypoints.append((kp_x, kp_y, kp_visible))

            annotations.append((class_id, cx, cy, box_w, box_h, keypoints))

    return annotations


def draw_annotations(image: np.ndarray, annotations: List[Tuple]) -> np.ndarray:
    """Draw bounding boxes and keypoints on image."""
    img_h, img_w = image.shape[:2]
    annotated = image.copy()

    for annotation in annotations:
        class_id, cx, cy, box_w, box_h, keypoints = annotation

        # Convert normalized coordinates to pixel coordinates
        cx_px = int(cx * img_w)
        cy_px = int(cy * img_h)
        w_px = int(box_w * img_w)
        h_px = int(box_h * img_h)

        # Calculate bounding box corners
        x1 = int(cx_px - w_px / 2)
        y1 = int(cy_px - h_px / 2)
        x2 = int(cx_px + w_px / 2)
        y2 = int(cy_px + h_px / 2)

        # Draw bounding box
        color = (0, 255, 0)  # Green
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

        # Draw class label
        label = f"Class {class_id}"
        cv2.putText(
            annotated, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
        )

        # Draw center point
        cv2.circle(annotated, (cx_px, cy_px), 4, (0, 0, 255), -1)

        # Draw keypoints
        for i, (kp_x, kp_y, kp_visible) in enumerate(keypoints):
            kp_x_px = int(kp_x * img_w)
            kp_y_px = int(kp_y * img_h)

            # Color coding based on visibility
            # 0 = not visible, 1 = occluded, 2 = visible
            if kp_visible == 0:
                kp_color = (128, 128, 128)  # Gray - not visible
            elif kp_visible == 1:
                kp_color = (0, 165, 255)  # Orange - occluded
            else:
                kp_color = (255, 0, 0)  # Blue - visible

            cv2.circle(annotated, (kp_x_px, kp_y_px), 5, kp_color, -1)
            cv2.circle(annotated, (kp_x_px, kp_y_px), 6, (255, 255, 255), 1)

            # Draw keypoint number
            cv2.putText(
                annotated,
                str(i),
                (kp_x_px + 8, kp_y_px - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                kp_color,
                1,
            )

    return annotated


def get_image_label_pairs(
    images_path: Path, labels_path: Path
) -> List[Tuple[Path, Path]]:
    """Find all matching image-label pairs."""
    pairs = []

    if not images_path.exists() or not labels_path.exists():
        return pairs

    # Get all image files
    image_extensions = [".jpg", ".jpeg", ".png", ".bmp"]
    image_files = []
    for ext in image_extensions:
        image_files.extend(images_path.glob(f"*{ext}"))
        image_files.extend(images_path.glob(f"*{ext.upper()}"))

    # Sort for consistent ordering
    image_files = sorted(image_files)

    # Find corresponding label files
    for img_path in image_files:
        label_path = labels_path / f"{img_path.stem}.txt"
        if label_path.exists():
            pairs.append((img_path, label_path))

    return pairs


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Draw YOLO annotations with keypoints on images. "
        "Use arrow keys to navigate: → (next), ← (previous), ESC/q (quit)"
    )
    parser.add_argument("images", type=Path, help="Path to directory containing images")
    parser.add_argument(
        "labels", type=Path, help="Path to directory containing label files"
    )
    args = parser.parse_args()

    images_path: Path = args.images
    labels_path: Path = args.labels

    # Validate paths
    if not images_path.exists():
        print(f"Error: Images path does not exist: {images_path}")
        return

    if not labels_path.exists():
        print(f"Error: Labels path does not exist: {labels_path}")
        return

    # Get all image-label pairs
    pairs = get_image_label_pairs(images_path, labels_path)

    if not pairs:
        print("No matching image-label pairs found!")
        return

    print(f"Found {len(pairs)} image-label pairs")

    # Navigation state
    current_idx = 0
    window_name = "YOLO Annotations"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    while True:
        # Load current image and annotations
        img_path, label_path = pairs[current_idx]
        image = load_image_render(img_path)
        annotations = parse_yolo_label(label_path)

        # Debug: Print parsed annotations
        print(f"\n{img_path.name}:")
        print(f"Image size: {image.shape[1]}x{image.shape[0]}")
        for ann in annotations:
            class_id, cx, cy, w, h, keypoints = ann
            print(
                f"  Class {class_id}: bbox=({cx:.3f}, {cy:.3f}) size=({w:.3f}, {h:.3f})"
            )
            for i, (kp_x, kp_y, kp_vis) in enumerate(keypoints):
                print(f"    Keypoint {i}: ({kp_x:.3f}, {kp_y:.3f}) vis={kp_vis}")

        # Draw annotations
        annotated_image = draw_annotations(image, annotations)

        # Add navigation info
        info_text = f"{current_idx + 1}/{len(pairs)} - {img_path.name} - {len(annotations)} objects"
        cv2.putText(
            annotated_image,
            info_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            annotated_image,
            info_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 0),
            1,
        )

        # Display image
        cv2.imshow(window_name, annotated_image)

        # Wait for key press
        key = cv2.waitKey(0) & 0xFF

        # Handle navigation
        if key == 27 or key == ord("q"):  # ESC or 'q' to quit
            break
        elif key == 81 or key == 2:  # Left arrow (Linux/Windows)
            current_idx = max(0, current_idx - 1)
        elif key == 83 or key == 3:  # Right arrow (Linux/Windows)
            current_idx = min(len(pairs) - 1, current_idx + 1)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
