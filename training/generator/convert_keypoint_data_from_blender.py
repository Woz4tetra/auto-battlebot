import argparse
from pathlib import Path
import tomllib

import cv2

from find_unique_blobs import find_unique_blobs, ColorRgbUint8, load_image_render


def color_close(c1: ColorRgbUint8, c2: ColorRgbUint8, tol: float):
    # c1, c2: (R,G,B) tuples, tol: percent
    return all(abs(a - b) <= tol/100*255 for a, b in zip(c1, c2))


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert keypoint and mask data from Blender renders to YOLO format using config file")
    parser.add_argument("data_dir", metavar="data-dir", type=Path, help="Path to the data directory containing config.toml")
    parser.add_argument("-o", "--output-dir", type=Path, help="Path to the output directory for YOLO labels")
    args = parser.parse_args()

    data_dir: Path = args.data_dir
    output_dir: Path = args.output_dir if args.output_dir else data_dir / "labels"
    output_dir.mkdir(parents=True, exist_ok=True)

    config_path = data_dir / "config.toml"
    with open(config_path, "rb") as f:
        config = tomllib.load(f)

    # Get mask label info and color tolerance
    mask_labels = config.get("masks", {}).get("labels", [])
    keypoint_labels = config.get("keypoints", {}).get("labels", [])
    color_tolerance = config.get("masks", {}).get("color_tolerance", 5)

    # Build label color lookup: (R,G,B) -> (id, name)
    label_colors = []
    for label in mask_labels:
        color = tuple(label["color"])
        label_colors.append((color, label["id"]))

    # Find mask image directories
    mask_dir = data_dir / config["mask_overlay"]
    if not mask_dir.exists():
        raise FileNotFoundError(f"Mask directory {mask_dir} not found")

    # Find all mask images
    mask_images = sorted(mask_dir.glob("*.png"))
    if not mask_images:
        raise FileNotFoundError(f"No mask images found in {mask_dir}")

    # Find keypoint image directories
    keypoints_dir = data_dir / config["point_overlay"]
    if not keypoints_dir.exists():
        raise FileNotFoundError(f"Keypoints directory {keypoints_dir} not found")

    # Find all key point images
    keypoint_images = sorted(keypoints_dir.glob("*.png"))
    if not keypoint_images:
        raise FileNotFoundError(f"No keypoint images found in {keypoints_dir}")

    for mask_path, keypoints_path in zip(mask_images, keypoint_images):
        mask_img = load_image_render(mask_path)
        keypoints_img = load_image_render(keypoints_path)
        mask_blobs = find_unique_blobs(mask_img)
        keypoint_blobs = find_unique_blobs(keypoints_img)
        h, w = mask_img.shape[:2]
        yolo_lines = []
        for blob in mask_blobs:
            # Match blob color to label
            blob_color = tuple(blob.color)
            class_id = None
            for label_color, label_id in label_colors:
                print(blob_color, label_color)
                if color_close(blob_color, label_color, color_tolerance):
                    class_id = label_id
                    break
            if class_id is None:
                continue  # skip unknown blobs
            # Get bounding box
            xs = [pt[0] for pt in blob.contour]
            ys = [pt[1] for pt in blob.contour]
            x_min, x_max = min(xs), max(xs)
            y_min, y_max = min(ys), max(ys)
            x_center = (x_min + x_max) / 2 / w
            y_center = (y_min + y_max) / 2 / h
            box_w = (x_max - x_min) / w
            box_h = (y_max - y_min) / h
            yolo_lines.append(f"{class_id} {x_center:.6f} {y_center:.6f} {box_w:.6f} {box_h:.6f}")
        # Write YOLO label file
        label_path = output_dir / (mask_path.stem + ".txt")
        with open(label_path, "w") as f:
            f.write("\n".join(yolo_lines) + "\n")
        print(f"Wrote {label_path}")

if __name__ == "__main__":
    main()
