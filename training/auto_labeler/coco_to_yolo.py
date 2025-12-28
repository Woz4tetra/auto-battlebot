#!/usr/bin/env python3
"""
Convert Auto Labeler COCO annotations to YOLO bounding box format.

The auto labeler outputs segmentation masks in COCO format. This script:
1. Reads the COCO annotations JSON
2. Converts segmentation masks to bounding boxes
3. Outputs YOLO format text files (one per image)
4. Optionally generates a class mapping JSON with label colors from server config

YOLO format: class_id center_x center_y width height (all normalized 0-1)

Usage examples:
  # Basic conversion
  python coco_to_yolo.py /path/to/coco /path/to/output
  
  # Generate class mapping JSON with colors from server config
  python coco_to_yolo.py /path/to/coco /path/to/output \\
    --class-mapping-json mapping.json \\
    --config config/labeling_config.yaml
"""

import argparse
import json
import shutil
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import cv2
import numpy as np
import yaml
from tqdm import tqdm


def load_colors_from_config(config_path: Path) -> Dict[str, List[int]]:
    """
    Load label colors from server config file.

    Args:
        config_path: Path to the server config YAML file

    Returns:
        Dictionary mapping label names to RGB color lists
    """
    with open(config_path, "r") as f:
        config_data = yaml.safe_load(f)

    colors = {}
    for label_data in config_data.get("object_labels", []):
        name = label_data.get("name")
        color_value = label_data.get("color", [255, 0, 0])

        # Parse color (handle both named colors and RGB lists)
        if isinstance(color_value, str):
            # For named colors, we'd need the full color mapping from config.py
            # For now, just note it's a named color
            colors[name] = color_value
        elif isinstance(color_value, list) and len(color_value) == 3:
            colors[name] = color_value
        else:
            colors[name] = [255, 0, 0]  # Default red

    return colors


def mask_to_bbox(mask: np.ndarray) -> Tuple[int, int, int, int]:
    """
    Convert a binary mask to a bounding box.
    Finds the largest contour and creates bbox from it to filter out noise.

    Args:
        mask: Binary mask array

    Returns:
        (x_min, y_min, x_max, y_max) in pixel coordinates
    """
    # Find contours in the mask
    contours, _ = cv2.findContours(
        mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return (0, 0, 0, 0)

    # Find the largest contour by area
    largest_contour = max(contours, key=cv2.contourArea)

    # Get bounding box of the largest contour
    x, y, w, h = cv2.boundingRect(largest_contour)

    return (int(x), int(y), int(x + w), int(y + h))


def rle_to_mask(rle: dict, height: int, width: int) -> np.ndarray:
    """
    Convert RLE (Run Length Encoding) to binary mask.

    Args:
        rle: RLE dict with 'counts' and 'size'
        height: Image height
        width: Image width

    Returns:
        Binary mask array
    """
    from pycocotools import mask as mask_utils

    if isinstance(rle, dict):
        mask = mask_utils.decode(rle)
    else:
        # Already decoded or in different format
        mask = np.zeros((height, width), dtype=np.uint8)

    return mask


def polygon_to_bbox(segmentation: List[List[float]]) -> Tuple[int, int, int, int]:
    """
    Convert polygon segmentation to bounding box.
    Uses the largest polygon only to filter out noise.

    Args:
        segmentation: List of polygon coordinates [x1, y1, x2, y2, ...]

    Returns:
        (x_min, y_min, x_max, y_max) in pixel coordinates
    """
    if not segmentation or len(segmentation) == 0:
        return (0, 0, 0, 0)

    # Find the largest polygon by computing area
    largest_polygon = None
    largest_area = 0

    for polygon in segmentation:
        if len(polygon) < 6:  # Need at least 3 points (6 coordinates)
            continue

        # Convert to numpy array for area calculation
        points = np.array(polygon).reshape(-1, 2)

        # Calculate polygon area using Shoelace formula
        x = points[:, 0]
        y = points[:, 1]
        area = 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))

        if area > largest_area:
            largest_area = area
            largest_polygon = polygon

    if largest_polygon is None:
        return (0, 0, 0, 0)

    # Extract x and y coordinates from the largest polygon
    x_coords = largest_polygon[0::2]
    y_coords = largest_polygon[1::2]

    x_min = int(min(x_coords))
    y_min = int(min(y_coords))
    x_max = int(max(x_coords))
    y_max = int(max(y_coords))

    return (x_min, y_min, x_max, y_max)


def convert_bbox_to_yolo(
    bbox: Tuple[int, int, int, int], img_width: int, img_height: int
) -> Tuple[float, float, float, float]:
    """
    Convert pixel bounding box to YOLO format (normalized).

    Args:
        bbox: (x_min, y_min, x_max, y_max) in pixels
        img_width: Image width
        img_height: Image height

    Returns:
        (center_x, center_y, width, height) normalized to [0, 1]
    """
    x_min, y_min, x_max, y_max = bbox

    # Calculate center and dimensions
    center_x = (x_min + x_max) / 2.0
    center_y = (y_min + y_max) / 2.0
    width = x_max - x_min
    height = y_max - y_min

    # Normalize to [0, 1]
    center_x /= img_width
    center_y /= img_height
    width /= img_width
    height /= img_height

    # Clamp to valid range
    center_x = max(0.0, min(1.0, center_x))
    center_y = max(0.0, min(1.0, center_y))
    width = max(0.0, min(1.0, width))
    height = max(0.0, min(1.0, height))

    return (center_x, center_y, width, height)


def convert_coco_to_yolo(
    coco_dir: Path,
    output_dir: Path,
    copy_images: bool = True,
    split_name: str = "train",
    class_mapping_json: Optional[str] = None,
    config_path: Optional[Path] = None,
):
    """
    Convert COCO annotations to YOLO format.

    Args:
        coco_dir: Directory containing COCO annotations and images
        output_dir: Output directory for YOLO dataset
        copy_images: Whether to copy images to output directory
        split_name: Dataset split name (train/val/test)
        class_mapping_json: Optional path to save class index to name mapping as JSON
        config_path: Optional path to server config file to load label colors
    """
    # Read COCO annotations
    annotations_file = coco_dir / "annotations" / "annotations.json"
    if not annotations_file.exists():
        raise FileNotFoundError(f"COCO annotations not found: {annotations_file}")

    print(f"Loading COCO annotations from {annotations_file}")
    with open(annotations_file, "r") as f:
        coco_data = json.load(f)

    # Create output directories
    images_dir = output_dir / "images" / split_name
    labels_dir = output_dir / "labels" / split_name
    images_dir.mkdir(parents=True, exist_ok=True)
    labels_dir.mkdir(parents=True, exist_ok=True)

    # Build category mapping (COCO category_id to YOLO class_id)
    categories = {cat["id"]: idx for idx, cat in enumerate(coco_data["categories"])}
    category_names = {cat["id"]: cat["name"] for cat in coco_data["categories"]}

    # Save classes.txt
    classes_file = output_dir / "classes.txt"
    with open(classes_file, "w") as f:
        for cat in coco_data["categories"]:
            f.write(f"{cat['name']}\n")
    print(f"Saved class names to {classes_file}")

    # Save class mapping JSON if requested
    if class_mapping_json:
        # Load colors from config if provided
        label_colors = {}
        if config_path and config_path.exists():
            label_colors = load_colors_from_config(config_path)

        # Build class mapping with optional colors
        class_mapping = {}
        for idx, cat in enumerate(coco_data["categories"]):
            label_name = cat["name"]
            entry = {"name": label_name}

            # Add color if available
            if label_name in label_colors:
                entry["color"] = label_colors[label_name]

            class_mapping[idx] = entry

        mapping_path = Path(class_mapping_json)
        with open(mapping_path, "w") as f:
            json.dump(class_mapping, f, indent=2)
        print(f"Saved class mapping to {mapping_path}")

    # Build image_id to filename mapping
    images_dict = {img["id"]: img for img in coco_data["images"]}

    # Group annotations by image_id
    annotations_by_image = {}
    for ann in coco_data["annotations"]:
        image_id = ann["image_id"]
        if image_id not in annotations_by_image:
            annotations_by_image[image_id] = []
        annotations_by_image[image_id].append(ann)

    print(
        f"Converting {len(images_dict)} images with {len(coco_data['annotations'])} annotations"
    )

    # Process each image
    converted_count = 0
    skipped_count = 0

    for image_id, image_info in tqdm(images_dict.items(), desc="Converting to YOLO"):
        filename = image_info["file_name"]
        img_width = image_info["width"]
        img_height = image_info["height"]

        # Find source image
        source_image = coco_dir / filename
        if not source_image.exists():
            # Try in images subdirectory
            source_image = coco_dir / "images" / filename

        if not source_image.exists():
            print(f"Warning: Image not found: {filename}")
            skipped_count += 1
            continue

        # Copy or symlink image
        dest_image = images_dir / Path(filename).name
        if copy_images:
            shutil.copy2(source_image, dest_image)
        else:
            if dest_image.exists():
                dest_image.unlink()
            dest_image.symlink_to(source_image.resolve())

        # Get annotations for this image
        anns = annotations_by_image.get(image_id, [])

        # Convert each annotation to YOLO format
        yolo_lines = []
        for ann in anns:
            category_id = ann["category_id"]
            class_id = categories[category_id]

            # Get bounding box from different annotation formats
            if "segmentation" in ann:
                # Convert segmentation to bbox
                segmentation = ann["segmentation"]
                if isinstance(segmentation, dict):
                    # RLE format
                    try:
                        mask = rle_to_mask(segmentation, img_height, img_width)
                        bbox = mask_to_bbox(mask)
                    except Exception as e:
                        print(f"Warning: Could not decode RLE for {filename}: {e}")
                        continue
                elif isinstance(segmentation, list):
                    # Polygon format
                    bbox = polygon_to_bbox(segmentation)
                else:
                    print(f"Warning: Unknown segmentation format for {filename}")
                    continue
            elif "bbox" in ann and ann["bbox"]:
                # COCO bbox format: [x, y, width, height]
                x, y, w, h = ann["bbox"]
                bbox = (int(x), int(y), int(x + w), int(y + h))
            else:
                print(f"Warning: No bbox or segmentation for annotation in {filename}")
                continue

            # Skip invalid bboxes
            if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
                continue

            # Convert to YOLO format
            yolo_bbox = convert_bbox_to_yolo(bbox, img_width, img_height)

            # Format: class_id center_x center_y width height
            yolo_line = f"{class_id} {yolo_bbox[0]:.6f} {yolo_bbox[1]:.6f} {yolo_bbox[2]:.6f} {yolo_bbox[3]:.6f}"
            yolo_lines.append(yolo_line)

        # Write YOLO annotation file
        label_file = labels_dir / f"{Path(filename).stem}.txt"
        with open(label_file, "w") as f:
            f.write("\n".join(yolo_lines))
            if yolo_lines:  # Add trailing newline if there are annotations
                f.write("\n")

        converted_count += 1

    # Create data.yaml for YOLO
    yaml_file = output_dir / "data.yaml"
    yaml_content = f"""# YOLO dataset configuration
# Converted from COCO format by coco_to_yolo.py

path: {output_dir.absolute()}  # dataset root dir
train: images/{split_name}  # train images (relative to 'path')
val: images/{split_name}  # val images (relative to 'path')

# Classes
names:
"""
    for idx, cat in enumerate(coco_data["categories"]):
        yaml_content += f"  {idx}: {cat['name']}\n"

    with open(yaml_file, "w") as f:
        f.write(yaml_content)

    print(f"\nConversion complete!")
    print(f"  Converted: {converted_count} images")
    print(f"  Skipped: {skipped_count} images")
    print(f"  Output directory: {output_dir}")
    print(f"  Images: {images_dir}")
    print(f"  Labels: {labels_dir}")
    print(f"  Config: {yaml_file}")


def merge_to_yolo_dataset(
    coco_dir: Path,
    yolo_dataset: Path,
    split_name: str = "train",
    class_mapping_json: Optional[str] = None,
    config_path: Optional[Path] = None,
):
    """
    Merge COCO annotations into an existing YOLO dataset.

    Args:
        coco_dir: Directory containing COCO annotations
        yolo_dataset: Existing YOLO dataset directory
        split_name: Dataset split to merge into (train/val/test)
        class_mapping_json: Optional path to save class index to name mapping as JSON
        config_path: Optional path to server config file to load label colors
    """
    # Check if dataset exists
    if not yolo_dataset.exists():
        print(f"YOLO dataset not found at {yolo_dataset}, creating new one")
        convert_coco_to_yolo(
            coco_dir,
            yolo_dataset,
            copy_images=True,
            split_name=split_name,
            class_mapping_json=class_mapping_json,
            config_path=config_path,
        )
        return

    # Load existing classes
    classes_file = yolo_dataset / "classes.txt"
    if not classes_file.exists():
        print(f"Warning: classes.txt not found in {yolo_dataset}, creating new dataset")
        convert_coco_to_yolo(
            coco_dir,
            yolo_dataset,
            copy_images=True,
            split_name=split_name,
            class_mapping_json=class_mapping_json,
            config_path=config_path,
        )
        return

    with open(classes_file, "r") as f:
        existing_classes = [line.strip() for line in f if line.strip()]

    # Load COCO annotations to check class compatibility
    annotations_file = coco_dir / "annotations" / "annotations.json"
    with open(annotations_file, "r") as f:
        coco_data = json.load(f)

    new_classes = [cat["name"] for cat in coco_data["categories"]]

    # Check if classes match
    if set(new_classes) != set(existing_classes):
        print(f"Warning: Class mismatch!")
        print(f"  Existing classes: {existing_classes}")
        print(f"  New classes: {new_classes}")
        response = input("Continue anyway? This may cause incorrect labels (y/n): ")
        if response.lower() != "y":
            print("Merge cancelled")
            return

    print(f"Merging into existing YOLO dataset at {yolo_dataset}")
    convert_coco_to_yolo(
        coco_dir,
        yolo_dataset,
        copy_images=True,
        split_name=split_name,
        class_mapping_json=class_mapping_json,
        config_path=config_path,
    )


def main():
    parser = argparse.ArgumentParser(
        description="Convert Auto Labeler COCO annotations to YOLO bounding box format"
    )
    parser.add_argument(
        "coco_dir",
        type=str,
        help="Directory containing COCO annotations (with annotations/annotations.json)",
    )
    parser.add_argument(
        "output_dir",
        type=str,
        help="Output directory for YOLO dataset (will be created or merged)",
    )
    parser.add_argument(
        "--split",
        type=str,
        default="train",
        choices=["train", "val", "test"],
        help="Dataset split name (default: train)",
    )
    parser.add_argument(
        "--merge",
        action="store_true",
        help="Merge into existing YOLO dataset instead of overwriting",
    )
    parser.add_argument(
        "--symlink",
        action="store_true",
        help="Create symlinks to images instead of copying",
    )
    parser.add_argument(
        "--class-mapping-json",
        type=str,
        default=None,
        help="Optional output path for JSON file containing class index to label name mapping",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="Optional path to server config file (e.g., labeling_config.yaml) to load label colors",
    )

    args = parser.parse_args()

    coco_dir = Path(args.coco_dir)
    output_dir = Path(args.output_dir)

    if not coco_dir.exists():
        print(f"Error: COCO directory not found: {coco_dir}")
        return 1

    config_path = Path(args.config) if args.config else None

    if args.merge:
        merge_to_yolo_dataset(
            coco_dir, output_dir, args.split, args.class_mapping_json, config_path
        )
    else:
        convert_coco_to_yolo(
            coco_dir,
            output_dir,
            copy_images=not args.symlink,
            split_name=args.split,
            class_mapping_json=args.class_mapping_json,
            config_path=config_path,
        )

    return 0


if __name__ == "__main__":
    exit(main())
