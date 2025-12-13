import argparse
from pathlib import Path
import tomllib


from find_unique_blobs import find_unique_blobs, ColorRgbUint8, Blob, get_blob_centroid, get_blob_bounding_rectangle

from load_image_render import load_image_render


def color_close(c1: ColorRgbUint8, c2: ColorRgbUint8, tol: float):
    # c1, c2: (R,G,B) tuples, tol: percent
    return all(abs(a - b) <= tol / 100 * 255 for a, b in zip(c1, c2))


BoundingBoxAnnotation = tuple[int, float, float, float, float]
KeypointAnnotation = tuple[int, float, float]
KeypointAnnotationWithLabel = tuple[int, int, float, float]


def match_class_id(
    blob_color: ColorRgbUint8,
    label_colors: dict[int, ColorRgbUint8],
    color_tolerance: float,
) -> int | None:
    # Match blob color to label
    class_id = None
    for label_id, label_color in label_colors.items():
        if color_close(blob_color, label_color, color_tolerance):
            class_id = label_id
            break
    return class_id


def make_bounding_box_annotation(
    blob: Blob,
    width: int,
    height: int,
    label_colors: dict[int, ColorRgbUint8],
    color_tolerance: float,
) -> BoundingBoxAnnotation | None:
    class_id = match_class_id(blob.color, label_colors, color_tolerance)
    if class_id is None:
        return None  # skip unknown blobs
    # Get bounding box
    rectangle = get_blob_bounding_rectangle(blob)
    x_center, y_center, box_w, box_h = rectangle
    return class_id, x_center / width, y_center / height, box_w / width, box_h / height


def make_keypoint_annotation(
    blob: Blob,
    width: int,
    height: int,
    keypoint_colors: dict[int, dict[int, ColorRgbUint8]],
    color_tolerance: float,
) -> KeypointAnnotationWithLabel | None:
    matched_keypoint_id = None
    matched_class_id = None
    for class_id, keypoint_label_colors in keypoint_colors.items():
        matched_keypoint_id = match_class_id(
            blob.color, keypoint_label_colors, color_tolerance
        )
        if matched_keypoint_id is not None:
            matched_class_id = class_id
            break
    if matched_keypoint_id is None or matched_class_id is None:
        return None  # skip unknown blobs

    x_centroid, y_centroid = get_blob_centroid(blob)
    return (matched_class_id, matched_keypoint_id, x_centroid / width, y_centroid / height)

def load_color_configs(config: dict) -> tuple[dict[int, ColorRgbUint8], dict[int, dict[int, ColorRgbUint8]]]:
    mask_labels = config.get("masks", {}).get("labels", [])
    keypoint_labels = config.get("keypoints", {}).get("labels", [])

    # Build label color lookup
    label_colors = {}
    for label in mask_labels:
        label_id = label["id"]
        color = tuple(label["color"])
        label_colors[label_id] = color

    keypoint_colors = {}
    for label in keypoint_labels:
        label_id = label["id"]
        color = tuple(label["color"])
        keypoint_colors.setdefault(label_id, {})[label["keypoint_id"]] = color
    
    return label_colors, keypoint_colors

def get_image_paths(config: dict, data_dir: Path) -> list[tuple[Path, Path]]:
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

    # Create dictionaries for quick lookup by stem (filename without extension)
    mask_dict = {img.stem: img for img in mask_images}
    keypoint_dict = {img.stem: img for img in keypoint_images}

    # Find matching pairs
    matched_pairs = []
    all_stems = set(mask_dict.keys()) | set(keypoint_dict.keys())
    
    for stem in sorted(all_stems):
        if stem in mask_dict and stem in keypoint_dict:
            matched_pairs.append((mask_dict[stem], keypoint_dict[stem]))
        elif stem in mask_dict:
            print(f"Warning: No keypoint image found for mask image: {mask_dict[stem]}")
        else:
            print(f"Warning: No mask image found for keypoint image: {keypoint_dict[stem]}")
    
    return matched_pairs




def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert keypoint and mask data from Blender renders to YOLO format using config file"
    )
    parser.add_argument(
        "data_dir",
        metavar="data-dir",
        type=Path,
        help="Path to the data directory containing config.toml",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        help="Path to the output directory for YOLO labels",
    )
    args = parser.parse_args()

    data_dir: Path = args.data_dir
    output_dir: Path = args.output_dir if args.output_dir else data_dir / "labels"
    output_dir.mkdir(parents=True, exist_ok=True)

    config_path = data_dir / "config.toml"
    with open(config_path, "rb") as f:
        config = tomllib.load(f)

    # Get label info and color tolerance
    label_colors, keypoint_colors = load_color_configs(config)
    color_tolerance = config.get("masks", {}).get("color_tolerance", 5)

    matched_images = get_image_paths(config, data_dir)
    
    for mask_path, keypoints_path in matched_images:
        mask_img = load_image_render(mask_path)
        keypoints_img = load_image_render(keypoints_path)
        assert mask_img.shape == keypoints_img.shape
        mask_blobs = find_unique_blobs(mask_img)
        keypoint_blobs = find_unique_blobs(keypoints_img)
        height, width = mask_img.shape[:2]

        keypoint_annotations: dict[int, list[KeypointAnnotation]] = {}
        for blob in keypoint_blobs:
            annotation = make_keypoint_annotation(
                blob, width, height, keypoint_colors, color_tolerance
            )
            if annotation:
                keypoint_annotations.setdefault(annotation[0], []).append(
                    annotation[1:]
                )
        for keypoint_label_annotations in keypoint_annotations.values():
            keypoint_label_annotations.sort(key=lambda val: val[0])

        bounding_box_annotations: list[BoundingBoxAnnotation] = []
        for blob in mask_blobs:
            annotation = make_bounding_box_annotation(
                blob, width, height, label_colors, color_tolerance
            )
            if annotation:
                bounding_box_annotations.append(annotation)

        yolo_lines = []
        visibility = 2
        for bbox_anno in bounding_box_annotations:
            class_id, x_center, y_center, box_w, box_h = bbox_anno
            keypoint_label_annotations = keypoint_annotations[class_id]
            yolo_base_annotation = (
                f"{class_id} {x_center:.6f} {y_center:.6f} {box_w:.6f} {box_h:.6f}"
            )
            yolo_keypoint_annotations = []
            for key_anno in keypoint_label_annotations:
                keypoint_id, key_x, key_y = key_anno
                yolo_keypoint_annotations.append(
                    f"{key_x:.6f} {key_y:.6f} {visibility}"
                )
            yolo_annotation = " ".join(
                [yolo_base_annotation] + yolo_keypoint_annotations
            )
            yolo_lines.append(yolo_annotation)

        # Write YOLO label file
        label_path = output_dir / (mask_path.stem + ".txt")
        with open(label_path, "w") as f:
            f.write("\n".join(yolo_lines) + "\n")
        print(f"Wrote {label_path}")


if __name__ == "__main__":
    main()
