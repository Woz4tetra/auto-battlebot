import argparse
from pathlib import Path
import tomllib
from typing import Dict, List

import cv2
import numpy as np


def find_all_blobs(image: np.ndarray) -> List[Dict]:
    """Find all colored blobs in the image using connected components."""
    # Extract only non-transparent pixels (alpha == 0xFFFF for 16-bit)
    alpha_mask = (image[:, :, 3] == 0xFFFF).astype(np.uint8) * 255

    # Find all connected components in the alpha mask
    num_labels, labels = cv2.connectedComponents(alpha_mask, connectivity=8)

    blobs = []
    # Skip label 0 (background)
    for label_id in range(1, num_labels):
        # Get mask for this blob
        blob_mask = labels == label_id

        # Get all pixel coordinates for this blob
        coords = np.column_stack(np.where(blob_mask))
        # Convert from (y, x) to (x, y) format
        coords_list = [(int(x), int(y)) for y, x in coords]

        # Calculate centroid
        centroid_y, centroid_x = coords.mean(axis=0)

        # Get average color of this blob (in 16-bit BGR space)
        blob_pixels = image[blob_mask]
        avg_color = blob_pixels[:, :3].mean(axis=0)

        blobs.append(
            {
                "coords": coords_list,
                "centroid": (float(centroid_x), float(centroid_y)),
                "avg_color": avg_color,
                "num_pixels": len(coords_list),
            }
        )

    return blobs


def hex_to_bgr(hex_color: str) -> np.ndarray:
    """Convert hex color string to BGR numpy array (16-bit space)."""
    hex_color = hex_color.lstrip('#')
    r = int(hex_color[0:2], 16)
    g = int(hex_color[2:4], 16)
    b = int(hex_color[4:6], 16)
    # Convert 8-bit to 16-bit space (scale by 257 = 65535/255)
    return np.array([b * 257, g * 257, r * 257], dtype=np.float32)


def bgr_to_hex(bgr_color: np.ndarray) -> str:
    """Convert BGR numpy array (16-bit space) to hex color string."""
    # Convert 16-bit to 8-bit space (divide by 257)
    b = int(bgr_color[0] / 257)
    g = int(bgr_color[1] / 257)
    r = int(bgr_color[2] / 257)
    return f"{r:02x}{g:02x}{b:02x}"


def match_blob_to_label(blob_color: np.ndarray, labels: List[Dict], tolerance_percent: float) -> int:
    """Match a blob's average color to a label ID based on configured colors."""
    # Convert tolerance from percentage to 16-bit value
    # 100% = 65535, so tolerance = 65535 * (tolerance_percent / 100)
    tolerance = 65535 * (tolerance_percent / 100)
    
    for label in labels:
        label_color = label["color_bgr"]
        # Compare colors with tolerance (16-bit space)
        color_diff = np.abs(blob_color.astype(np.int32) - label_color.astype(np.int32))
        if np.all(color_diff < tolerance):
            return label["id"]

    # No match found
    return -1


def load_config(config_path: Path) -> Dict:
    """Load and parse the TOML configuration file."""
    with open(config_path, "rb") as f:
        config = tomllib.load(f)
    
    # Convert hex colors to BGR for labels
    for label in config["labels"]:
        label["color_bgr"] = hex_to_bgr(label["color"])
    
    return config


def discover_colors_mode(data_dir: Path) -> None:
    """Discover and display all unique colors in the point overlay images."""
    config_path = data_dir / "config.toml"
    
    if not config_path.exists():
        print(f"Error: Config file not found at {config_path}")
        return
    
    # Load configuration to get directory paths
    config = load_config(config_path)
    point_overlay_dir = data_dir / config["point_overlay"]
    
    if not point_overlay_dir.exists():
        print(f"Error: Point overlay directory not found at {point_overlay_dir}")
        return
    
    print(f"Scanning images in: {point_overlay_dir}")
    print(f"{'=' * 60}")
    
    # Collect all unique colors across all images
    all_colors = []
    image_paths = sorted([p for p in point_overlay_dir.iterdir() if p.suffix == ".png"])
    
    for image_path in image_paths:
        image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
        if image is None:
            continue
        
        blobs = find_all_blobs(image)
        for blob in blobs:
            # Check if this color is already in our list (with small tolerance)
            is_duplicate = False
            for existing_color in all_colors:
                color_diff = np.abs(blob["avg_color"].astype(np.int32) - existing_color.astype(np.int32))
                if np.all(color_diff < 1000):  # Small tolerance for duplicates
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                all_colors.append(blob["avg_color"])
    
    print(f"\nFound {len(all_colors)} unique colors:\n")
    
    for idx, color in enumerate(all_colors):
        hex_color = bgr_to_hex(color)
        b, g, r = int(color[0] / 257), int(color[1] / 257), int(color[2] / 257)
        print(f"Color {idx}:")
        print(f"  Hex:  {hex_color}")
        print(f"  RGB:  ({r}, {g}, {b})")
        print(f"  BGR (16-bit): ({int(color[0])}, {int(color[1])}, {int(color[2])})")
        print()
    
    print(f"{'=' * 60}")
    print("\nTo use these colors in your config.toml:")
    print("\n[[labels]]")
    print('name = "your_label_name"')
    print('id = 0')
    print(f'color = "{bgr_to_hex(all_colors[0]) if all_colors else "ababab"}"')
    print()


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert keypoint data from Blender renders using config file")
    parser.add_argument("data_dir", type=str, help="Path to the data directory containing config.toml")
    parser.add_argument("--show-debug", action="store_true", help="Show individual blob masks")
    parser.add_argument("--discover-colors", action="store_true", help="Discover and display all unique colors in images")
    args = parser.parse_args()

    data_dir = Path(args.data_dir)
    
    # Handle discover-colors mode
    if args.discover_colors:
        discover_colors_mode(data_dir)
        return
    
    config_path = data_dir / "config.toml"
    
    if not config_path.exists():
        print(f"Error: Config file not found at {config_path}")
        return
    
    # Load configuration
    config = load_config(config_path)
    
    # Get directory paths from config
    point_overlay_dir = data_dir / config["point_overlay"]
    base_render_dir = data_dir / config["base_render"]
    
    if not point_overlay_dir.exists():
        print(f"Error: Point overlay directory not found at {point_overlay_dir}")
        return
    
    show_debug = args.show_debug
    color_tolerance = config["keypoints"]["color_tolerance"]
    labels = config["labels"]

    # Store results for all images
    all_results = {}
    
    # Track which label IDs were found
    found_label_ids = set()

    # Sort image paths for consistent ordering
    image_paths = sorted([p for p in point_overlay_dir.iterdir() if p.suffix == ".png"])
    background_image_paths = (
        sorted([p for p in base_render_dir.iterdir() if p.suffix == ".png"]) if base_render_dir.exists() else []
    )
    background_image_stems = [p.stem for p in background_image_paths]

    for image_path in image_paths:
        print(f"\nProcessing: {image_path.name}")

        if show_debug and image_path.stem in background_image_stems:
            background_image_path = background_image_paths[background_image_stems.index(image_path.stem)]
            background_image = cv2.imread(str(background_image_path))
        else:
            background_image = None

        image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
        if image is None:
            print("  Failed to load image")
            continue

        # Find all blobs in the image
        blobs = find_all_blobs(image)
        print(f"  Found {len(blobs)} blobs")

        # Map blobs to label IDs based on color similarity
        image_keypoints = {}
        for blob in blobs:
            label_id = match_blob_to_label(blob["avg_color"], labels, color_tolerance)
            
            if label_id == -1:
                print(f"  Warning: Blob with color {blob['avg_color']} did not match any configured label")
                continue
            
            found_label_ids.add(label_id)

            image_keypoints[label_id] = {
                "color": blob["avg_color"].tolist(),  # BGR values (16-bit)
                "coords": blob["coords"],
                "centroid": blob["centroid"],
                "num_pixels": blob["num_pixels"],
            }

            cx, cy = blob["centroid"]
            label_name = next(label["name"] for label in labels if label["id"] == label_id)
            print(f"  {label_name} (ID {label_id}): {blob['num_pixels']} pixels, centroid at ({cx:.1f}, {cy:.1f})")

        all_results[image_path.name] = image_keypoints

        # Debug visualization showing individual blobs
        if background_image is not None and len(image_keypoints) > 0:
            vis = background_image.copy()
            for label_id, data in image_keypoints.items():
                centroid = data["centroid"]
                cx, cy = int(centroid[0]), int(centroid[1])

                # Draw circle at centroid
                cv2.circle(vis, (cx, cy), 5, (0, 255, 0), -1)
                # Draw label name with background
                label_name = next(label["name"] for label in labels if label["id"] == label_id)
                text = f"{label_name} ({label_id})"
                (text_w, text_h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(vis, (cx + 10, cy - text_h - 5), (cx + 10 + text_w, cy + 5), (0, 0, 0), -1)
                cv2.putText(
                    vis,
                    text,
                    (cx + 10, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                )

            cv2.imshow("Keypoints", vis)
            print("  Press any key to continue, 'q' to quit...")
            key = chr(cv2.waitKey(0) & 0xFF)
            if key == "q":
                quit()

    if show_debug:
        cv2.destroyAllWindows()

    # Print summary
    print(f"\n{'=' * 60}")
    print(f"SUMMARY: Found {len(found_label_ids)} unique labels across all images")
    print(f"{'=' * 60}")
    
    for label in labels:
        if label["id"] in found_label_ids:
            print(f"  ✓ {label['name']} (ID {label['id']})")
        else:
            print(f"  ✗ {label['name']} (ID {label['id']}) - NOT FOUND")

    for img_name, keypoints in all_results.items():
        print(f"\n{img_name}:")
        for label_id in sorted(keypoints.keys()):
            data = keypoints[label_id]
            cx, cy = data["centroid"]
            label_name = next(label["name"] for label in labels if label["id"] == label_id)
            print(f"  {label_name} (ID {label_id}): centroid=({cx:.1f}, {cy:.1f}), pixels={data['num_pixels']}")


if __name__ == "__main__":
    main()
