"""Interactive viewer for an image + segmentation mask pair.

Displays the image with a colored overlay of label indices. Press 'T' to
toggle between the plain image and the overlay. Uses matplotlib so you can
pan and zoom.

Usage:
    python view_mask.py image.jpg image_mask.png
    python view_mask.py image.jpg image_mask.png --alpha 0.5
"""

import argparse
import sys
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap


def build_colormap(num_labels: int) -> ListedColormap:
    """Label 0 is transparent; remaining labels cycle through tab20."""
    base = plt.colormaps["tab20"]
    colors = [(0.0, 0.0, 0.0, 0.0)]
    for i in range(1, num_labels):
        colors.append(base(i % base.N))
    return ListedColormap(colors)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="View an image with segmentation mask overlay"
    )
    parser.add_argument("image", type=str, help="Path to the image (.jpg or .png)")
    parser.add_argument("mask", type=str, help="Path to the mask (_mask.png)")
    parser.add_argument(
        "--alpha", type=float, default=0.45, help="Overlay opacity (default: 0.45)"
    )
    args = parser.parse_args()

    image_path = Path(args.image)
    mask_path = Path(args.mask)

    if not image_path.exists():
        print(f"Error: image not found: {image_path}", file=sys.stderr)
        sys.exit(1)
    if not mask_path.exists():
        print(f"Error: mask not found: {mask_path}", file=sys.stderr)
        sys.exit(1)

    image_bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image_bgr is None:
        print(f"Error: could not read image: {image_path}", file=sys.stderr)
        sys.exit(1)
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

    mask_raw = cv2.imread(str(mask_path), cv2.IMREAD_UNCHANGED)
    if mask_raw is None:
        print(f"Error: could not read mask: {mask_path}", file=sys.stderr)
        sys.exit(1)
    mask = mask_raw[:, :, 0] if mask_raw.ndim == 3 else mask_raw

    if mask.shape[:2] != image_rgb.shape[:2]:
        mask = cv2.resize(
            mask,
            (image_rgb.shape[1], image_rgb.shape[0]),
            interpolation=cv2.INTER_NEAREST,
        )

    unique_labels = np.unique(mask)
    num_labels = int(unique_labels.max()) + 1
    cmap = build_colormap(num_labels)

    print(f"Image: {image_path.name} ({image_rgb.shape[1]}x{image_rgb.shape[0]})")
    print(f"Mask: {mask_path.name}, labels present: {unique_labels.tolist()}")
    print("Press 'T' to toggle overlay")

    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    fig.canvas.manager.set_window_title(f"Mask Viewer - {image_path.name}")

    img_layer = ax.imshow(image_rgb)
    overlay_layer = ax.imshow(
        mask,
        cmap=cmap,
        vmin=0,
        vmax=num_labels - 1,
        alpha=args.alpha,
        interpolation="nearest",
    )

    legend_handles = []
    for label in unique_labels:
        color = cmap(label / max(num_labels - 1, 1))
        patch = plt.matplotlib.patches.Patch(
            facecolor=color, edgecolor="black", label=f"Label {label}"
        )
        legend_handles.append(patch)
    ax.legend(handles=legend_handles, loc="upper right", fontsize=9, framealpha=0.8)

    ax.set_axis_off()
    fig.tight_layout()

    overlay_visible = [True]

    def on_key(event):
        if event.key in ("t", "T"):
            overlay_visible[0] = not overlay_visible[0]
            overlay_layer.set_alpha(args.alpha if overlay_visible[0] else 0.0)
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect("key_press_event", on_key)
    plt.show()


if __name__ == "__main__":
    main()
