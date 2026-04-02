#!/usr/bin/env python3
"""Visual diagnostic for SAM3 floor segmentation using text prompts only.

Usage examples:

    # Default prompt from pipeline.toml prompt_tags (joined with " . ")
    python test_sam3.py /path/to/video.mp4 --frames 90,200,500

    # Explicit text prompt
    python test_sam3.py /path/to/video.mp4 --frames 90 --prompt "arena floor"

    # Save outputs to a custom directory
    python test_sam3.py /path/to/video.mp4 --frames 90,300 -o /tmp/sam3_debug

Outputs one PNG per frame: the original frame with the predicted mask drawn as
a semi-transparent overlay. The arena crop box and prompt string are drawn for
context.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from common import configure_logging, load_pipeline_config  # noqa: E402
from sam3_adapter import build_adapter  # noqa: E402

MASK_COLOR = (0, 200, 80)
MASK_ALPHA = 0.45
BOX_COLOR = (255, 200, 0)


def extract_frame(video_path: str, frame_idx: int) -> np.ndarray:
    cap = cv2.VideoCapture(video_path)
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError(f"Cannot read frame {frame_idx} from {video_path}")
    return frame


def infer_arena_box(h: int, w: int) -> tuple[int, int, int, int]:
    ix, iy = int(0.06 * w), int(0.06 * h)
    return (ix, iy, w - ix, h - iy)


def default_prompt_from_config(cfg) -> str:
    tags = cfg.sam3.prompt_tags
    if not tags:
        return "floor"
    return " . ".join(tags)


def draw_overlay(
    frame: np.ndarray,
    mask: np.ndarray | None,
    arena_box: tuple[int, int, int, int],
    prompt_text: str,
) -> np.ndarray:
    vis = frame.copy()

    if mask is not None:
        colored = np.zeros_like(vis)
        colored[mask > 0] = MASK_COLOR
        cv2.addWeighted(colored, MASK_ALPHA, vis, 1.0, 0, vis)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(vis, contours, -1, MASK_COLOR, 2)
    else:
        cv2.putText(vis, "NO MASK RETURNED", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

    x1, y1, x2, y2 = arena_box
    cv2.rectangle(vis, (x1, y1), (x2, y2), BOX_COLOR, 1, cv2.LINE_AA)

    caption = prompt_text if len(prompt_text) <= 80 else prompt_text[:77] + "..."
    y0 = 28
    cv2.putText(vis, caption, (20, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 2, cv2.LINE_AA)

    if mask is not None:
        pct = 100.0 * np.count_nonzero(mask) / mask.size
        cv2.putText(vis, f"floor: {pct:.1f}%", (20, y0 + 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

    return vis


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visual SAM3 segmentation test (text prompts only)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("video", type=str, help="Path to video file")
    parser.add_argument(
        "--frames", type=str, default="90",
        help="Comma-separated frame indices (default: 90)",
    )
    parser.add_argument(
        "--config", type=Path,
        default=Path(__file__).parent / "config" / "pipeline.toml",
    )
    parser.add_argument("-o", "--output-dir", type=Path, default=None,
                        help="Output directory (default: training/floor/data/test_output)")
    parser.add_argument(
        "--prompt", type=str, default=None,
        help="SAM3 text prompt (default: pipeline.toml prompt_tags joined with ' . ')",
    )
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    configure_logging(args.verbose)
    cfg = load_pipeline_config(args.config)

    prompt_text = (args.prompt or "").strip() or default_prompt_from_config(cfg)
    if not prompt_text:
        print("Error: empty text prompt (set --prompt or prompt_tags in config)",
              file=sys.stderr)
        sys.exit(1)

    frame_indices = [int(f.strip()) for f in args.frames.split(",")]
    video_path = Path(args.video).resolve()
    if not video_path.exists():
        print(f"Error: video not found: {video_path}", file=sys.stderr)
        sys.exit(1)

    out_dir = args.output_dir or Path(cfg.paths.root) / "test_output"
    out_dir.mkdir(parents=True, exist_ok=True)

    print("Loading SAM3 adapter...")
    adapter = build_adapter(
        adapter_module=cfg.sam3.adapter_module,
        checkpoint=cfg.sam3.checkpoint,
        device=cfg.sam3.device,
        amp=cfg.sam3.amp,
        model_cfg=cfg.sam3.model_cfg,
    )
    segment_text = getattr(adapter, "segment_frame_text", None)
    if not callable(segment_text):
        print(
            "Error: adapter does not implement segment_frame_text(); "
            "this test script requires SAM3 text prompting.",
            file=sys.stderr,
        )
        sys.exit(1)

    print(f"Adapter ready. Processing {len(frame_indices)} frame(s) from {video_path.name}")
    print(f"Text prompt: {prompt_text!r}")

    cap = cv2.VideoCapture(str(video_path))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    cap.release()
    print(f"Video: {w}x{h}, {total_frames} frames")

    arena_box = infer_arena_box(h, w)

    for fidx in frame_indices:
        if fidx >= total_frames:
            print(f"  Skipping frame {fidx} (video only has {total_frames} frames)")
            continue

        frame = extract_frame(str(video_path), fidx)
        print(f"  Frame {fidx}: text prompt")

        try:
            mask = segment_text(
                video_path=video_path,
                frame_idx=fidx,
                text=prompt_text,
            )
        except Exception as exc:
            print(f"  Frame {fidx}: SAM3 error: {exc}")
            mask = None

        vis = draw_overlay(frame, mask, arena_box, prompt_text)

        stem = video_path.stem
        out_path = out_dir / f"{stem}_frame{fidx:06d}.png"
        cv2.imwrite(str(out_path), vis)
        if mask is not None:
            pct = 100.0 * np.count_nonzero(mask) / mask.size
            print(f"  Frame {fidx}: mask {pct:.1f}% → {out_path.name}")
        else:
            print(f"  Frame {fidx}: NO MASK → {out_path.name}")

    print(f"\nOutputs saved to {out_dir}")


if __name__ == "__main__":
    main()
