#!/usr/bin/env python3
"""Downscale/transcode downloaded videos and build chunk/frame metadata."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from common import (
    LOGGER,
    configure_logging,
    json_dump,
    load_pipeline_config,
    mkdir,
    run_cmd,
)


def ffprobe_stream_info(video_path: Path) -> dict:
    command = [
        "ffprobe",
        "-v",
        "error",
        "-show_entries",
        "stream=width,height,r_frame_rate,avg_frame_rate,nb_frames",
        "-of",
        "json",
        str(video_path),
    ]
    result = run_cmd(command)
    data = json.loads(result.stdout)
    streams = data.get("streams", [])
    if not streams:
        raise RuntimeError(f"No video streams found: {video_path}")
    return streams[0]


def ffprobe_duration(video_path: Path) -> float:
    command = [
        "ffprobe",
        "-v",
        "error",
        "-show_entries",
        "format=duration",
        "-of",
        "default=noprint_wrappers=1:nokey=1",
        str(video_path),
    ]
    result = run_cmd(command)
    return float(result.stdout.strip())


def transcode_video(input_path: Path, output_path: Path, width: int, height: int, fps: int, crf: int, preset: str, keep_audio: bool) -> None:
    mkdir(output_path.parent)
    vf = f"scale={width}:{height}:force_original_aspect_ratio=decrease,pad={width}:{height}:(ow-iw)/2:(oh-ih)/2"
    command = [
        "ffmpeg",
        "-y",
        "-i",
        str(input_path),
        "-vf",
        vf,
        "-r",
        str(fps),
        "-c:v",
        "libx264",
        "-preset",
        preset,
        "-crf",
        str(crf),
    ]
    if keep_audio:
        command.extend(["-c:a", "aac", "-b:a", "128k"])
    else:
        command.append("-an")
    command.append(str(output_path))
    run_cmd(command)


def build_chunks(duration: float, chunk_seconds: int) -> list[dict]:
    chunks: list[dict] = []
    start = 0.0
    idx = 0
    while start < duration:
        end = min(duration, start + chunk_seconds)
        chunks.append(
            {
                "chunk_index": idx,
                "start_seconds": round(start, 3),
                "end_seconds": round(end, 3),
                "duration_seconds": round(end - start, 3),
            }
        )
        start = end
        idx += 1
    return chunks


def main() -> None:
    parser = argparse.ArgumentParser(description="Transcode floor videos and generate chunk metadata")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).parent / "config" / "pipeline.toml",
    )
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    configure_logging(args.verbose)
    cfg = load_pipeline_config(args.config)
    mkdir(cfg.paths.processed_videos)
    mkdir(cfg.paths.metadata_dir)

    raw_videos = sorted(
        p for p in cfg.paths.raw_videos.glob("*") if p.suffix.lower() in {".mp4", ".mov", ".mkv", ".webm"}
    )
    if not raw_videos:
        LOGGER.warning("No raw videos found at %s", cfg.paths.raw_videos)
        return

    transcode_index: list[dict] = []
    for idx, raw_video in enumerate(raw_videos, start=1):
        out_name = f"{raw_video.stem}_w{cfg.video.target_width}h{cfg.video.target_height}_{cfg.video.target_fps}fps.mp4"
        output_video = cfg.paths.processed_videos / out_name
        if output_video.exists():
            LOGGER.info("[%d/%d] Reusing transcoded video %s", idx, len(raw_videos), output_video.name)
        else:
            LOGGER.info("[%d/%d] Transcoding %s", idx, len(raw_videos), raw_video.name)
            transcode_video(
                input_path=raw_video,
                output_path=output_video,
                width=cfg.video.target_width,
                height=cfg.video.target_height,
                fps=cfg.video.target_fps,
                crf=cfg.video.crf,
                preset=cfg.video.preset,
                keep_audio=cfg.video.keep_audio,
            )

        stream_info = ffprobe_stream_info(output_video)
        duration = ffprobe_duration(output_video)
        chunks = build_chunks(duration, cfg.video.chunk_seconds)

        metadata = {
            "input_video": str(raw_video),
            "output_video": str(output_video),
            "duration_seconds": duration,
            "stream_info": stream_info,
            "chunks": chunks,
        }
        meta_path = cfg.paths.metadata_dir / f"{output_video.stem}.json"
        json_dump(meta_path, metadata)
        transcode_index.append(metadata)

    json_dump(cfg.paths.metadata_dir / "transcode_index.json", transcode_index)
    LOGGER.info("Done. Prepared %d videos.", len(transcode_index))


if __name__ == "__main__":
    main()
