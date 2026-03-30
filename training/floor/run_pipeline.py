#!/usr/bin/env python3
"""Unified CLI entrypoint for the floor segmentation pipeline."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from common import LOGGER, configure_logging, load_pipeline_config, mkdir


def run_stage(script_name: str, config_path: Path, verbose: bool, extra_args: list[str] | None = None) -> None:
    script_path = Path(__file__).parent / script_name
    command = [sys.executable, str(script_path), "--config", str(config_path)]
    if verbose:
        command.append("--verbose")
    if extra_args:
        command.extend(extra_args)
    LOGGER.info("Running stage: %s", script_name)
    subprocess.run(command, check=True)


def ensure_directories(config_path: Path) -> None:
    cfg = load_pipeline_config(config_path)
    mkdir(cfg.paths.root)
    mkdir(cfg.paths.raw_videos)
    mkdir(cfg.paths.processed_videos)
    mkdir(cfg.paths.metadata_dir)
    mkdir(cfg.paths.masks_dir)
    mkdir(cfg.paths.filtered_masks_dir)
    mkdir(cfg.paths.export_dir)
    mkdir(cfg.paths.logs_dir)
    mkdir(cfg.paths.checkpoints_dir)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run floor segmentation data pipeline")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).parent / "config" / "pipeline.toml",
    )
    parser.add_argument(
        "--start-at",
        choices=[
            "scrape",
            "transcode",
            "segment_seed",
            "propagate",
            "quality",
            "export",
        ],
        default="scrape",
    )
    parser.add_argument("--skip-scrape", action="store_true")
    parser.add_argument("--disable-cache", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    configure_logging(args.verbose)
    ensure_directories(args.config)
    stages = [
        ("scrape", "scrape_brettzone.py", []),
        ("transcode", "transcode_videos.py", []),
        ("segment_seed", "sam3_floor_segment.py", []),
        (
            "propagate",
            "propagate_masks.py",
            ["--disable-cache"] if args.disable_cache else [],
        ),
        ("quality", "quality_filter.py", []),
        ("export", "export_dataset.py", []),
    ]
    start_idx = next(i for i, (name, _, _) in enumerate(stages) if name == args.start_at)
    for name, script_name, extra in stages[start_idx:]:
        if name == "scrape" and args.skip_scrape:
            LOGGER.info("Skipping scrape stage by request")
            continue
        run_stage(script_name, args.config, args.verbose, extra)

    LOGGER.info("Pipeline finished.")


if __name__ == "__main__":
    main()
