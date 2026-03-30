# Floor Segmentation Data Pipeline

This project builds a floor-segmentation dataset for YOLO-Seg from BrettZone fight videos with no manual intervention in the generation loop.

## What It Does

1. Scrapes BrettZone fight pages and downloads random fight videos.
2. Downscales/transcodes long 720p60 videos to manageable resolution/FPS.
3. Runs prompt-grounded SAM3 seed segmentation per chunk.
4. Propagates masks across full videos in chunked windows.
5. Applies automatic quality gates to reduce false positives/negatives.
6. Exports every `n`th frame to YOLO-Seg labels + `_mask.png`.

## Install

Base environment:

```bash
./install/install_python_environment.sh
```

Floor + SAM3 environment:

```bash
./install/install_floor_sam3_environment.sh --sam3-path /path/to/sam3
# or
./install/install_floor_sam3_environment.sh --sam3-git-url https://github.com/<org>/<sam3-repo>.git
```

## SAM3 Adapter Contract

`training/floor/sam3_adapter.py` expects one of:

- `sam3.build_floor_segmenter(checkpoint, device, amp)` in the installed `sam3` package, or
- a custom module defined in `config/pipeline.toml` (`sam3.adapter_module`) that exports:

```python
def build_adapter(checkpoint: str, device: str, amp: bool):
    ...
```

Adapter object must implement:

- `segment_seed(seed: SegmentationSeed) -> np.ndarray` (`0/1` mask)
- `propagate_video(video_path, initial_mask, start_frame_idx, end_frame_idx, async_prefetch, disable_cache) -> dict[int, np.ndarray]`

## Configuration

Main config: `training/floor/config/pipeline.toml`

- Prompt strategy defaults to tokenized tags:
  - `arena floor . floor . ground . mat . painted floor . logo . red corner . blue corner`
- Low initial grounding thresholds:
  - `text_threshold = 0.18`, `box_threshold = 0.18`
- Automatic fallback seed points and arena box are generated per chunk.

Additional prompt presets: `training/floor/config/prompts.toml`

## Scraper Usage

The scraper stage is implemented in `training/floor/scrape_brettzone.py`.
It discovers fight pages from BrettZone, resolves downloadable entries, randomly samples fights, and deduplicates downloaded videos by SHA-256.

Run scraper only:

```bash
python training/floor/scrape_brettzone.py --config training/floor/config/pipeline.toml
```

Useful options:

```bash
# Download fewer fights for a quick run
python training/floor/scrape_brettzone.py --limit 10

# Deterministic random sampling
python training/floor/scrape_brettzone.py --seed 2026

# Show debug logs
python training/floor/scrape_brettzone.py --verbose
```

Scraper outputs (default under `training/floor/data/metadata`):

- `videos_index.jsonl`: one row per newly downloaded video (source URL, tournament/game IDs, camera metadata, duration, hash, local path)
- `scrape_state.json`: persistent dedupe state and selected fight URLs

Downloaded media is written to:

- `training/floor/data/raw/videos`

Notes:

- Requires `yt-dlp` (included via `pip install -e ".[floor]"` or `install_floor_sam3_environment.sh`)
- If BrettZone page layout changes, scraper still attempts regex fallback extraction of `fightReviewSync.php` links

## Run

Single command pipeline:

```bash
python training/floor/run_pipeline.py --config training/floor/config/pipeline.toml --disable-cache
```

Resume from a later stage:

```bash
python training/floor/run_pipeline.py --start-at propagate --disable-cache
```

## Outputs

Under `training/floor/data` (default):

- `raw/videos` downloaded videos
- `processed/videos` transcoded videos
- `masks/raw` SAM3 seeds + propagated masks
- `masks/filtered` quality-passed masks
- `exports/yolo_seg/images`, `labels`, `masks`
- `metadata/*.json|*.jsonl` manifests and per-stage summaries

## Testing

Run unit tests:

```bash
python -m unittest discover -s training/floor/tests -p "test_*.py"
```

Run smoke test (no SAM3 required):

```bash
python training/floor/tests/smoke_run.py
```
