# Cage-high annotation script details

## Annotating clips (`playground/bgsub_cage/annotate_cage_video.py`)

Annotate fixed-camera cage clips with YOLO detections, gated by the cage hull.

Runs one or more ultralytics .pt models over each clip and writes an annotated mp4 per model. Detections whose box centre falls inside the DeepLab convex hull draw solid; detections outside draw dim and dashed. Keeping both on screen is the point: without the dropped boxes you cannot tell "the hull deleted it" from "the network never found it".

Also writes a compact per-clip stats JSON, which is where the write-up's numbers come from. Per-frame kept counts are what reliability is measured on: a 1v1 fight has two robots, so a frame with fewer than two kept `robot` boxes is a dropout.

```
Usage:
    python annotate_cage_video.py CLIP_OR_DIR         --models a.pt b.pt -o OUT_DIR
```

## Downloading clips (`playground/bgsub_cage/download_cage_video.py`)

Download NHRL fixed overhead cage video from BrettZone as 1080p clips.

Every recording in this repo comes from a moving ZED, so background subtraction cannot be evaluated on it. This pulls the one NHRL camera that never moves: ``Cage-N-Overhead-High``, a near top-down view of the cage floor.

The source object on Linode is 3840x2160 at 59.94 fps and about 1.4 GB per fight. It serves byte ranges, so ffmpeg seeks straight to the fight window and scales to 1080p in the same pass. The 4K never lands on disk.

Output names match the BrettZone export pattern that ``training/deeplab/field_labels.py`` already parses::

```
    BZ-<tournamentID>-<p1clean>-<p2clean>-<gameID>-Cage-<n>-Overhead-High.mp4
```

```
Usage:
    python download_cage_video.py OUTPUT_DIR --limit 20 --seed 0 --since 2026-04-01
    python download_cage_video.py OUTPUT_DIR --dry-run
```
