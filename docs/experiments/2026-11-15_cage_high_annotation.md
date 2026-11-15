# 2026-11-15 cage high annotation

## Notes moved from code comments (2026-11-15)

### `playground/bgsub_cage/annotate_cage_video.py`

Runs one or more ultralytics .pt models over each clip and writes an annotated mp4
per model. Detections whose box centre falls inside the DeepLab convex hull draw
solid; detections outside draw dim and dashed. Keeping both on screen is the point:
without the dropped boxes you cannot tell "the hull deleted it" from "the network
never found it".

Also writes a compact per-clip stats JSON, which is where the write-up's numbers
come from. Per-frame kept counts are what reliability is measured on: a 1v1 fight
has two robots, so a frame with fewer than two kept `robot` boxes is a dropout.

The DeepLab loader is imported from auto_battlebot.segmentation, so no PYTHONPATH is needed.

Usage:
python annotate_cage_video.py CLIP_OR_DIR         --models a.pt b.pt -o OUT_DIR

- A dropped box with at least this much of its area inside the hull is a robot against the cage wall, not something outside the cage. (at `WALL_OVERLAP_MIN = 0.10`)
- Arrow drawn past the front keypoint so a short front-back span is still readable. (at `HEADING_EXTEND = 1.6`)
- Full box dumps every Nth frame. The per-frame count arrays cover every frame; this keeps a readable sample of actual geometry without a 20 MB JSON per clip. (at `BOX_DUMP_STRIDE = 10`)

Keypoints come as (n, 2, 3): front then back, each x, y, conf. The heading is only
reported when both keypoints clear KEYPOINT_MIN_CONF, because a vector built from a
guessed keypoint points somewhere arbitrary and would be worse than drawing nothing.

`box` is off when the poses come from the same model as the detections: that box is
already on screen in its hull-gate colour, and drawing it twice hides the gate.

`kept` is the gate: the box centre is inside the hull. Centre rather than full
containment, because a robot pinned against the cage wall has a box that spills
onto the barrier while the robot itself is still on the floor.

`overlap` records how much of the box is inside anyway, which separates the two
reasons a box gets dropped. A robot at the wall lands just outside the centre
test but still overlaps the hull; a person in the crowd overlaps nothing. Without
that split, a wall pin and a spectator look like the same failure.

That makes `both_robots_rate` a reliability measure with no hand labelling: any
frame holding fewer than two kept fighter boxes is a miss.

- A cached hull still needs its preview drawn if the preview is missing, so a rerun into a fresh output directory does not silently skip the eyeball check. (at `if cache.exists() and preview.exists() and not args.rebuild_hulls:`)
- Full stem, not the leading token: four models of one family and date differ only in the middle of the name, and a shorter tag overwrites their outputs. (at `tag = model_path.stem`)

### `playground/bgsub_cage/download_cage_video.py`

Every recording in this repo comes from a moving ZED, so background subtraction
cannot be evaluated on it. This pulls the one NHRL camera that never moves:
``Cage-N-Overhead-High``, a near top-down view of the cage floor.

The source object on Linode is 3840x2160 at 59.94 fps and about 1.4 GB per fight.
It serves byte ranges, so ffmpeg seeks straight to the fight window and scales to
1080p in the same pass. The 4K never lands on disk.

Output names match the BrettZone export pattern that
``training/deeplab/field_labels.py`` already parses::

BZ-<tournamentID>-<p1clean>-<p2clean>-<gameID>-Cage-<n>-Overhead-High.mp4

Usage:
python download_cage_video.py OUTPUT_DIR --limit 20 --seed 0 --since 2026-04-01
python download_cage_video.py OUTPUT_DIR --dry-run

- The docs ask for no more than one request every 5 s against live endpoints. The archive endpoints used here carry no such limit, but a small gap keeps us polite. (at `REQUEST_GAP_S = 0.25`)
- Pre-roll before the fight starts and post-roll after it ends, in seconds. The pre-roll catches the countdown with both robots parked, which is the cleanest background frame a clip contains. (at `PRE_ROLL_S = 3.0`)
- Oversample so fights whose overhead recording is missing can be replaced without a second pass over the API. (at `candidates = sample_fights(fights, args.limit * 3, args.per_tournament`)

