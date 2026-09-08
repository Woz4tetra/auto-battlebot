# Stationary-camera cage footage

Every recording in this repo comes from a moving ZED on our own robot, so background
subtraction cannot be evaluated on any of it. These two scripts build a stationary-camera
test set from NHRL's fixed `Cage-N-Overhead-High` camera and run the 2class detectors over
it with the cage floor masked by DeepLab plus a convex hull.

Background: `docs/experiments/perception_performance/rembg_cage_high_plan.md` (the planned
version of this experiment, built on stills) and `rembg_field_2026-09-04.md` (the moving-ZED
null result it follows up).

## Download

```bash
source scripts/activate_python.sh

venv/bin/python playground/bgsub_cage/download_cage_video.py \
    data/downloads/brettzone_cage_high --limit 20 --seed 0 --since 2026-04-01 --dry-run

venv/bin/python playground/bgsub_cage/download_cage_video.py \
    data/downloads/brettzone_cage_high --limit 20 --seed 0 --since 2026-04-01
```

Source objects on `nhrl-matches.us-east-1.linodeobjects.com` are 3840x2160 at 59.94 fps and
about 1.4 GB per fight. They serve byte ranges, so ffmpeg seeks to the fight window and scales
to 1080p in one pass. The 4K is never written to disk.

`--since 2026-04-01` keeps the sample away from the tournaments in the detector's training
corpus. That is a best effort, not a guarantee: the script also lists
`training/data/nhrl_robots_bbox_2class.tar.gz` once and records any overlap in `MANIFEST.md`.
Pass `--skip-corpus-check` to skip that listing, which takes a few minutes over 12.1 GB.

## Annotate


```bash
# Hulls and previews only. Look at these before committing to a full inference run.
venv/bin/python playground/bgsub_cage/annotate_cage_video.py \
    data/downloads/brettzone_cage_high \
    --models data/eval_models/yolo26n_nhrl_robots_bbox_2class_2026-09-04.pt \
    -o data/downloads/brettzone_cage_high/annotated --hull-only

# Both models over every clip.
venv/bin/python playground/bgsub_cage/annotate_cage_video.py \
    data/downloads/brettzone_cage_high \
    --models data/eval_models/yolo26n_nhrl_robots_bbox_2class_2026-09-04.pt \
             data/eval_models/yolo26x_nhrl_robots_bbox_2class_2026-09-04.pt \
    -o data/downloads/brettzone_cage_high/annotated
```

`--imgsz` defaults to 640 because both 2class models were trained at 640. `--conf` defaults
to 0.25, low enough that marginal detections stay visible in the clips.

## What comes out

```
data/downloads/brettzone_cage_high/
    BZ-<tournament>-<p1>-<p2>-<gameID>-Cage-<n>-Overhead-High.mp4   1080p clip
    BZ-....hull.json                                               cached hull polygon
    manifest.json, MANIFEST.md                                     provenance
    annotated/
        BZ-..._yolo26n.mp4, BZ-..._yolo26x.mp4                     annotated clips
        BZ-..._yolo26n.dets.json                                   per-frame stats
        summary.json                                               all clips, all models
        hull_previews/BZ-..._hull.jpg                              median frame + hull
```

Clip names follow the BrettZone export pattern `training/deeplab/field_labels.py` already
parses, so these drop into the field-mask tooling without a new filename rule.

## Reading the annotated clips

- Cyan outline: the convex hull of the DeepLab floor mask, computed once per clip from a
  per-pixel median frame. The camera is fixed, so one hull covers the whole clip.
- Solid box: detection whose centre is inside the hull. Green is `robot`, amber is `house_bot`.
- Dashed grey box: detection outside the hull, dropped. These stay drawn on purpose. Without
  them you cannot tell whether the hull deleted a robot or the network never found it.

In `summary.json`, `both_robots_rate` is the fraction of frames holding at least two kept
`robot` boxes. A 1v1 fight has two robots on the floor the whole time, so that number is a
direct reliability measure with no hand labelling. `longest_blind_frames` is the worst run of
consecutive frames with zero robots found.
