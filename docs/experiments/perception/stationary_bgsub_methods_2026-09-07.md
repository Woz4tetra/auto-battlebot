# What tracks small moving robots reliably on a fixed overhead cage camera?

Status: **done** (2026-09-07). Code: `playground/bgsub_cage/`. Footage, annotated clips and
per-frame stats: `data/downloads/brettzone_cage_high/`.

Twenty NHRL fights from the `Cage-N-Overhead-High` camera, 178,031 frames at 1080p and 59.94 fps,
run through `yolo26n` and `yolo26x` with the cage floor masked by DeepLab plus a convex hull.
This is the first stationary-camera footage in the repo. Every existing recording comes from a
moving ZED, which is why `rembg_field_2026-09-04.md` could not separate a viewpoint effect from
one lucky recording, and why `rembg_cage_high_plan.md` was written and never run.

**Near 100% per-frame detection is not reachable here, and it is also not the thing to chase.**
The best per-frame configuration measured holds both robots on 96.7% of frames. What gets to
near 100% is track continuity, because the gaps are short: 72% last under 100 ms and 93% under
400 ms. A tracker that coasts through them closes what no detector setting does.

The biggest single win costs nothing to build. `yolo26n` at its default `imgsz 640` and
`conf 0.25` holds both robots on 86.6% of frames. Running the same weights at `imgsz 1280` and
`conf 0.05` behind the hull takes the same three clips from 83.7% to 94.0%. At 1080p with
`imgsz 640` the median robot reaches the network at 40 px and the 5th percentile at 21 px, so
the detector was being starved of resolution, not of capability.

## The three methods, ranked

**1. Run the detector you already have at `imgsz 1280` and `conf 0.05`, behind the cage hull.**

| clip | 640 / 0.25 | 0.05 only | 1280 only | 1280 / 0.05 |
|---|---|---|---|---|
| `luna-xenomorph` | 74.4% | 86.6% | 79.0% | **89.3%** |
| `hss-pepperoni` | 91.5% | 94.0% | 94.3% | **97.5%** |
| `parallax-catharsis` | 83.6% | 88.6% | 91.1% | **96.3%** |

Resolution and threshold recover overlapping but different frames: 43% of misses for `imgsz`
alone, 39% for `conf` alone, and together they take the mean from 83.7% to 94.0%. The hull is
what makes the low threshold usable. Dropping to `conf 0.05` at 640 more than doubles off-field
false positives on `hss-pepperoni` (109 to 1,347), and every one of them lands outside the cage
where the hull removes it. Raising resolution then takes the false positives back down: at
1280 / 0.05, `luna-xenomorph` produces 1,271 off-field detections against 5,305 at 640 / 0.05.

Cost is inference time. `yolo26n` at 640 runs 178,031 frames in about 30 minutes on an RTX 4080
laptop; at 1280 it is roughly 3x that. The Jetson budget is 60 ms end to end, so this is a
choice about where the frame budget goes, not a free lunch on the robot.

**2. Per-clip median background subtraction, fused into the detector.**
Still worth adding after tuning, with a smaller share: on the same three clips it lifts the
tuned detector from 94.0% to 96.7%, rescuing 46% of the frames the detector still loses. On the
untuned detector across all 20 clips it lifts 86.6% to 92.8% for `yolo26n` and 86.5% to 93.5%
for `yolo26x`, rescuing 46% and 51% of their misses. Both models land in the same place, which
says the gain comes from the subtractor seeing something the detector cannot rather than from
either model's particular weaknesses.

It is cheap and needs no alignment on this footage. Within a single fight the camera moves at
most 3.25 px and 18 of 20 clips stay under 1 px, so a plain per-pixel median over 60 sampled
frames is a clean empty cage. It returns motion, not identity, so it belongs behind the detector
as a recall stage feeding a tracker, never as a replacement: blobs carry no class, and the house
bot and loose debris make them too.

**3. Temporal association that coasts through gaps.**
The one that actually reaches near 100%, and the only one addressing the real failure. Across
178,031 frames there are 2,527 gaps where fewer than two robots are held. Median 3 frames
(50 ms), and 72% are under 100 ms. A constant-velocity track with a 400 ms coast covers 93% of
them. Frames where the detector holds no robot at all are rare: 125 gaps in 178,031 frames, p90
of 8 frames, worst case 50 frames (834 ms).

Build it against the tail, not the median. p99 is 100 frames (1.66 s) and the worst gap runs
491 frames (8.2 s), so a coast alone will not carry every case and re-identification has to
handle the rest. That shape matches the moving-camera baseline in this repo almost exactly, p90
317 ms here against 340 ms there, which says the tail is not a property of our camera, our
motion, or our detector. It is a property of two robots fighting.

## What ran

```bash
venv/bin/python playground/bgsub_cage/download_cage_video.py \
    data/downloads/brettzone_cage_high --limit 20 --seed 0 --since 2026-04-01

PYTHONPATH=training/deeplab venv/bin/python playground/bgsub_cage/annotate_cage_video.py \
    data/downloads/brettzone_cage_high \
    --models data/eval_models/yolo26{n,x}_nhrl_robots_bbox_2class_2026-09-04.pt \
    -o data/downloads/brettzone_cage_high/annotated
```

Twenty fights across `nhrl_apr26`, `nhrl_may26`, `nhrl_may26pro` and `nhrl_jun26`, in 3 lb,
12 lb and 30 lb, from cages 1 and 2. Source objects are 3840x2160 at 59.94 fps and about 1.4 GB
each; ffmpeg range-seeks the fight window off the Linode object store and scales to 1080p in one
pass, so the 4K never lands on disk. Both detectors ran at `imgsz 640`, the size they were
trained at, and `conf 0.25`, unless a row says otherwise. The `imgsz` and `conf` sweeps ran on
three or four clips, not all twenty; treat those as a direction with a consistent sign, not a
scored result.

### Reliability is measured without hand labelling

A 1v1 fight keeps exactly two robots on the floor for its whole duration. So the fraction of
frames holding at least two kept `robot` boxes inside the hull is a reliability measure that
needs no annotation, and it is what every rate here reports. It is a recall lower bound, not an
accuracy score: it says nothing about whether a box sits on the right robot.

### The detectors have not seen this footage

`yolo26*_nhrl_robots_bbox_2class` was trained on `nhrl_robots_bbox_2class`, whose 22 cage-high
scenes come from this same BrettZone archive. Listing that corpus gives 11 tournaments, newest
`nhrl_feb26_3lb`, and its newest cage-high scene is dated 2026-03-07. Every clip here is from
2026-04-04 or later. **Zero overlap**, so these are detection numbers rather than training fit.
That was the risk `rembg_cage_high_plan.md` raised, where 18 of 22 scenes were contaminated and
the YOLO reference had to fall back to four val scenes.

## The robots are small where it counts

| | p1 | p5 | p50 | p95 |
|---|---|---|---|---|
| `robot` box side, px at 1080p | 55 | 64 | 121 | 257 |
| same, as seen by the network at `imgsz 640` | 18 | 21 | 40 | 86 |

16.6% of robot boxes are under 80 px on a side. The existing labelled cage-high corpus has a
median GT box of 191 px, so these fights run smaller than the corpus the detector was tuned
against. This is the mechanism behind recommendation 1: the boxes are workable in the frame and
marginal by the time they reach a 640 px input.

## The cage hull works, including on Cage 1

One hull per clip: per-pixel median of 60 frames spread across the fight, DeepLab floor mask,
largest connected component, `cv2.convexHull`. The median carries no robots, so the mask has no
robot-shaped holes and the hull has nothing to repair. It adds 0.51 percentage points over the
raw mask on average and never more than 0.82, which is the cleanest version of the hull the
method can produce, as `rembg_cage_high_plan.md` predicted.

| | mean | min | max |
|---|---|---|---|
| hull, share of frame | 67.8% | 61.9% (cage 1) | 77.7% (cage 2) |
| hull minus raw mask | 0.51 pp | | 0.82 pp |
| camera drift over one fight | 0.23 px median | | 3.25 px |

All 20 hulls bound the cage floor and exclude the glass, the crowd behind it, and the structure.
Cage 1 is the notable one: `deeplab_field_data_plan.md` records Cage 1 as a single training frame
and calls it dead as a field, and 12 of these 20 clips are Cage 1. The hull is correct on all of
them. Previews are in `annotated/hull_previews/`.

The stability result is worth keeping. The existing corpus has five cage-high scenes drifting 9
to 37 px, which is why the plan budgeted `phaseCorrelate` plus `warpAffine` before any median
subtraction. Those scenes span a whole event. Within one fight the camera does not move enough
to matter, so a per-fight median needs no alignment at all.

### Read the dashed boxes carefully

Detections are gated on box centre inside the hull, and dropped boxes stay drawn so the two
failure modes stay apart. A box that still overlaps the hull is a robot pinned at the cage wall;
one that overlaps nothing is outside the cage. **78% of the wall-adjacent drops are the house
bot**, parked at the hull edge with 12 px of positional spread on `terminationshock-beedrill`.
Only 1,092 of 4,942 sampled wall drops are `robot`. The headline rates are unaffected, since they
count `robot` boxes only, but the raw `dropped_at_wall_total` column is mostly one stationary
object and should not be read as robots lost at the wall.

## Detector results


Both models at `imgsz 640`, `conf 0.25`. "both%" is the share of frames holding at least
two kept `robot` boxes; "any%" is the share holding at least one. "med robot px" is the
median kept `robot` box side length in the 1080p frame.

| clip | cage | frames | n both% | x both% | any% (n) | hull% | drift px | med robot px |
|---|---|---|---|---|---|---|---|---|
| `apr26_12lb-luna-xenomorph` | 1 | 5,875 | 74.4 | 78.9 | 99.6 | 63 | 0.19 | 170 |
| `apr26_12lb-maximizer-infrared` | 1 | 9,951 | 85.4 | 83.5 | 99.9 | 64 | 0.18 | 103 |
| `apr26_12lb-twelvespeed-riot` | 1 | 11,029 | 86.5 | 91.3 | 99.9 | 63 | 0.08 | 118 |
| `apr26_30lb-hothoney-salimander` | 1 | 6,894 | 92.0 | 97.9 | 100.0 | 63 | 0.36 | 96 |
| `apr26_30lb-hss-pepperoni` | 1 | 4,616 | 91.5 | 93.5 | 99.9 | 63 | 3.16 | 175 |
| `apr26_30lb-termigator-hothoney` | 1 | 3,357 | 99.6 | 96.7 | 100.0 | 62 | 0.38 | 154 |
| `apr26_3lb-juxtaposition-painwin` | 2 | 11,269 | 87.8 | 94.6 | 100.0 | 78 | 0.49 | 119 |
| `apr26_3lbxp-quinquereme-burningmoney` | 2 | 11,269 | 97.5 | 99.7 | 100.0 | 78 | 0.13 | 183 |
| `jun26_3lb-mothership-uchfunt` | 2 | 11,329 | 85.4 | 85.9 | 99.5 | 66 | 0.21 | 121 |
| `jun26_3lb-terminationshock-beedrill` | 2 | 7,493 | 90.2 | 90.0 | 100.0 | 68 | 0.17 | 224 |
| `may26_12lb-bluemarlin-michael` | 1 | 3,897 | 64.1 | 64.3 | 99.9 | 64 | 0.12 | 146 |
| `may26_30lb-parallax-catharsis` | 1 | 7,193 | 83.6 | 85.8 | 100.0 | 64 | 0.24 | 114 |
| `may26_30lb-parallax-emulsifier` | 1 | 4,976 | 86.1 | 92.0 | 99.7 | 64 | 0.26 | 167 |
| `may26_3lb-clyde-badluckv` | 2 | 11,269 | 68.6 | 67.3 | 97.4 | 78 | 3.25 | 192 |
| `may26_3lb-drumderchild-butterfly` | 2 | 11,269 | 85.1 | 86.4 | 100.0 | 78 | 0.22 | 112 |
| `may26_3lb-projectliftoff3-saccharine` | 2 | 11,269 | 91.7 | 68.0 | 99.9 | 77 | 0.54 | 119 |
| `may26pro_12lb-maximizer-caldera12` | 1 | 11,269 | 95.9 | 96.1 | 99.9 | 63 | 0.18 | 102 |
| `may26pro_12lb-robocat-grandeur` | 1 | 11,269 | 97.0 | 93.4 | 100.0 | 62 | 0.15 | 125 |
| `may26pro_12lb-thatsalotoftpu-grandeur` | 1 | 11,269 | 84.5 | 84.8 | 100.0 | 62 | 0.77 | 166 |
| `may26pro_3lb-turbofiend-eruption` | 2 | 11,269 | 84.6 | 83.8 | 99.8 | 77 | 0.32 | 131 |


### yolo26x is not worth its cost here

`yolo26x` averages 86.7% against `yolo26n`'s 86.6%, a mean difference of +0.1 percentage points.
It wins 13 of 20 clips, but the spread runs from -23.8 pp to +6.8 pp, and it is worse on the
metric that matters most for a tracker: its worst fully blind run is 352 frames (5.9 s) against
50 frames (834 ms) for `yolo26n`, and its mean any-robot rate is lower (99.61% against 99.76%).
`projectliftoff3-saccharine` is the clearest case, 91.7% for `yolo26n` and 68.0% for `yolo26x`.

Roughly ten times the parameters and three times the inference time buy nothing. Taken with the
`imgsz` result, that says the ceiling on this footage is set by how much of the robot survives
the resize to a 640 px input, not by model capacity. Spend the compute on input resolution.

### The two hardest clips

`bluemarlin-michael` sits at 64% for both models, the worst in the set, and `clyde-badluckv` at
68.6% is second. `clyde-badluckv` also carries the largest camera movement (3.25 px) and the
lowest any-robot rate (97.4%), so it is the one clip where a median background model would
benefit from the `phaseCorrelate` alignment the earlier plan budgeted. Both are worth watching
before trusting any aggregate.

## Where this lands

The gap-length result points at the same place the current gap-fill work is going: coasting a
prediction through short dropouts, and admitting weak detections only when they associate with
an existing track. This footage says the second half of that is the larger lever on a fixed
camera, provided a cage mask is there to absorb the false positives a low threshold produces.

Two caveats on reusing these numbers. They are `robot`-count rates against a two-robot prior,
not IoU-scored recall, so they bound recall from below and say nothing about box quality. And
the `imgsz` and `conf` sweeps ran on three and four clips; the sign is consistent and the effect
is large, but they are not the full twenty.

## Files

| what | where |
|---|---|
| clips, 1080p | `data/downloads/brettzone_cage_high/*.mp4` |
| provenance, corpus overlap | `data/downloads/brettzone_cage_high/MANIFEST.md` |
| annotated clips | `data/downloads/brettzone_cage_high/annotated/*_yolo26{n,x}.mp4` |
| per-frame counts and boxes | `data/downloads/brettzone_cage_high/annotated/*.dets.json` |
| all clips, all models | `data/downloads/brettzone_cage_high/annotated/summary.json` |
| hull previews | `data/downloads/brettzone_cage_high/annotated/hull_previews/` |
| cached hull polygons | `data/downloads/brettzone_cage_high/*.hull.json` |
| code | `playground/bgsub_cage/` |
