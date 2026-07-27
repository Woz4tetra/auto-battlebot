# How much labelled data do I need — scenes vs frames

Status: **ready to run** (2026-07-27). Dataset validated, splits built and verified; no training started.

## Question

**How much labelled data does the opponent detector actually need, and does it matter whether that
data comes from more *scenes* or more *frames*?**

Two costs are being separated, because they are paid differently:

- **Frames** cost annotation time. Every box is drawn by hand.
- **Scenes** cost acquisition. You get a whole fight at once, and every frame in it is a
  near-duplicate of its neighbours.

If accuracy tracks frame count, label more of the footage you already have. If it tracks scene count,
labelling more frames per fight is close to wasted and you need more fights. Nothing measured so far
distinguishes these — `category_addition_2026-07-25` graded that question **very weak** because its
two data points differed in class count and time period as well as size.

## Dataset — `nhrl_robots_bbox_2class`, human-validated

| | |
|---|---|
| frames | **31,465** |
| `robot` boxes | 68,104 |
| `house_bot` boxes | 24,347 |
| scenes | 56 |
| `validate_yolo_integrity --strict` | 0 errors, 0 warnings |
| empty labels | 0 |
| synthetic | none |
| storage | standalone copies (no hardlinks, no symlinks) |

Provenance: `/media/storage/auto-battlebots-archive/nhrl_seg_indiv_labeled` → per-scene name-based
remap → top-3-contour box conversion → review-pass filter → **manual validation, 4,146 frames
removed** (`remove_failed_annotations.py`, backed up to `validation_backup/`).

`robot` means *anything in the field that is not the house bot* — every competitor, our own machines,
and the former `object` class. Instance identity is the keypoint model's job.

**Scene sizes are extremely uneven: 9 to 1,363 frames, median 526.** This is what forces the design
below.

## Design — frame-matched, so the comparison isolates diversity

Sampling "50 % of scenes" and "50 % of frames" would **not** be comparable: 28 of 56 scenes could be
anywhere from 7,932 to 23,533 frames depending on which ones are drawn. The two arms would then differ
in volume *and* diversity, and the result would be uninterpretable — the same confound that made the
previous data-floor result unusable.

So both sampling modes hit the **same frame count**; only the number of scenes those frames come from
differs.

### Held-out val — **BUILT**, fixed across every arm

9 scenes, **4,732 frames (15.0 %)** — scene-disjoint from all training and identical for all 5 arms,
so the val curves are directly comparable. Carved once by `make_scaling_splits.py`, never resampled.

**Training pool (the 100 % base): 47 scenes, 26,733 frames.**

### Arms

**BUILT** — `training/data/datascale_2026-07-27/`, realised counts:

| arm | sampling | scenes | frames | % of pool |
|---|---|---|---|---|
| **base100** | all | 47 | 26,733 | 100.0 % |
| **scene75** | whole scenes | 37 | 21,159 | 79.1 % |
| **scene50** | whole scenes | 24 | 13,670 | 51.1 % |
| **rand75** | random frames | 47 | 20,049 | 75.0 % |
| **rand50** | random frames | 47 | 13,366 | 50.0 % |

Scene arms add whole scenes until the frame target is met, so they overshoot slightly — scene75 is
5.5 % larger than rand75, scene50 2.3 % larger than rand50. **That overshoot favours the scene arms**,
so a scene-arm *win* needs discounting by roughly that margin while a scene-arm *loss* is if anything
understated.

**Arms are nested and verified as such:** `scene50 ⊂ scene75 ⊂ base100`, `rand50 ⊂ rand75 ⊂ base100`,
built as prefixes of one fixed shuffle. Each step only adds data, so a drop cannot be blamed on which
scenes were drawn. `make_scaling_splits.py` asserts the subset relations and the val disjointness
before writing, and the build is deterministic (verified by rebuilding and comparing checksums).

Arms are **image-list `.txt` files**, not copied frames: ultralytics accepts a list of image paths for
`train`/`val`, so the five arms cost kilobytes, share one `cache="disk"` `.npy` set next to the source
images, and cannot drift out of sync with the dataset. Verified loading — `rand50.txt` yields 13,366
frames.

**5 arms, cold start from COCO, 100 epochs, `--save-period 25`, current `train.py` defaults**
(`lr0` 0.01, `lrf` 0.1, `degrees` 45, `flipud` 0.0, `close_mosaic` 0, batch 128, imgsz 640).

At ~0.5 s/frame/epoch on 3× A6000 the pool scales roughly linearly: base-100 ≈ 2.0 h, the 75 % arms
≈ 1.5 h each, the 50 % arms ≈ 1.0 h each. **Total ≈ 7 h.**

## What each comparison answers

| comparison | question |
|---|---|
| base-100 vs 75 % vs 50 % | **is there headroom to cut?** — does halving the labelling budget still clear the gate |
| scene-N vs rand-N at matched frames | **scenes or frames?** — if random wins, frames-within-a-fight carry real signal; if they tie, you are paying to label near-duplicates |
| slope of each curve | whether more data still buys anything at 26.7 k frames, or the corpus has saturated |

The scene-vs-random contrast is the load-bearing one. Random sampling draws from all 47 scenes, so it
has **maximum scene diversity at every volume**; scene sampling has the same volume concentrated in
fewer fights. A gap between the curves is a direct measure of what diversity is worth.

## Scoring

Verdict on `nhrl_keypoints_eval_test` (372 reviewed frames, robot-camera footage, scene-disjoint from
all training), paired bootstrap 1000×, conf 0.5.

- **2-class engines** → `--labels "opponent,house_bot"`. All 5 arms share a vocabulary, so **all of
  them plus the deployed baseline score in one paired invocation.**
- **`taxonomy_merged.yaml`** maps `mr_stabs_mk2` / `mrs_buff_mk3` in the eval GT to `opponent`, so the
  GT vocabulary matches what a 2-class model predicts. Without it every detection of our own robot
  counts as a wrong class.
- Baseline anchor must be **re-measured under this taxonomy** before comparing — the familiar
  0.742 / 0.962 / 0.838 was taken with a different label mapping.

**Gate:** earliest ladder checkpoint whose agnostic recall Δ CI lower bound ≥ **−0.04**, with
precision and F1 not significantly worse. Report the earliest clearing checkpoint per arm, not the
best-scoring one — "good enough" means non-inferior to the deployed baseline.

Score the ep{25,50,75,100} ladder for every arm. `category_addition_2026-07-25` found the
fully-annealed endpoint failed the gate for all four of its arms while ep75–100 cleared it comfortably,
so **grading on endpoints alone would systematically understate every arm here.**

## Deliverable

An accuracy-vs-data curve with two lines (scene-sampled, random-sampled) against a horizontal
baseline-parity line, plus a one-line answer of the form *"N frames from M scenes reaches parity;
below that, X."*

## Risks / caveats

- **One seed per arm, and δ = 0.04 is tighter than the 0.048 cross-run spread** Phase A measured
  between two nominally identical runs. Differences smaller than ~0.05 recall between adjacent arms
  are not distinguishable. The *shape* of the curve across 3 volumes is more trustworthy than any
  single pairwise gap; if two arms land close, re-run one with a second seed before concluding.
- **With 25 % dropped, this bounds the floor from above but does not locate it.** If 50 % clears the
  gate, the answer is "the floor is at or below 13.4 k frames" — where exactly is untested. Phase A's
  12 k-frame arm failed all three of its gates on the old contaminated corpus, so the floor plausibly
  sits just below this experiment's smallest arm. Adding a 25 % pair later costs ~1 h and is the
  obvious follow-up if 50 % clears comfortably.
- **Random sampling still draws near-duplicates.** Consecutive frames of one fight are highly
  correlated, so rand-50 is not 13,366 independent samples. This biases *against* finding a
  scene-diversity effect, so a measured gap is a lower bound on the true one.
- **Scene arms are one draw.** Which 25 scenes land in scene-50 matters — a draw heavy in `mini_bot`
  footage differs from one heavy in cage overheads. Fix the seed, record the scene list, and treat a
  single scene-arm result as indicative until repeated.
- **The domain gap is untouched.** Training is NHRL overhead cage footage; the eval is the robot's own
  ZED. This experiment measures how much *of that* footage is needed, not whether it is the right
  footage. A flat curve would mean more overhead data buys nothing — which is a statement about the
  domain gap, not about labelling.
- **`house_bot` balance is checked in the val carve but not in every training arm.** Scene sampling
  can skew it; record the per-arm class balance so a surprising result can be checked against it.
- **154 frames across 6 scenes have a known bad `house_bot` mask** (the Jelly Baby/Anubis field
  annexation and similar). These survived manual validation or were removed by it — confirm which
  before reading any `house_bot` result.

## Artifacts / locations

- Dataset: `training/data/nhrl_robots_bbox_2class/` (validated; failures in `validation_backup/`)
- Splits: `training/data/datascale_<date>/{val,base100,scene75,scene50,rand75,rand50}/`
  plus a manifest recording the scene list and realised frame count of every arm
- Models: `data/models/yolo26n_datascale_<arm>_<date>.{pt,onnx,_x86_64_sm86.engine}`
- Scores: `training/data/nhrl_keypoints_eval_test/scores_datascale/`
- Taxonomy: `training/model_eval/taxonomy_merged.yaml` (to be written)
- Writeup: `docs/experiments/perception_performance/data_scaling_<date>.md`

## Commands

```bash
# splits (already built)
python3 training/yolo/make_scaling_splits.py \
    --src training/data/nhrl_robots_bbox_2class \
    --out training/data/datascale_2026-07-27

# one arm, from training/yolo/
python3 train.py ../data/datascale_2026-07-27/<arm>.yml yolo26n \
    -e 100 --save-period 25 -d 0 1 2
```
