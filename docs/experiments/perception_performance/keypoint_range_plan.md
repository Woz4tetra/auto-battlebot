# Keypoint accuracy beyond 1.8 m — why it fails and what to try

Status: **not started** (2026-07-29). No code written, no runs launched. Phase 0 is a prerequisite for
grading anything below it.

## Question

**Why does the keypoint model degrade past ~1.8 m, and which interventions actually recover heading
accuracy at range?**

The deployed `yolo26n-pose_our_robots_2026-05-01` scores 9.0 deg mean heading error and 0.797
`heading_acc@10deg` on `nhrl_keypoints_eval_test` (`deploy_keypoints_2026-07-16.md`). Those are
**whole-set means dominated by close-range instances**. Nothing measured so far reports accuracy as a
function of range, and the geometry below says the far half of the set must be much worse than 9.0 deg.

## Why it fails — the pixel budget, measured

This is not a hypothesis. Every number here is measured from the repo or the eval labels.

| quantity | value | source |
|---|---|---|
| camera `fx` @ 1280×720 | **531.53** | `/usr/local/zed/settings/SN33234316.conf`, `[LEFT_CAM_HD]` |
| mrs_buff front→back baseline | **0.1528 m** | `training/synthetic/config.toml`, `[robots.keypoints]` |
| mr_stabs front→back baseline | **0.1321 m** | same |
| engine input | **640×640 square letterbox** | `data/models/yolo26n-pose_our_robots_2026-05-01.onnx` |
| effective downscale of 1280×720 | **×0.5** | `min(640/1280, 640/720)` |

The heading measurement is the angle of a **0.1528 m** vector. Its apparent length is
`sep_px = fx · 0.1528 / d`, and for independent per-axis keypoint noise σ the heading error is
`σ_θ ≈ √2 · σ / sep_px`:

| distance | sep @1280×720 | sep @network input (×0.5) | σ_θ if σ=2 src px | if σ=4 src px |
|---|---|---|---|---|
| 1.0 m | 81 px | 41 px | 2.0° | 4.0° |
| **1.8 m** | **45 px** | **23 px** | **3.6°** | **7.2°** |
| 2.5 m | 33 px | 16 px | 5.0° | 10.0° |
| 3.0 m | 27 px | 14 px | 6.0° | 12.0° |
| 4.0 m | 20 px | 10 px | 8.0° | 16.0° |

**1.8 m is where the vector drops below ~23 px at network resolution.** Past 2.5 m the model is being
asked to resolve the orientation of a 14-pixel segment. The chosen 1.8 m threshold is not arbitrary —
it is where the pixel budget runs out.

### The loss is not the cause

Checked directly, ultralytics 8.4.9 (`venv/.../ultralytics/utils/loss.py`):

- `sigmas = torch.ones(nkpt) / nkpt` → **0.5 per keypoint** for `kpt_shape [2,3]` (line 638).
- `e = d / ((2σ)² · area · 2)` (line 329) — normalized by **box area**, so the OKS term is scale
  invariant and far/small objects are **not** down-weighted. At realistic errors `e ≪ 1`, so
  `1 − exp(−e)` is in its linear regime and does not saturate; saturation would need ~45 input px of
  error.

Per unit squared pixel error a far object actually receives *more* gradient than a near one
(`1/(2·area)` scales inversely with area). **Do not spend time retuning `pose`, `kobj`, or the sigmas
as a range fix.** The binding constraint is information, not loss weighting.

### The eval set already covers the range of interest

Measured over all 389 `mrs_buff_mk3` GT instances in `nhrl_keypoints_eval_test` (622 label files,
1280×720):

| proxy | frac > 1.8 m | frac > 2.5 m | median implied distance |
|---|---|---|---|
| kp separation (`d = fx·0.1528/sep`) | **0.43** | 0.25 | 1.64 m |
| box width (`d = fx·0.30/box_w`) | **0.47** | 0.16 | 1.72 m |

**~175 instances beyond 1.8 m.** That is enough to stratify on without collecting anything new. Both
proxies agree; the kp-separation proxy is inflated by yaw foreshortening, which is why Phase 0 should
recover true depth instead.

`mrs_buff_mk3` box size percentiles (px, @1280×720) for calibrating any crop design:
`box_w` p5/25/50/75/95 = 55 / 72 / 93 / 138 / 180; `box_h` = 28 / 38 / 52 / 82 / 123.

## Phase 0 — measurement (prerequisite, blocks everything)

`score.py` reports `kp_err_px`, `kp_pck@0.1`, `kp_heading_err_deg`, `kp_heading_acc@10deg` as
**whole-set means** (`KEYPOINT_METRICS`, line 87). There is no distance or size stratification. Until
that exists, a far-range improvement is invisible — it is diluted by the near instances that already
work.

Three additions to `training/model_eval/score.py`:

1. **Distance bins.** `<1.2 / 1.2–1.8 / 1.8–2.5 / >2.5 m`, reported for every keypoint metric **and
   for recall**. Preferred distance source is the ZED depth from the source recording — each eval
   subdir's `data.yaml` carries `source_mcap`, so per-instance depth at the box centroid is
   recoverable. Fall back to `d = fx·0.1528/sep_gt` only if the MCAP path is expensive; record which
   was used, because the proxy is yaw-biased.
2. **Common-mode vs differential-mode keypoint error.** The baseline reports `kp_err_px` 9.8 but
   `heading_err` 9.0° — that combination is only possible if the two keypoint errors are strongly
   correlated, i.e. mostly a **translation of the pair**, which is harmless for heading. Split the
   error into the component shared by both keypoints (translation) and the component that rotates the
   vector (differential). **Differential error is the quantity that costs heading**; optimizing
   `kp_err_px` optimizes partly the wrong thing.
3. **Per-bin recall alongside per-bin heading.** A far-range miss and a far-range sloppy heading need
   different fixes. `all_robots_pose_2026-07-14.md` already showed box recall and keypoint precision
   move independently.

Extend the existing bootstrap to the binned metrics so per-bin deltas carry CIs — with ~175 far
instances the far bin is noisier than the whole-set numbers everyone is used to reading.

**Deliverable:** a re-scored deployed baseline, binned. This becomes the reference every phase below is
graded against, and it is the first honest statement of how bad range performance actually is.

## Phase 1 — GT noise floor at range (cheap, defines the target)

At 45 px separation, a human placing front/back to ±2–3 px is **4–7° of irreducible heading noise at
1.8 m and 8–12° at 3 m**. If the GT floor in the far bin is 12°, that bin cannot validate a "<10° at
3 m" target and every later phase would be chasing label noise.

Blind double-label ~50 far-range eval frames (or re-label with `training/model_eval/edit_labels.py` and
compare to the existing labels), and report annotator disagreement **per distance bin**.

If the floor is too high to grade against, the fallback is to re-derive far GT geometrically — fit the
CAD model against ZED depth rather than clicking pixels — but do not start that until the measurement
says it is needed.

**Gate:** far-bin GT disagreement must be comfortably below the improvement any later phase claims.
Adjust the target to the floor, do not pretend the floor is zero.

## Phase 2 — stop spending 44 % of the input on padding

**640×640 on 16:9 input puts the image in 640×360 and pads 280 rows.** 44 % of the input tensor is
grey. That compute is paid on every frame for nothing.

The C++ side already handles this: `YoloKeypointModel` reads the input size from the engine tensor
shape (`src/keypoint_model/yolo_keypoint_model.cpp:63`) and the letterbox / `scale_boxes` /
`scale_keypoint` math is fully general in width and height (lines 120, 300, 318). **A rectangular
engine is a drop-in with no C++ change** — worth verifying with one playback run, but no code is
expected.

| config | tensor px | vs 640² | letterbox gain | sep @1.8 m | sep @3.0 m |
|---|---|---|---|---|---|
| 640×640 (today) | 409,600 | 1.00 | ×0.50 | 23 px | 14 px |
| 640×384 | 245,760 | **0.60** | ×0.50 | 23 px | 14 px |
| **832×480** | 399,360 | **0.98** | **×0.65** | **29 px** | **18 px** |
| 960×544 | 522,240 | 1.27 | ×0.75 | 34 px | 20 px |

Two arms:

- **2a — 640×384.** Identical detail, ~40 % less compute. A pure latency refund; useful on its own if
  the Jetson budget is tight.
- **2b — 832×480.** *Same cost as today* and ×1.3 linear resolution on the target. This is the
  interesting arm: it should move each distance bin roughly one column left in the geometry table for
  free.

`train.py` hardcodes `imgsz` per model in the `configs` dict (lines 55–100) and has **no `--imgsz`
CLI flag** — a small argparse addition accepting one or two ints is needed. `convert_to_onnx.py`
already takes `--imgsz`.

**Gate:** 2b must not regress the near bins, and must improve far-bin heading. If 2b wins, re-measure
Jetson latency before assuming it is free — the anchor count is similar (8,190 vs 8,400) but memory
traffic is not identical.

## Phase 3 — widen the keypoint baseline

Heading error is inversely proportional to keypoint separation, and the current design uses the
**shortest available baseline**: 0.1528 m front-to-back centroid on a robot whose full extent is
~0.30 m. Placing keypoints at the widest CAD features (weapon tips, chassis corners) and fitting
heading over 4+ points **roughly halves heading error at every distance**, and averaging over more
points further suppresses the differential component.

Labels come from CAD — `[robots.keypoints]` in `training/synthetic/config.toml` — so **synthetic
regeneration costs nothing but render time**. Real frames would need relabeling, which is why this runs
as a synthetic-train / synthetic-val ablation first to size the win before committing annotation
effort.

Downstream this touches `kpt_shape`, `flip_idx`, and `front_back_keypoint_converter.cpp`. **Do not
touch the C++ during the ablation** — grade it offline first.

**Gate:** measured far-bin heading improvement on synthetic val of at least the ~2× the geometry
predicts. If it comes in well under, something in the head or the assigner is the limit, not the
baseline length, and Phase 5 becomes more attractive.

## Phase 4 — crop-and-zoom second stage

Standard top-down pose. Crop a padded ROI around the box, resize to **192×192**, run a small pose model
on the crop.

At 1.8 m the box is ~89×45 src px; a 128 px ROI resized to 192 is a **×1.5 upsample against today's
×0.5 downsample — ×3 linear, ~6× the pixels**. Critically, the crop **holds separation roughly constant
with range** (~68 px at both 1.8 m and 3.0 m) instead of letting it fall off as 1/d. That is the whole
point of the approach.

Cost is `(192/640)² ≈ 9 %` of a full-frame pass — order 1–2 ms on the Jetson — but it is **serial**,
added after detection, unlike the current parallel `parallel_model_batch` arrangement. Budget it
against the 60 ms end-to-end target explicitly.

Two variants:

- **4a — refinement pass.** Keep the full-frame model for detection unchanged; run the crop pass only
  on our-robot boxes and take its keypoints. No behavior change if it fails; easy to A/B in playback.
- **4b — track-driven ROI.** Predict the ROI from the previous frame's track (`robot_keypoint_tracker`)
  and run crop-only, re-acquiring full-frame every N frames or on track loss. Cheaper steady-state, but
  adds a re-acquisition failure mode.

Use the ZED depth to set crop scale adaptively — the expected pixel size at a known range is exactly
the table above.

**Start with 4a.** 4b is an optimization of a thing that has not been shown to work yet.

## Phase 5 — data realism at range, and capacity

Lower confidence than 2–4; run only if the far bin is still short after them.

- **5a — far-range render realism.** The camera sampler already draws distance **uniformly over
  [0.3, 6.0] m** (`synthgen/camera.py:32`), so ~70 % of renders are already past 1.8 m — this is
  probably **not** a coverage problem. It is more likely realism: a synthetic robot at 4 m is crisp,
  a real one is smeared by ZED ISP denoise, sensor noise, and motion. Motion blur is currently a flat
  0.3 probability independent of range. Make degradation distance-correlated and re-measure.
- **5b — real far-range coverage.** With only ~360–500 real mrs_buff frames total
  (`all_robots_pose_2026-07-14.md`), the far bin may have very few real examples anchoring the domain.
  Count them before assuming synthetic is the problem.
- **5c — capacity ladder.** P2 (stride-4) head → `yolo26s-pose` → `yolo26m-pose`.
  `deploy_keypoints_2026-07-16.md` established the nano pose head plateaus at epoch 16 of 150, so
  capacity is a real ceiling — but Phases 2 and 4 buy more resolution per millisecond than a bigger
  backbone does. **Check Jetson latency first.**

## Phase 6 — bank the residual outside the model

Heading from a 0.15 m baseline at 3 m is fundamentally a noisy per-frame measurement, and no model
change removes that. `robot_temporal_motion_filter` can fuse velocity direction with keypoint heading
and weight the keypoint measurement by its expected variance — **the geometry table above is a closed
-form noise model, parameterized by the range the ZED already reports.**

This does not improve the model and should not be graded as if it did, but it is the correct place to
spend whatever error survives Phases 2–4.

## Order and expected payoff

| phase | effort | expected far-bin gain | blocks |
|---|---|---|---|
| 0 — binned scoring | ~0.5 day | measurement only | everything |
| 1 — GT noise floor | ~0.5 day | defines the target | grading 2–5 |
| 2b — 832×480 rect input | 1 train run | ~×1.3 resolution → ~25 % heading err | — |
| 3 — wider baseline (synthetic ablation) | 1 render + 1 train | ~×2 on heading | — |
| 4a — 192² crop refinement | 2–3 days incl. C++ | ~×3 resolution, largest single win | — |
| 5 — realism / capacity | 1 render + 1 train each | unknown, likely modest | — |
| 6 — temporal fusion | 1–2 days | banks the residual | — |

**2b and 3 are single training runs against the existing harness and need no C++ work.** Run them
before starting Phase 4. If they land as the geometry predicts they roughly quarter far-range heading
error between them, which may make Phase 4 unnecessary.

## Deliverable

A heading-error-vs-distance curve, one line per arm, against the deployed baseline and a horizontal
GT-noise-floor line from Phase 1, plus a one-line answer of the form *"heading stays under X deg out to
Y m with recipe Z."*

## Risks / caveats

- **Phase 0 changes what every prior number means.** The familiar 9.0 deg / 0.797 are whole-set means
  over a set that is ~45 % far-range. Re-scoring the baseline binned will make the deployed model look
  worse than the writeups suggest. That is the point, but it should be stated plainly in the writeup so
  the numbers are not read as a regression.
- **The far bin is ~175 instances, not 389.** Bootstrap CIs there will be materially wider than the
  whole-set CIs the previous experiments reported. `data_scaling_2026-07-27.md` measured ~0.048 recall
  spread between nominally identical runs on the *full* set; the far-bin spread will be worse. Do not
  call a far-bin delta on one seed.
- **Distance proxies are biased.** `kp_sep` shrinks with yaw foreshortening as well as distance, so it
  *overstates* distance for robots pointing at or away from the camera. The two proxies agreeing to
  within 0.04 on the >1.8 m fraction is reassuring, not proof. Prefer real depth.
- **GT noise may dominate the far bin.** Phase 1 exists precisely to find this out before Phases 2–5
  are graded against numbers that cannot support them.
- **The 1.8 m threshold is a proxy for a control requirement that is not written down.** How much
  heading error the navigation stack actually tolerates at 3 m has not been measured. Worth pinning
  down — the target may be looser (or tighter) than 10 deg.
- **Phase 2's "free" claim assumes latency scales with tensor pixels.** It roughly does for these
  shapes, but confirm on the Jetson rather than the dev GPU before treating 832×480 as cost-neutral.
- **Phase 4 adds serial latency** to a pipeline whose models currently run in parallel
  (`parallel_model_batch`). The 9 % FLOP figure is not a 9 % wall-clock figure.
- **Phase 3 changes the C++ contract** (`kpt_shape`, `flip_idx`,
  `front_back_keypoint_converter.cpp`). Keep it offline until the ablation justifies the change.
- **No mr_stabs in the eval set** (`deploy_keypoints_2026-07-16.md`). Every keypoint number here is
  mrs_buff only, and mr_stabs has a *shorter* baseline (0.1321 m), so its range behavior is worse than
  anything measured and entirely untested.

## Artifacts / locations

- Eval set: `training/data/nhrl_keypoints_eval_test/` (372 reviewed frames; 622 label files on disk)
- Scoring: `training/model_eval/score.py`, `taxonomy_keypoint.yaml` (excludes opponents)
- Baseline: `data/models/yolo26n-pose_our_robots_2026-05-01.{pt,onnx,_x86_64_sm89.engine}`
- Synthetic config: `training/synthetic/config.toml` (`[camera]`, `[[robots]].keypoints`)
- Training: `training/yolo/train.py` (needs an `--imgsz` flag for Phase 2)
- Scores: `training/data/nhrl_keypoints_eval_test/scores_kprange_<arm>/`
- Writeup: `docs/experiments/perception_performance/keypoint_range_<date>.md`

## Commands

```bash
# Phase 0 — re-score the deployed baseline, binned (after score.py gains --distance-bins)
venv/bin/python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate pose_our=data/models/yolo26n-pose_our_robots_2026-05-01_x86_64_sm89.engine \
  --labels "mr_stabs_mk2,mrs_buff_mk3" \
  --taxonomy training/model_eval/taxonomy_keypoint.yaml --conf 0.5 \
  --distance-bins "0,1.2,1.8,2.5,inf" \
  --output training/data/nhrl_keypoints_eval_test/scores_kprange_baseline

# Phase 2b — rectangular input (after train.py gains --imgsz)
venv/bin/python training/yolo/train.py \
  training/data/our_robot_keypoints/data.yml yolo26n-pose --imgsz 480 832 -e 500

venv/bin/python training/yolo/convert_to_onnx.py \
  data/models/yolo26n-pose_kprange_rect_<date>.pt --imgsz 480 832
venv/bin/python training/yolo/convert_to_tensorrt.py \
  data/models/yolo26n-pose_kprange_rect_<date>.onnx --workspace 1
```
