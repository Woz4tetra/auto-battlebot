# Robot-filter keypoint-override decay — measurement plan (handoff)

Handoff for another agent. Goal: **measure the pros and cons of adding an identity *decay / hysteresis*
to the robot filter's keypoint-override**, so our robot is not briefly emitted as an opponent during
keypoint-detection dropouts. This document is self-contained — read the "Key files & data" section
first, then execute the tiers in order. Each tier has its own section with objective, inputs, method,
metrics, deliverables, and a decision gate.

## Context & motivation

A perception experiment (`docs/experiments/perception_performance/synthetic_plus_bbox_2026-07-22.md`,
plus its interpretability probes) established two things that shape the architecture:

1. The opponent/blob detector generalizes robots by **context** ("a compact fast-moving object on the
   arena floor that isn't us or the house bot"), **not by appearance**. So making the *detector* carry
   our-robots as separate appearance classes does **not** scale — it dilutes the detection signal
   (confirmed earlier by `nhrl_robots_7class` recall collapse 0.675→0.481 and the 84-class `indiv`
   split). Keep the detector generic.
2. Our robots are an **instance** task (fixed CAD, appearance-reliable), best identified by the
   dedicated keypoint model — the one thing that even benefits from synthetic (exact-CAD mrs_buff AP rose
   on both evals).

The scalable design that follows is what production already does: **generic detector + keypoint model
for our robots + a filter that overrides coincident detections.** Adding one of our robots = add its CAD
to the keypoint model and register it in the filter; the detector never changes. This plan validates the
one missing robustness piece in that override — surviving keypoint dropouts.

## The design under test

In `RobotFrontBackSimpleFilter` (`src/robot_filter/robot_front_back_simple_filter.cpp`) the override is
`merge_blob_detections` → `is_blob_suppressed_by_keypoint`: a blob within an adaptive radius of a
**keypoint measurement this frame** is dropped, so our robot isn't double-counted as an opponent blob.

**The gap:** when the keypoint model *misses* our robot on a frame, there is no keypoint measurement, so
the coincident blob is **not** suppressed → it is assigned an opponent `FrameId` and emitted as an
opponent at our robot's location. `RobotTemporalMotionFilter::update_with_prediction` already
dead-reckons OUR_ROBOT_1 forward during such dropouts (command feedback) and flags it `is_stale`, but
that predicted pose is **not** currently used to suppress the coincident blob or to hold the "ours"
identity.

**The proposed decay:** for a `hold_window` after the last keypoint confirmation of a track, keep
treating it as ours — suppress blobs near its predicted/held pose (reuse the `is_blob_suppressed_by_keypoint`
radius against the predicted pose) and keep its label OUR_ROBOT — so short keypoint gaps don't leak an
opponent. This is temporal identity hysteresis on the track.

## Definitions & the core tradeoff

Everything reduces to choosing `hold_window` on a two-sided error tradeoff:

- **Leak (the pro of decay):** a tick where our robot's coincident blob is emitted as an **opponent**
  because keypoints were missed. Self-targeting risk. Decay suppresses these. Lower is better.
- **Over-hold (the con of decay):** a tick where decay wrongly holds the "ours" label — after our robot
  has genuinely left/died, or when a **real opponent** moves into the recently-vacated location within
  `hold_window` (that opponent is then suppressed for up to the window). Also "stale-label latency": how
  long the ours-label persists after true departure. Lower is better.

The whole study is a `hold_window` sweep producing a **leak-vs-over-hold curve**; the knee is the
recommended value. The keypoint-dropout-gap distribution (Tier 0) is what makes leak cheap to fix and
bounds the over-hold cost.

## Key files & data

- **Filter:** `src/robot_filter/robot_front_back_simple_filter.cpp` / `include/robot_filter/…`
  (`merge_blob_detections`, `is_blob_suppressed_by_keypoint`), `robot_temporal_motion_filter.cpp`
  (prediction/stale), `FrameIdAssigner`, `robot_keypoint_tracker.cpp`. Pipeline doc:
  `docs/robot_filter_pipeline.md`.
- **Config:** `config/playback/_playback.toml` + `config/playback/mrs_buff_mk3_playback.toml`
  (`[robot_filter]` section). Any new decay flag goes here (config-driven, per project convention).
- **Diagnostics (already emitted):** `src/runner.cpp` writes a `perception` subsection per tick
  (`their_count_total`, `their_count_live`, `our_present_live`) plus `jump_reject` events, on
  `/diagnostics` (recordings post 2026-06-19). Real camera-frame stamps → real dropout durations.
- **Eval GT:** `training/data/nhrl_keypoints_eval_test` — 372 hand-reviewed frames with our-robot
  front/back keypoint GT, frame-named by SVO `stamp_ns` so it aligns to playback.
- **Existing tools to reuse:**
  - `training/model_eval/perception_reliability.py` — already computes our-robot/opponent validity %
    and dropout-gap length distributions (frames + ms) from a playback MCAP. **Tier 0 base.**
  - `playground/control_stage0/prediction_eval.py` — SVO-playback A/B: two recordings of the same SVO
    differing only in a filter flag, gap-aligned metrics. **Tier 2 template.**
  - `training/model_eval/score.py` — per-frame keypoint detection accuracy on our robot.
- **Playback:** `./scripts/build_and_run.sh -c config/playback.toml` (hardware-free). Recordings:
  `data/recordings/*.mcap`; SVOs under `data/svo` / `/media/storage/true-battlebot-media/svo`.

---

## Tier 0 — Bound the window (cheap; do first)

**Objective.** Decide whether the decay is even worth building, and bound `hold_window`, from the
keypoint-dropout statistics alone. If our-robot keypoint gaps are short and the "blob-present-but-
keypoint-missed" rate is low, the decay is nearly free and a small fixed window suffices.

**Inputs.** A handful of playback MCAPs over our-robot recordings (mrs_buff, mr_stabs) carrying
`/diagnostics`; the `nhrl_keypoints_eval_test` GT for a keypoint-accuracy cross-check.

**Method.**
1. Run `perception_reliability.py` on the recordings to get the **our-robot dropout-gap distribution**
   (`our_present_live == 0` run-lengths, frames + ms) and our-robot validity %.
2. Extend it (small addition) to measure the **leak-opportunity rate**: fraction of ticks where our
   robot is truly present (per GT / recent keypoint history) **and** keypoints are missed **and** a blob
   is present at that location. This is the exact set of ticks the decay would fix. The base tool has
   counts but not per-location overlap — add the "blob near last-our-position while keypoint missed"
   check using the diagnostics + the filter's measurement log (or extend the runner diagnostics to emit
   a `our_blob_present_no_keypoint` flag per tick).
3. Cross-check keypoint miss cause with `score.py` keypoint recall on the eval (is the miss a detection
   miss or an association miss?).

**Metrics.**
- Our-robot keypoint-dropout gap length: median, p90, max (frames and ms).
- Leak-opportunity rate (ticks/second and % of our-present ticks).
- Our-robot validity % (baseline, no decay).

**Deliverables.** A short results note (`docs/experiments/.../robot_filter_decay_tier0_<date>.md`) with
the gap histogram and leak-opportunity rate; any `perception_reliability.py` extension committed.

**Decision gate.**
- If gaps are mostly ≤ a few frames and leak-opportunity is low → the decay is cheap; set
  `hold_window` ≈ p90 gap and proceed straight to a minimal Tier 2 (or ship behind a flag after Tier 1
  confirms no over-hold surprise).
- If gaps are long/frequent → the over-hold cost is real; **Tier 1 is required** to quantify the
  tradeoff before implementing.

---

## Tier 1 — Offline filter simulation & parameter sweep (no C++ change)

**Objective.** Produce the full **leak-vs-over-hold curve as a function of `hold_window`** without
touching the C++ filter, by replaying logged detection streams through a Python model of the
override+decay.

**Inputs.** From one playback pass per recording, log the two per-tick streams the filter consumes:
blob detections (position + size) and keypoint detections (our-robot measurements), plus the field
transform — enough to reproduce `is_blob_suppressed_by_keypoint` and association offline. Get these from
the recording's `/diagnostics` / detection topics, or add a lightweight per-tick dump to the runner.
Use the eval GT (or the keypoint-confirmed track history) as the "is this really our robot" reference.

**Method.**
1. Implement a faithful **Python model of the override + decay**: replicate the suppression radius
   (`radius = max(min_distance, size_scale·(blob.size.x + keypoint.size.x))`, see the cpp) and the
   greedy `FrameIdAssigner` association with the jump gate. The decay adds: hold the last keypoint-
   confirmed our-robot pose for `hold_window`, suppress blobs near it, keep the label ours.
2. **Sweep `hold_window`** over a range (e.g. 0 → 2× the Tier-0 p90 gap). For each value replay all
   recordings and compute the metrics below. `hold_window = 0` reproduces today's behavior (control).
3. Also sweep the suppression radius scale if it interacts.

**Metrics (per `hold_window`).**
- **leak_rate** — fraction of our-present ticks where our blob is emitted as an opponent (↓ = the pro).
- **over_hold_rate** — fraction of ticks the "ours" label persists after true departure (↑ = the con).
- **stale_latency** — mean ms the ours-label persists after true departure.
- **opponent_suppression events** — count/duration where a real opponent within `hold_window` of a
  vacated our-location is suppressed.
- Plot **leak_rate vs over_hold_rate** parameterized by `hold_window`; mark the knee.

**Deliverables.** The sweep script (`playground/control_stage0/decay_sim.py` or
`training/model_eval/…`), a metrics CSV, the leak-vs-over-hold plot, and a results note recommending a
`hold_window` (the knee) or reporting that no window gives acceptable both (→ redesign, e.g.
confidence/velocity-gated hold instead of pure time decay).

**Decision gate.** If a `hold_window` drives leak_rate near zero at negligible over_hold → adopt it and
validate on the real stack (Tier 2). If the curve has no good knee → the time-decay design is
insufficient; document why and propose the alternative before writing C++.

---

## Tier 2 — Playback A/B on the real stack (definitive)

**Objective.** Confirm the Tier-1 recommendation on the actual C++ perception→filter→nav stack,
including timing/latency the offline sim can't capture, and check for navigation regressions.

**Inputs.** The decay implemented in `RobotFrontBackSimpleFilter` behind a **TOML flag** in
`[robot_filter]` (e.g. `keypoint_hold_window_s`, default 0 = off). Real fight recordings that include
our robot and opponent interaction (so both leak and over-hold can occur).

**Method (mirror `prediction_eval.py`).**
1. Implement the decay in C++ behind the flag; suppress blobs near the held/predicted our-robot pose for
   `hold_window` after the last keypoint, and keep the OUR label. Build with
   `./scripts/build_and_test.sh` and add a GoogleTest covering: keypoint miss + coincident blob →
   suppressed & labeled ours within the window; and expiry after the window.
2. Run playback twice over the **same SVO**, differing only in the flag: `--decay-off` (control) and
   `--decay-on` (`hold_window` from Tier 1). Detections are identical between arms, so gaps align and
   ground truth matches — the same trick `prediction_eval.py` relies on.
3. Compute the sequence-level metrics below from `/diagnostics` and the emitted `RobotDescription`
   stream; extend the diagnostics if a needed signal (e.g. per-track label, self-target flag) isn't
   emitted yet.

**Metrics.**
- **Self-target / leak events** — ticks where an opponent track coincides with our robot (decay on vs
  off); the headline safety metric.
- **Our-robot track continuity** — ID switches per minute on OUR_ROBOT_1.
- **Over-hold latency** — ms the ours-label persists after true departure; opponent-suppression events
  near vacated locations.
- **Navigation regression check** — target-selection / nav outcomes unchanged except for the removed
  self-targets (reuse the nav sim sweep from the `eval` skill).

**Deliverables.** The C++ decay behind the flag + unit test; a results writeup
(`docs/experiments/.../robot_filter_decay_2026-*.md`) with the A/B table; a recommendation to ship (flag
default on) or not.

**Decision gate.** Ship (default the flag on) if leak/self-target events drop materially with negligible
over-hold latency and no nav regression. Otherwise keep it off and record the tradeoff.

---

## Suggested order & overall decision flow

1. **Tier 0** always. It often settles the question: short gaps → cheap decay; long gaps → real cost.
2. **Tier 1** whenever Tier 0 shows non-trivial gaps, or before writing any C++ — it picks `hold_window`
   and proves the tradeoff cheaply.
3. **Tier 2** only once Tier 1 recommends a window — it's the expensive one (C++ + build + playback).

## Risks & notes

- **Association vs labeling.** Confirm whether a coincident blob during a keypoint miss currently gets
  our robot's `FrameId` (association) but the wrong *label*, or spawns a new opponent id entirely — the
  decay must fix the one that actually happens. Read `merge_blob_detections` + `FrameIdAssigner`
  closely; the existing prediction already advances `last_position`, which may partly associate.
- **Don't double-count the existing prediction.** `RobotTemporalMotionFilter` already coasts our robot
  during dropouts; the decay is specifically about *suppressing the coincident blob / holding identity*,
  not re-implementing pose prediction. Reuse the predicted pose as the suppression anchor.
- **Time vs count window.** Prefer a time-based `hold_window` (seconds) over a frame count so it's
  robust to frame-rate changes; the diagnostics carry real stamps.
- **Over-hold is the real risk in a fight** (an opponent rushing our robot as keypoints drop). Make sure
  Tier 1/2 recordings include such events, or the con will be under-measured.
