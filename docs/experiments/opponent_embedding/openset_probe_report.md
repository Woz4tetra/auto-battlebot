# Open-set detector probe report: proposals yes, identity no

> Retired 2026-09-05. `openset_probe_predict.py`, `openset_probe_score.py` and the
> outputs under `training/data/openset_probe/` are deleted. Recover the tools with
> `git checkout 8b0c239 -- training/model_eval`. This document is the record.


Zero-shot promptable detectors see the robots the deployed engine is blind to.
OWLv2-large with the fixed text prompt "robot" proposes a box for 97.1% of core
GT opponents (deployed engine ceiling at conf 0.05: 86.1%), and its per-recording
proposal ceiling never drops below 0.80 on any of the 8 recordings, massD and the
stress recordings included. The OOD blind-miss collapse that killed the embedding
probe rescue path (massD: 28 of 53 deployed misses had no proposal at all) mostly
disappears.

Two qualifications keep this from being a drop-in detector:

1. **Score thresholds do not transfer between recordings.** A single global
   threshold reaches uniform recall (worst recording 0.80-0.90) only at 4-12%
   precision. Calibrated per recording, OWLv2-large, OWLv2-base, and OmDet-Turbo
   all hit >= 0.80 opponent recall on every recording, at precision that varies
   0.05-0.76 by venue. Same pathology as the background-subtraction gate: the
   signal separates, the scale shifts.
2. **Text prompts cannot name the opponent, and the exemplar prompt does not
   rescue identity.** "robot" fires on our robot and the house bot too, so
   opponent-only precision is structurally capped near the opponent's share of
   robots on the floor. The single-prompt arm (OWLv2 image-guided detection,
   seeded with the pre-match opponent crops) was killed outright: its similarity
   scores carry no instance identity under match blur, the same failure the
   frozen embedders showed. These models answer "where are the robots";
   "which one is the opponent" still has to come from elimination against our
   known robot track.

SAM 3, the model this probe was designed around, is still unmeasured:
`facebook/sam3` and `facebook/sam3.1` are gated on Hugging Face and this machine
has no authenticated token. `openset_probe_predict.py --methods sam3_text` is
ready to run once access is approved (request at huggingface.co/facebook/sam3,
then `hf auth login`).

## Method

Same data and matching as the embedding probe PR comparison
(`embedding_probe_pr.py`): 688 validated GT frames across 8 eval recordings,
seed frames excluded, greedy matching at IoU >= 0.5, predictions ranked by
score. Two matching modes:

- **strict**: matched against GT opponent boxes only; hits on our robot or the
  house bot count as false positives. Directly comparable to the baseline and
  lowconf rows of the embedding probe report.
- **robot**: matched against all GT robot boxes; recall is still counted on
  opponents only, precision counts any robot hit as a true positive. This is
  what an identity-by-elimination consumer sees from the detector.

Models, all off-the-shelf weights, no training:

| method | model | arm |
|---|---|---|
| owlv2_text | google/owlv2-large-patch14-ensemble | zero-prompt, text "robot" |
| owlv2_base_text | google/owlv2-base-patch16-ensemble | zero-prompt, edge-sized |
| owlv2_exemplar | OWLv2-large image-guided detection | single-prompt, 2 pre-match opponent crops |
| gdino_text | IDEA-Research/grounding-dino-base | zero-prompt |
| omdet_text | omlab/omdet-turbo-swin-tiny-hf | zero-prompt, real-time-sized |
| yoloworld_text | ultralytics yolov8x-worldv2 | zero-prompt |
| sam3_text | facebook/sam3 | blocked, HF gated |

Prompt fixed at "robot" after a sweep on 11-45 only ("robot", "combat robot",
"battlebot", "remote controlled vehicle"): all within 0.05 recall for OWLv2;
multi-word prompts made Grounding DINO spray 250+ boxes per frame for no recall
gain. Zero user prompting in the product sense: the prompt is a constant, not a
per-match input.

```bash
cd training/model_eval
python openset_probe_predict.py --methods owlv2_text,gdino_text,...  # detections
python openset_probe_score.py                                        # pr_openset.csv, tables
```

Outputs live in `training/data/openset_probe/` (per-recording
`detections_<method>.csv` and `manifest_<method>.json`, pooled `pr_openset.csv`).

## Precision/recall vs deployed baseline (strict, core recordings, 490 GT boxes)

Baseline rows reproduced from the embedding probe report. New methods at their
pooled-F1 threshold (a single global operating point, tuned the way the deployed
conf 0.6 was), and at the score floor (the proposal ceiling, analogous to
lowconf).

| Method | Recall | Precision | tp | fp |
|---|---|---|---|---|
| baseline (deployed 0.6) | 67.8% | 43.3% | 332 | 435 |
| lowconf (0.05) | 86.1% | 35.4% | 422 | 771 |
| owlv2_text @ F1 | 61.4% | 40.5% | 301 | 443 |
| owlv2_base_text @ F1 | 59.4% | 48.1% | 291 | 314 |
| omdet_text @ F1 | 53.9% | 42.6% | 264 | 356 |
| gdino_text @ F1 | 48.0% | 41.2% | 235 | 336 |
| yoloworld_text @ F1 | 39.6% | 53.7% | 194 | 167 |
| owlv2_text ceiling (0.02) | 97.1% | 4.7% | 476 | 9701 |
| owlv2_base_text ceiling (0.02) | 97.1% | 4.1% | 476 | 11114 |
| gdino_text ceiling (0.02) | 93.7% | 3.6% | 459 | 12341 |

Reads:

- At a single tuned threshold, zero-shot OWLv2-large pools slightly below the
  deployed engine (61.4/40.5 vs 67.8/43.3). Pooled numbers hide the point,
  though: the baseline earns its pool on the May recordings it was trained near
  and collapses OOD, while the open-vocab models fail differently (see the
  per-recording table). Strict precision is also structurally capped for text
  arms, since every our-robot hit is an FP by construction.
- The ceiling rows are the finding. 97.1% of core GT opponents get a proposal,
  vs 86.1% for the deployed engine at conf 0.05. On massD alone: 93.3% vs 68.5%,
  and deployed misses there were mostly blind. The proposal-starvation wall that
  capped every rescue scheme in the embedding probe is not a wall for these
  models.
- yoloworld_text is a kill: 39.6% pooled at its best, worst recording 4.3%.
  CLIP-distilled open-vocab at YOLO scale does not see these robots.

## Uniformity: recall >= 0.80 on every recording (robot mode)

Per-recording threshold chosen for max precision subject to opponent recall >=
0.80 (models the 10-20 s pre-match calibration window; robot mode, so our-robot
hits are not FPs). Recall first, precision in parentheses.

| Recording | owlv2_text | owlv2_base_text | omdet_text | gdino_text |
|---|---|---|---|---|
| 10-06 | 0.80 (0.06) | 0.80 (0.05) | 0.80 (0.10) | 0.75 (0.08) |
| 11-45 | 0.87 (0.76) | 0.86 (0.73) | 0.80 (0.47) | 0.80 (0.33) |
| 14-12 | 0.80 (0.33) | 0.81 (0.27) | 0.82 (0.37) | 0.80 (0.15) |
| 15-35 | 0.81 (0.27) | 0.81 (0.45) | 0.82 (0.11) | 0.80 (0.13) |
| 16-18 | 0.81 (0.14) | 0.81 (0.34) | 0.81 (0.16) | 0.84 (0.15) |
| 17-26 | 0.82 (0.17) | 0.80 (0.19) | 0.83 (0.29) | 0.74 (0.17) |
| 17-42 | 0.81 (0.72) | 0.82 (0.69) | 0.80 (0.55) | 0.83 (0.56) |
| massd | 0.81 (0.49) | 0.81 (0.48) | 0.81 (0.48) | 0.81 (0.07) |

- Three models make the 80%-everywhere bar with per-venue thresholds:
  OWLv2-large, OWLv2-base, OmDet-Turbo. Grounding DINO misses on 10-06 (0.75)
  and 17-26 (0.74).
- Precision at that recall is the cost, and it is venue-dependent: 0.72-0.76 on
  17-42/11-45, 0.05-0.10 on 10-06 for every model. 10-06 is the recording where
  the arena floor is busiest relative to a single small opponent (20 GT boxes
  across its frames).
- With one global threshold instead (no calibration), min-recording recall stays
  0.80-0.90 for these three models but pooled precision drops to 4-12%. The
  per-venue calibration is load-bearing, exactly as it was for the
  background-subtraction channel.
- At the r80 operating points, 0-61% of the residual FP volume (median ~14%)
  sits in fixed 64 px cells recurring across frames, so full-rate static
  suppression removes some but not most of it. These FPs flicker rather than
  persist, matching the stage-4 finding in the embedding probe. The remaining
  cleanup has to come from motion/depth gating and track-level temporal
  consistency, not from a static-box filter.

## Latency (RTX 4080 Laptop, fp32, median ms/frame)

| method | ms/frame |
|---|---|
| yoloworld_text | 15 |
| omdet_text | 48 |
| owlv2_base_text | 203 |
| gdino_text | 386 |
| owlv2_text | 1113 |
| owlv2_exemplar | 4600-13200 (2-4 queries, unoptimized image-guided path) |

None of these run in the 60 ms perception loop and none need to: the intended
role is a slow anchor detector (0.5-2 Hz) with a cheap tracker propagating
between anchors. OWLv2-base at 203 ms here is the interesting price point;
OmDet at 48 ms is edge-rate but pays for it in per-venue precision.

## Exemplar arm (single prompt): kill

owlv2_exemplar seeds OWLv2-large image-guided detection with the pre-match
opponent crops the embedding probe used (2 seed frames; up to 4 query crops on
17-42). It was the one arm that could have carried opponent identity rather
than robot-ness. It does not:

- At its pooled-F1 operating point, strict recall is 5.7% at 4.8% precision on
  the core recordings. Per recording, 6 of 8 sit at exactly 0.000 recall; only
  11-45 shows any ranking signal (0.278 recall at 0.153 precision).
- Thresholding for 80% recall gives 1-2% precision everywhere except 11-45
  (14%), and 14-12 tops out at 0.66 recall.
- Its proposal ceiling (0.85-0.99, except 0.66 on 14-12) is below the text arm,
  with far higher raw detection volume (up to 58k boxes on one recording).

The query-image similarity carries no more instance identity under match blur
than the frozen dinov2/clip/resnet embedders did. Same verdict, different
mechanism, and at 4.6-13.2 s/frame it is also the slowest arm. Kill the
exemplar path; identity stays with elimination.

## What changes

1. The embedding probe's structural wall (no proposal to rescue) is specific to
   the deployed engine, not to the scene. A slow open-vocab anchor detector is a
   viable detector-independent recall channel: >= 0.80 opponent recall on every
   recording, massD included, with zero training and a constant prompt.
2. Deployable only as a proposal source. Downstream it needs (a) per-venue
   threshold calibration in the pre-match window, (b) motion/depth/static gating
   and track-level fusion to absorb 30-90% venue-dependent FP volume, (c)
   identity by elimination against our known robot track, because text prompts
   cannot name the opponent.
3. SAM 3 remains the untested candidate most likely to beat these numbers (its
   concept prompts add exemplar refinement and its video mode adds temporal
   memory). Unblocking it is a one-time manual step: request access at
   huggingface.co/facebook/sam3, `hf auth login`, then
   `python openset_probe_predict.py --methods sam3_text`.
4. Kill yoloworld_text. Keep omdet_text only if Jetson latency forces it.

## Side finding: deployed miss rates, reframed

The deployed engine's massD recall ceiling at conf 0.05 is 68.5%; OWLv2's is
93.3% on the same frames with no NHRL data in its training set. The 2class
engine's OOD gap is a capacity/data problem of that engine, not evidence that
the robots are unseeable. That strengthens the case for the self-training path
(mine multi-venue footage with motion pseudo-labels) as a parallel fix: the
open-vocab models can label most of what the engine currently misses.
