# Embedding prototype probe report: kill

Frozen off-the-shelf embedders cannot carry opponent identity from a pre-match
seed through the rest of a match at our crop quality. Per the plan's criterion
(every embedder below AUC 0.8 on at least one core recording), this is a kill
for the frozen-prototype approach. Stages 4-5 (full-playback propagation and
overlay rendering) were initially skipped per the plan's early-stop provision,
then run afterward on request; Eval C corroborates the kill (see below).

Two findings survive the kill and redirect the effort:

1. The intra-instance gap is the failure, not the harness. The same opponent
   across the two seed frames, seconds apart, scores as low as 0.525 cosine
   (dinov2 on 14-12). Pose change and motion blur move a robot's embedding
   more than the difference between two robots.
2. Confidence-rescue of any kind is capped well below the target. Of the GT
   opponents the deployed engine misses at conf 0.6, 20-71% per recording have
   no proposal at all even at conf 0.05. On massD the deployed miss rate is
   60% and 28 of the 53 misses are blind. No similarity gate, threshold trick,
   or reranker recovers a box the detector never proposes. Recall on OOD
   opponents needs a channel that does not depend on the detector firing:
   background subtraction and template tracking.

## Method

Per the plan: seed gallery from the opponent boxes of the 2 earliest validated
GT frames per recording, score every eval-frame candidate by max cosine to the
gallery. Candidates are all GT boxes plus deployed-engine proposals
(`yolo26n_nhrl_robots_bbox_2class_mixed_2026-07-31_x86_64_sm89.engine`,
conf 0.05, NMS 0.45, TrtYoloModel path). Embedders: DINOv2 ViT-S/14 at 224,
CLIP ViT-B/32, ResNet-50 ImageNet, all frozen timm weights. 688 validated
frames across 8 recordings; 17-42 is a multi-opponent match, so its gallery
holds 4 crops of 2 distinct robots.

Stage scripts and cached outputs:

```bash
cd training/model_eval
python embedding_probe_extract.py   # candidates.csv, seed/timeline mosaics
python embedding_probe_embed.py     # embeddings_<embedder>.npz per recording
python embedding_probe_score.py     # metrics.csv, figures/, stdout table
python embedding_probe_playback.py  # Eval C: playback_log.csv.gz, eval_c_metrics.json
python embedding_probe_render.py    # overlay.mp4 per recording
```

Outputs live in `training/data/embedding_probe/` (per-recording dirs,
`metrics.csv`, `figures/roc_*.png`, `figures/sim_vs_time.png`).

## Eval A: identity discrimination (pad 10%, gallery seeds)

AUC is opponent GT vs all negatives (GT robots + hard-negative proposals).
Top-1 is the fraction of frames where the most similar candidate is the
opponent. Go bar was AUC >= 0.95 and top-1 >= 90% on all five core recordings.

| Recording | dinov2 AUC | clip AUC | resnet50 AUC | dinov2 top-1 | clip top-1 | resnet50 top-1 |
|---|---|---|---|---|---|---|
| 10-06 | 0.826 | 0.693 | 0.676 | 80% | 65% | 65% |
| 11-45 | 0.765 | 0.772 | 0.770 | 59% | 61% | 63% |
| 14-12 | 0.829 | 0.803 | 0.819 | 71% | 72% | 75% |
| 17-42 | 0.618 | 0.724 | 0.678 | 76% | 77% | 86% |
| massd | 0.820 | 0.923 | 0.922 | 58% | 85% | 84% |
| 15-35 (stress) | 0.749 | 0.889 | 0.891 | 60% | 79% | 77% |
| 16-18 (stress) | 0.825 | 0.668 | 0.658 | 65% | 43% | 37% |
| 17-26 (stress) | 0.752 | 0.842 | 0.838 | 52% | 72% | 78% |

Best core-recording mean is 0.804 (clip, pad 0%). Every embedder has at least
two core recordings below 0.8. No configuration approaches the 0.95 bar.

### Why it fails: no margin against the other robots

Mean similarity to the seed gallery by candidate group (pad 10%):

- Opponent GT vs our-robot GT margins are 0.001 to 0.08. On 10-06, clip
  scores our robot (0.793) above the opponent (0.792). On 17-42, dinov2 gives
  0.617 vs 0.605.
- Opponent vs hard negatives (logos, arena clutter) separates much better:
  dinov2 core-mean AUC 0.886 against that pool alone.

The embedding distinguishes robot-from-background reasonably, robots from each
other barely. The positive control pins the cause: same-robot similarity
across the two seed frames is 0.525-0.966 depending on pose and blur, the same
range as cross-robot similarity. These crops are small, dark, and
motion-blurred; frozen general-purpose features cannot be both invariant to
that and sensitive to instance identity.

## Eval B: false-negative rescue (dinov2, operating point at 2% pooled false accepts)

| Recording | GT opp | deployed misses | rescuable | blind | rescued | FA rate |
|---|---|---|---|---|---|---|
| 10-06 | 20 | 7 | 2 | 5 | 0 | 0.0% |
| 11-45 | 97 | 13 | 10 | 3 | 1 | 0.0% |
| 14-12 | 97 | 25 | 17 | 8 | 2 | 0.0% |
| 17-42 | 187 | 60 | 36 | 24 | 12 | 15.6% |
| massd | 89 | 53 | 25 | 28 | 1 | 0.3% |
| 15-35 | 95 | 23 | 11 | 12 | 5 | 1.7% |
| 16-18 | 68 | 47 | 35 | 12 | 4 | 0.5% |
| 17-26 | 92 | 15 | 8 | 7 | 1 | 2.6% |

The go bar was rescuing >= 50% of rescuable misses; the best core case is
12/36 (17-42), and that operating point spends 15.6% false accepts there
because the two-opponent gallery accepts broadly. Everywhere else rescue is
0-2 boxes. The blind column is the structural result: on the recordings that
matter most (massd 28/53, 17-42 24/60), half the misses have no proposal to
rescue.

## Eval D (hard-negative half): logo suppression

Opponent vs the hard-negative proposal pool alone: dinov2 core-mean AUC 0.886
(per-recording 0.744-0.955). Moderate signal, and the one part of the
prototype idea that works at all. Not worth building alone: stabilized
background subtraction attacks the same false positives without a learned
component.

The static-FP half came back empty from the stage-4 pass: no proposal held the
same 8 px-quantized box for 100+ consecutive frames in any recording. Logo
false positives at conf 0.05 flicker and jitter rather than persisting, so
the "static box = fixture" heuristic finds nothing to pool at this threshold.

## Eval C: full-playback propagation

Run after the kill call, on request. Setup: clip embedder at 0% padding (the
best stage-3 configuration), seed gallery from the same 2 GT frames, deployed
engine at conf 0.05 on every /camera/image frame of each fight mcap, accepting
the top-similarity proposal above the stage-3 operating point (0.886 cosine,
the pooled 2%-false-accept threshold).

One plan deviation: the fight mcaps carry no /blob_detections topic (the
pipeline's detections were never recorded there, only marker topics), so the
agreement-with-recorded-pipeline metric and the dropout-gap coverage metric
are unmeasurable and dropped. Remaining metrics per recording:

| Recording | frames | accepted frames | jump rate per accepted |
|---|---|---|---|
| 10-06 | 14184 | 37.5% | 0.8% |
| 11-45 | 15749 | 8.7% | 0.0% |
| 14-12 | 9878 | 37.6% | 3.0% |
| 17-42 | 6223 | 1.2% | 0.0% |
| massd | 5827 | 14.7% | 0.0% |
| 15-35 (stress) | 7174 | 38.8% | 0.2% |
| 16-18 (stress) | 9290 | 21.5% | 0.0% |
| 17-26 (stress) | 13151 | 19.6% | 0.0% |

The go bar asked for accepted-opponent coverage above the recorded pipeline's
live-rate (under 50%); the best recording reaches 38.8% and 17-42 collapses to
1.2% (its opponent similarities sit almost entirely below the pooled
threshold). The similarity gate either starves coverage at a safe threshold or
sprays false accepts at a permissive one; there is no operating point that
does both. This is the same tradeoff Eval A measured, now visible end to end.

Overlay videos (one per recording, full match, proposals colored by
similarity, accepted box in green, GT flashes white):
`training/data/embedding_probe/<recording>/overlay.mp4`. Per-frame logs are in
`playback_log.csv.gz` and the metrics in `eval_c_metrics.json` next to each
video.

## Precision/recall vs the deployed baseline

Opponent detection scored on the eval frames (seed frames excluded), greedy
matching at IoU >= 0.5, similarity from clip at 0% padding. Methods: baseline
is the deployed engine rule (opponent class, conf >= 0.6); lowconf lowers the
gate to 0.05; simgate accepts any conf >= 0.05 proposal with similarity above
the stage-3 operating point (top1 keeps only the best per frame); union is
baseline plus simgate top-1, the proposed runtime rescue rule. Full table in
`pr_comparison.csv`, curves in `figures/pr_curves.png`; produced by
`embedding_probe_pr.py`.

Core-recording totals (490 GT opponent boxes):

| Method | Recall | Precision | tp | fp |
|---|---|---|---|---|
| baseline (deployed 0.6) | 67.8% | 43.3% | 332 | 435 |
| lowconf (0.05) | 86.1% | 35.4% | 422 | 771 |
| simgate | 10.2% | 76.9% | 50 | 15 |
| simgate top-1 | 9.6% | 75.8% | 47 | 15 |
| union (baseline + top-1) | 69.8% | 43.2% | 342 | 450 |

Reads:

- The similarity gate is precise when it fires (77-100% per recording) but
  fires rarely; it adds 2 points of recall over baseline (union row) at the
  same precision. That is the entire value of the frozen prototype as a
  runtime mechanism, and it does not move 80% toward 99%.
- The engine's own confidence sweep dominates the similarity sweep on the PR
  plane in every recording (`figures/pr_curves.png`): ranking proposals by
  conf beats ranking them by frozen-embedder similarity everywhere. If a
  recall/precision trade is wanted, moving the confidence threshold is
  strictly better than gating on this similarity.
- The baseline numbers are themselves the OOD story: deployed recall is 40.4%
  on massd at 17.3% precision (the logo false positives live here), 30.9% on
  16-18, against 65-87% on the May NHRL fights.
- lowconf's 86.1% recall is the proposal ceiling: the 14 remaining points are
  the blind misses no post-hoc filter can recover.

## Sensitivity

- Padding: 0% context beats 10% and 25% everywhere (core-mean AUC 0.787 vs
  0.772 vs 0.732 for dinov2). More context pulls in arena pixels shared with
  every candidate.
- Gallery vs single mean prototype: within 0.01 AUC. Seeding from 1 frame
  instead of 2 costs 0.03-0.05 AUC. None of it changes the verdict.

## What this changes

1. Detection recall on OOD opponents goes to detector-independent channels:
   stabilized background subtraction (prior art:
   `training/model_eval/background_subtraction_predict.py`) and a pre-match
   template tracker. The blind-miss numbers here are the budget argument.
2. The two-threshold + similarity-gate runtime idea is dropped as a recall
   mechanism. At best it recovers half the misses, and the frozen features
   cannot police the identity side.
3. A metric-learned embedder (the plan's middle-band path) is not ruled out,
   but this probe raises its bar: it must close a near-zero opponent-vs-our-robot
   margin under heavy blur, not polish an existing 0.8. If pursued, train for
   blur/pose invariance on tracklet identity and evaluate with this harness
   (`embedding_probe_score.py` runs unchanged on new embeddings).
4. Per-match seeding worked operationally: 2 frames, 2-4 crops, no gradient
   steps. The capture workflow is worth keeping for whatever replaces the
   embedder (template tracker initialization).

## Side finding: deployed miss rates on the eval set

Deployed-threshold miss rates on GT opponents (same engine and thresholds as
the C++ config): 13-35% on the May NHRL recordings, 60% on massD, 69% on
16-18 (robots embedded together). massD is a different venue with different
lighting; this is the OOD gap measured directly, and most of it is blind
misses, not depressed confidence.
