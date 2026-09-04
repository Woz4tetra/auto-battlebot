# Embedding prototype probe plan: seed-and-propagate on NHRL May 2026 eval recordings

Test whether a frozen off-the-shelf embedder, seeded with a handful of opponent
crops from the start of a match, can identify that opponent for the rest of the
match. This is the go/kill gate before building anything real (metric-learned
embedder, ReID head on the detector, runtime integration). No training, no new
C++, one afternoon of GPU time on existing assets.

The runtime concept under test: capture an opponent prototype in the 10-20 s
pre-match window, then at match time run the detector at a low confidence
threshold and accept proposals by cosine similarity to the prototype. That
attacks both current failure modes at once:

- False negatives: OOD opponents fire at depressed confidence; similarity
  rescues proposals the objectness gate drops.
- False positives: field logos embed far from any robot prototype; similarity
  rejects them.

Prior art in this repo:

- `training/model_eval/interpret_context_vs_appearance.py` showed the deployed
  detector keys on arena context, not the opponent's own pixels. This probe
  tests the complementary claim: a general-purpose embedder can read instance
  identity from those pixels even though the detector does not.
- `training/model_eval/background_subtraction_predict.py` grades the
  geometry-only channel. The embedding channel is meant to compose with it, not
  replace it.

## Data

Ground truth: `training/data/nhrl_keypoints_eval_test`, 22-100 validated frames
per recording (688 total), classes
`[mr_stabs_mk2, mrs_buff_mk3, opponent, house_bot, object]` (massD's
`data.yaml` has no `object` class). Only frames marked `pass` in
`validation_state.json` count; do not gate on `.edit_state.json`. (`score.py`'s
`reviewed_stems` now prefers `validation_state.json` too and falls back to
`.edit_state.json` only when no validation state exists, so reusing `load_gt`
gates correctly.)

Full playback: the `/camera/image` stream in the full fight mcaps under
`data/saved_recordings/NHRL_2026-05-02/` and
`data/saved_recordings/MassD_2026-08-29/` (the ~1-2 GB
`auto_battlebot_*__<ts>.mcap` files). GT image filenames are the
`/camera/image` header `stamp_ns`, so GT frames key directly into the mcap
stream. Everything in this probe reads mcap frames; no SVO decode, which also
sidesteps the desktop rectification warp (playback frames are affine-warped
~2-3% vs live frames; mcap frames are the live frames).

One opponent per recording, so per-recording metrics are per-opponent grades:

| Recording | Opponent | Role | Notes |
|---|---|---|---|
| `2026-05-02_10-06-02` | clyde | core | |
| `2026-05-02_11-45-05` | sphinx | core | clean fight |
| `2026-05-02_14-12-25` | wreckcreation | core | clean fight |
| `2026-05-02_15-35-00` | ironwarrior | stress | our robot inverted most of match; tests seed robustness when the scene is chaotic |
| `2026-05-02_16-18-05` | (16-18 opponent) | stress | robots embedded together; occlusion test |
| `2026-05-02_17-26-12` | (17-26 opponent) | stress | heavy damage; appearance-drift test |
| `2026-05-01_17-42-20` | (17-42 opponent) | core | long recording, GT frames spread over time |
| massd 2026-08-29 | massD event opponent(s) | core | real match footage, 98 validated frames, 91 opponent boxes |

Core recordings gate the go/kill decision. The massD recording spans 12
minutes and may contain more than one fight; check this during extraction, and
if so split it into per-fight segments and seed each segment from its own
earliest validated frames (the pre-match capture is per-match in the real
workflow anyway). Stress recordings are reported but
expected to be worse; they tell us what the real system needs on top of the
prototype (gallery updates, occlusion handling), not whether the idea works.

## Seed protocol

Simulates the pre-match capture:

1. Take the earliest 2 validated GT frames of the recording (or fight
   segment).
2. Crop the `opponent` boxes (1-2 crops per frame), padded 10% for context,
   from the mcap frame at that stamp.
3. Embed all seed crops, L2-normalize, keep them all as the gallery (2-4
   vectors). Match score for any candidate crop = max cosine over the gallery.

Seed frames are excluded from all evaluation. Sensitivity variants (cheap once
embeddings are cached): 0% and 25% padding, single-mean prototype vs gallery,
seeding from 1 frame only.

## Embedders

All frozen, offline, dev GPU, no latency constraint. The question is whether
appearance carries the signal at all, so use the strongest available features
plus a small-model floor:

- DINOv2 ViT-S/14 (best available instance features, primary candidate)
- CLIP ViT-B/32 image encoder
- OSNet or ImageNet ResNet-50 pooled features (floor; approximates what a
  small deployable net could reach)

Install probe deps (`torch` hub / `timm` / `open_clip`) into `venv/` only. Do
not touch `pyproject.toml` for this probe; deps get added if the approach
graduates.

## Candidate crops

Two pools per recording:

- GT boxes on evaluation frames, all classes. `opponent` boxes are positives;
  `mr_stabs_mk2` / `mrs_buff_mk3` / `house_bot` / `object` boxes are labeled
  negatives.
- Detector proposals: run the deployed bbox engine through the `score.py`
  preprocessing + NMS path at `--conf 0.05` on the same frames. Proposals with
  IoU < 0.3 against every GT box are hard negatives (this pool contains the
  field-logo FPs). Proposals overlapping a GT opponent are the FN-rescue
  candidates for Eval B.

## Evaluations

### Eval A: identity discrimination on held-out GT frames

For each recording, score every candidate crop on the remaining validated
frames against the seed gallery.

- ROC AUC, opponent vs all negatives (GT negatives + hard negatives), per
  recording.
- Top-1 identity rate: fraction of evaluation frames where the
  highest-similarity candidate is the opponent.
- Similarity vs match time: does the opponent's score decay as damage
  accumulates? A steep decay on 17-26 quantifies how much mid-match gallery
  updating the real system needs.

### Eval B: false-negative rescue

The payoff metric. On evaluation frames:

1. Find GT opponents missed by the engine at the deployed threshold.
2. For each miss, check whether a low-conf proposal (conf >= 0.05, IoU >= 0.5
   with the GT box) exists. If none exists, the detector is truly blind there
   and no threshold trick helps; count these separately, they bound what the
   prototype can recover.
3. Pick one similarity operating point per embedder (from Eval A ROC) and
   report: misses rescued, false accepts added, per recording.

### Eval C: full-playback propagation

The "rest of the match" ask. No GT here, so the assessment is consistency plus
eyeball:

1. Stream every `/camera/image` frame of the fight mcap (a few thousand frames
   per fight). Run the engine at conf 0.05, embed proposals, accept the
   top-similarity proposal above the Eval B operating point.
2. Metrics per recording:
   - Accepted-opponent frame fraction, compared against the recorded
     pipeline's opponent live-rate on the same frames (baseline is under 50%).
   - Frame-to-frame jump rate of the accepted box (teleports = identity
     swaps or logo locks).
   - Agreement with recorded `/blob_detections` opponent boxes where the
     recorded pipeline did fire (IoU >= 0.5 rate). Disagreement frames are
     exactly the interesting ones; dump them as crops for review.
   - Accepted fraction inside recorded dropout gaps, since coverage there is
     the entire point.
3. Render an overlay video per recording with similarity-colored proposal
   boxes and the accepted box highlighted.

### Eval D: logo suppression

Static-FP pool: hard negatives from Eval A plus any full-playback proposal
whose box stays within 5 px of the same location for 100+ frames (logos and
arena fixtures self-identify by not moving). Report AUC of opponent vs
static-FP pool and the false-accept rate of the Eval B operating point on this
pool.

## Success criteria

- Go: best embedder reaches AUC >= 0.95 and top-1 >= 90% (Eval A) on all five
  core recordings, and Eval B rescues >= 50% of rescuable misses at <= 2 added
  false accepts per 100 evaluated candidate crops, and the Eval C overlay
  shows no sustained lock onto our robot or a logo.
- Kill: every embedder is below AUC 0.8 on the core recordings. Appearance at
  this crop resolution does not carry instance identity; drop the prototype
  approach and put the effort into background subtraction and template
  tracking instead.
- Middle band (0.8-0.95): signal exists but frozen generic features are not
  enough. Proceed to a small metric-learned embedder trained on tracklet
  identity (separate plan), using this probe's harness as its eval.

Secondary read regardless of outcome: if the ResNet/OSNet floor lands close to
DINOv2, a deployable-size net can carry the signal and the runtime latency
story is easy. If only DINOv2 works, distillation becomes part of the follow-up
plan.

## Steps

One script per stage, each with inspectable output, all under
`training/model_eval/`. Outputs go to `training/data/embedding_probe/`
(derived data; regenerate in place, never delete the directory).

1. `embedding_probe_extract.py`: walk the eval dataset + fight mcaps, write
   crops and metadata (recording, stamp_ns, source [gt-class|proposal], conf,
   box, IoU-to-GT) for seed frames, evaluation frames, and the full-playback
   proposal stream. Engine inference reuses the `score.py` preprocessing/NMS
   path.
2. `embedding_probe_embed.py`: embed all crops with each embedder, cache
   `.npz` per (recording, embedder).
3. `embedding_probe_score.py`: Evals A, B, D. Prints the per-recording metric
   table, saves ROC and similarity-vs-time figures.
4. `embedding_probe_playback.py`: Eval C metrics + disagreement crop dumps.
5. `embedding_probe_render.py`: overlay videos.
6. Write `embedding_prototype_probe_report.md` next to this plan with the
   tables, figures, and the go/kill call.

Stages 1-3 alone answer go/kill; run them first and stop early on a kill.

## Caveats

- Frame validity comes from `validation_state.json` (entries whose value is
  `pass`), keyed by `<subdataset>/images/<stamp_ns>.png`. `score.py`'s `load_gt`
  applies this already; `.edit_state.json` is only its fallback when no
  validation state exists.
- GT stamp filenames key into the mcap `/camera/image` stream directly; never
  join by nearest timestamp.
- `--labels` order must match the deployed engine's class count when reusing
  the `score.py` inference path (wrong length silently misparses the tensor).
- Do not write anything under `data/`.
- Activate `venv/` (`source scripts/activate_python.sh`) before running any
  stage.

## Next steps

1. Review this plan.
2. Implement stages 1-3, run on the five core recordings.
3. Make the go/kill call; on go, run stages 4-5 and write the report.
