# 7-class archetype blob model vs generic baseline, 2026-07-13

Grouping the blob segmentation model's opponents into four weapon archetypes (plus house_bot and
our two robots) hurts detection less than the 84-class per-robot split did, but it still loses to
the generic 5-class model on this test set. Localization is nearly intact: the archetype model's
recall ceiling is 0.898 vs the generic 0.919, and at a confidence threshold matched for recall its
localization mAP50 (0.676) actually edges the generic (0.664). The cost shows up in two places.
First, seven-way classification dilutes per-detection confidence, so at the deployed 0.6 threshold
recall collapses from 0.675 to 0.481. Second, coarse labeling gets worse: wrong-class rate rises to
0.306 (vs 0.194) and our own mrs_buff_mk3 recall falls hardest. And the one thing the archetype
split was built for, telling a spinner from a control bot from a hammersaw, is invisible here
because the ground truth labels every opponent generically.

Recommendation: keep the generic blob model deployed. The archetype model is a competent detector at
a re-tuned threshold (~0.4), but it offers no measurable upside on this GT and regresses coarse
labeling. To justify the split, label a test set with per-archetype opponent identities so the
weapon-type signal it was built to produce can actually be scored.

## Models scored

- New: `data/models/yolo26n-seg_nhrl_robots_7class_2026-07-13_x86_64_sm89.engine`, 7 classes
  (`horizontal_spinner, vertical_spinner, control, hammersaw, house_bot, mr_stabs_mk2,
  mrs_buff_mk3`). Trained on megamind (`nhrl_robots_archetype`), 450 epochs in 70.6 hours, DDP
  across 3 GPUs. Weights: `yolo26n-seg_nhrl_robots_7class_2026-07-13.pt`.
- Baseline: `data/models/yolo26n-seg_nhrl_robots_2026-04-27_x86_64_sm89.engine`, 5 classes
  (`object, robot, house_bot, mr_stabs_mk2, mrs_buff_mk3`), the currently deployed blob model.

Both are x86_64 sm89 desktop engines built from the same path (`.pt` -> ONNX -> TensorRT FP16). Same
weights the Jetson would run, different TensorRT build. Treat this as a model-weights comparison, not
a latency or Jetson-parity measurement.

Neither is a keypoint model. The archetype dataset has no `kpt_shape`, so this is a blob (box)
detection comparison only. No keypoint metrics apply.

## Ground truth

`training/data/nhrl_keypoints_eval_test`, 372 frames marked reviewed in `.edit_state.json`, the same
snapshot the 2026-07-07 baseline and the 2026-07-09 indiv comparison used. Instance boxes: 461
opponent, 359 mrs_buff_mk3, 206 house_bot, plus a handful of `object` (excluded). The GT has no
mr_stabs_mk2 instances, so its per-class AP reads N/A for both models. The GT vocabulary is coarse:
`mr_stabs_mk2, mrs_buff_mk3, opponent, house_bot, object`. Every non-house opponent is labeled just
`opponent`, with no weapon-type or per-robot identity.

## Label mapping for the 7-class model

`score.py --labels` maps each engine class index to a GT label. The 7 archetype classes collapse to
the GT vocabulary by this rule, in class order:

- `horizontal_spinner`, `vertical_spinner`, `control`, `hammersaw` -> `opponent`
- `house_bot` -> `house_bot`
- `mr_stabs_mk2` -> `mr_stabs_mk2`
- `mrs_buff_mk3` -> `mrs_buff_mk3`

Full string: `opponent,opponent,opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3`.

Consequence of the coarse GT: when the archetype model correctly calls a vertical spinner a
`vertical_spinner`, the map sends it to `opponent` and it scores the same as a generic detection. The
test set cannot reward correct weapon-type naming. It can only show the cost of the split.

## Headline: fair, threshold-independent comparison (conf floor 0.05)

`score.py` gates predictions at the confidence threshold before computing mAP, so a fair mAP needs a
low floor for both models. At conf 0.05 the PR curve is fully populated:

| level | metric | generic (old) | 7-class (new) |
|---|---|---|---|
| agnostic | mAP50 | **0.862** | 0.831 |
| agnostic | mAP50-95 | **0.583** | 0.541 |
| agnostic | recall ceiling | **0.919** | 0.898 |
| instance | mAP50 | **0.748** | 0.626 |
| instance | mAP50-95 | **0.540** | 0.446 |
| instance | wrong-class rate | **0.194** | 0.306 |

The generic model wins every metric, but read the two halves differently. On agnostic (pure
localization) the gap is small: recall ceiling 0.898 vs 0.919, mAP50 0.831 vs 0.862. The archetype
model finds almost as many robots. On instance (localization plus correct coarse label) the gap is
wider: mAP50 0.626 vs 0.748, and wrong-class rate 0.306 vs 0.194. So the seven-way split barely
dents detection but noticeably hurts labeling.

## Deployed operating point (conf 0.6)

The deployed blob runs at confidence 0.6. Both models at that threshold:

| model | level | mAP50 | precision | recall | F1 | wrong-class |
|---|---|---|---|---|---|---|
| generic | agnostic | 0.664 | 0.943 | 0.675 | 0.787 | 0.000 |
| generic | instance | 0.620 | 0.818 | 0.586 | 0.683 | 0.133 |
| 7-class | agnostic | 0.481 | 0.965 | 0.481 | 0.642 | 0.000 |
| 7-class | instance | 0.443 | 0.785 | 0.391 | 0.522 | 0.187 |

The archetype model at 0.6 finds only 48% of robots. Precision is very high (0.965) because it only
fires when confident. This is the same signature the 84-class indiv model showed, milder: seven-way
classification spreads the softmax mass, per-detection scores drop, and the 0.6 gate rejects most
true boxes. The generic baseline reproduces its documented numbers exactly (agnostic recall 0.675).

## Confidence sweep (7-class model, agnostic)

| conf | mAP50 | precision | recall |
|---|---|---|---|
| 0.05 | 0.831 | 0.581 | 0.898 |
| 0.10 | 0.819 | 0.668 | 0.871 |
| 0.20 | 0.784 | 0.769 | 0.823 |
| 0.30 | 0.728 | 0.839 | 0.759 |
| 0.40 | 0.676 | 0.904 | 0.690 |
| 0.50 | 0.585 | 0.943 | 0.594 |
| 0.60 | 0.481 | 0.965 | 0.481 |

To match the baseline's 0.675 recall the archetype model only needs to drop to conf ~0.40, where its
recall is 0.690, its mAP50 is 0.676 (slightly above the baseline's 0.664 at 0.6), and its precision
is 0.904 (vs the baseline's 0.943). So at a re-tuned operating point matched for recall, the archetype
model is an even localizer: as good on mAP50, a little less precise. Its problem at 0.6 is purely the
inherited threshold, not the detector.

## Per-class coarse identity

Recall by GT class at conf 0.6, where "correct" means the right coarse label and "localized" means a
box was found regardless of label:

| class | model | correct recall | localized recall | total |
|---|---|---|---|---|
| opponent | generic | 0.375 | 0.518 | 461 |
| opponent | 7-class | 0.286 | 0.388 | 461 |
| mrs_buff_mk3 | generic | 0.680 | 0.752 | 359 |
| mrs_buff_mk3 | 7-class | 0.248 | 0.373 | 359 |
| house_bot | generic | 0.893 | 0.893 | 206 |
| house_bot | 7-class | 0.874 | 0.874 | 206 |

Two things stand out:

1. **House Bot survives the split, our robot does not.** House Bot holds ~0.87 recall in both models
   because it is a single distinct class with abundant training data. mrs_buff_mk3, split off into its
   own class alongside four archetype classes, collapses from 0.752 localized recall to 0.373: at 0.6
   the archetype model simply misses 225 of its 359 boxes. For aim-assist, dropping our own robot's
   detections that hard is the worst place to pay the cost.
2. **Opponents are localized less and labeled worse.** The archetype model localizes 0.388 of
   opponents vs the generic 0.518, and of the ones it finds it mislabels more (correct 0.286 vs
   0.375). The confusion matrix shows the leakage: opponent boxes it does fire on scatter into
   `mrs_buff_mk3` and `mr_stabs_mk2`, exactly the opponent-vs-our-robot confusion that matters most.

## Why the archetype split hurt (and hurt less than the per-robot split)

- **Seven-way softmax dilutes confidence, but only mildly.** More classes spread the score mass, so
  detections need a lower gate. With seven classes instead of 84 the effect is real but small: at 0.6
  the archetype model keeps 0.481 agnostic recall vs the indiv model's 0.410 and the generic's 0.675.
- **The rare classes starve.** house_bot and the dominant archetypes hold up; mrs_buff_mk3 and the
  thinner opponent archetypes lose recall. This is the class-imbalance signature, less severe than
  the 84-way spread (111 to 24,992 instances) but present.
- **The test set cannot pay the split back.** Weapon-type identity is the whole point of the archetype
  head, and coarse GT makes it unmeasurable. Every archetype scores as `opponent`, so the experiment
  can only see the detection cost, never the classification benefit.

## Caveats

- The GT is coarse. This experiment measures only the downside of the archetype split. To measure the
  upside (correct weapon-type naming) the test set needs per-archetype opponent labels.
- x86 sm89 engines, not the Jetson aarch64 engines.
- conf 0.6 is the generic model's deployed operating point. The archetype model has no tuned
  threshold; the sweep is the fair way to read it, and ~0.4 is where its recall matches the baseline.
- GT labeling was corrected from the deployed generic model's pre-labels, so the baseline has a mild
  home-field advantage on the exact boxes. It does not explain the labeling gap.

## Next steps

1. Keep the generic blob model deployed. It is the stronger detector at the current operating point.
2. If weapon-archetype identity is wanted, label a test set with per-archetype opponent identities.
   Until then this GT cannot tell whether the archetype head learned anything useful.
3. If the archetype model is kept as a detector, re-tune its confidence threshold to ~0.4 to recover
   recall, and re-measure precision/false positives at that point before deploying.
4. Address the mrs_buff_mk3 recall collapse before any retrain: it is our own robot and it is the
   biggest regression here. Check its instance count in `nhrl_robots_archetype` against the archetype
   classes and oversample if it is starved.

Scores and plots: `assets/2026-07-13_7class/`. Reproduce with, for the 7-class model:

```bash
python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate blob_7class=data/models/yolo26n-seg_nhrl_robots_7class_2026-07-13_x86_64_sm89.engine \
  --labels "opponent,opponent,opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3" \
  --conf 0.6 --taxonomy training/model_eval/taxonomy.yaml
```

and for the generic baseline:

```bash
python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate blob_generic=data/models/yolo26n-seg_nhrl_robots_2026-04-27_x86_64_sm89.engine \
  --labels opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3 \
  --conf 0.6 --taxonomy training/model_eval/taxonomy.yaml
```
