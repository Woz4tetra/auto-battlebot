# Per-robot blob model vs generic baseline, 2026-07-09

Splitting the blob segmentation model into 84 per-robot classes made it worse on every
metric the current test dataset can measure. At a matched confidence floor the generic
5-class model beats the 84-class model on localization mAP, recall ceiling, instance mAP,
and coarse-label confusion. At the deployed 0.6 threshold the new model collapses to 0.41
recall. And the one thing the per-robot model was built for, naming a specific opponent,
is invisible to this test set because the ground truth labels every opponent generically.

Recommendation: keep the generic blob model deployed. Do not ship the per-robot split as-is.
If per-robot naming is the goal, it needs an instance-labeled test set to measure it and a
fix for the training-set class imbalance before a retrain is worth scoring again.

## Models scored

- New: `data/models/yolo26n-seg_nhrl_robots_indiv_2026-07-09_x86_64_sm89.engine`, 84 classes,
  one per robot name. Trained on megamind (`nhrl_robots_indiv`, 109,523 instances).
- Baseline: `data/models/yolo26n-seg_nhrl_robots_2026-04-27_x86_64_sm89.engine`, 5 classes
  (`object, robot, house_bot, mr_stabs_mk2, mrs_buff_mk3`), the currently deployed blob model.

Both are x86_64 sm89 desktop engines built from the same conversion path (`.pt` -> ONNX with
end2end disabled -> TensorRT FP16). Same weights the Jetson would run, different TensorRT
build. Treat this as a model-weights comparison, not a latency or Jetson-parity measurement.

Neither is a keypoint model. The indiv dataset has no `kpt_shape`, so this is a blob (box)
detection comparison only. No keypoint metrics apply.

## Ground truth

`training/data/nhrl_keypoints_eval_test`, 372 frames marked reviewed in `.edit_state.json`,
the same snapshot the 2026-07-07 baseline used. Instance boxes: 461 opponent, 359
mrs_buff_mk3, 206 house_bot, plus a handful of `object` (excluded). The GT vocabulary is
coarse: `mr_stabs_mk2, mrs_buff_mk3, opponent, house_bot, object`. Every non-house opponent
is labeled just `opponent`, with no per-robot identity.

## Label mapping for the 84-class model

`score.py --labels` maps each engine class index to a GT label. The 84 fine classes collapse
to the 5-label GT vocabulary by this rule:

- `Mr Stabs MK2` -> `mr_stabs_mk2`
- `Mrs Buff MK2` -> `mrs_buff_mk3` (our robot; the training labels call it Mk2, the GT calls
  it Mk3, same physical robot)
- `House Bot`, `House Bot Orange Halloween`, `House Bot White Halloween` -> `house_bot`
- `0-0`, `0-0 Part 2` -> `object` (non-robot arena fixtures at the top wall edge, verified by
  cropping instances; `object` is excluded from scoring)
- all other 77 robot names -> `opponent`

The full ordered mapping string is in `assets/2026-07-09_indiv/indiv_labels_map.txt`.

Consequence of the coarse GT: when the indiv model correctly names an opponent (e.g. predicts
`Blindside` on the real Blindside), the map sends it to `opponent` and it scores as correct,
same as a generic detection. The test set cannot reward fine-grained naming. It can only show
the cost of the split.

## Headline: fair, threshold-independent comparison (conf floor 0.05)

`score.py` gates predictions at the confidence threshold before computing mAP, so a fair mAP
needs a low floor for both models. At conf 0.05 the PR curve is fully populated:

| level | metric | generic (old) | indiv (new) |
|---|---|---|---|
| agnostic | mAP50 | **0.862** | 0.785 |
| agnostic | mAP50-95 | **0.583** | 0.518 |
| agnostic | recall ceiling | **0.919** | 0.871 |
| instance | mAP50 | **0.748** | 0.618 |
| instance | mAP50-95 | **0.540** | 0.450 |
| instance | wrong-class rate | **0.194** | 0.290 |

The generic model wins every metric. It localizes more robots (recall ceiling 0.919 vs
0.871), scores higher localization-quality mAP, and assigns the correct coarse label more
often (wrong-class 0.194 vs 0.290). The per-robot split did not just fail to help, it
regressed pure detection.

## Deployed operating point (conf 0.6)

The deployed blob runs at confidence 0.6. Both models at that threshold:

| model | level | mAP50 | precision | recall | F1 | wrong-class |
|---|---|---|---|---|---|---|
| generic | agnostic | 0.664 | 0.943 | 0.675 | 0.787 | 0.000 |
| generic | instance | 0.620 | 0.818 | 0.586 | 0.683 | 0.133 |
| indiv | agnostic | 0.412 | 0.963 | 0.410 | 0.576 | 0.000 |
| indiv | instance | 0.428 | 0.824 | 0.351 | 0.492 | 0.145 |

The indiv model at 0.6 finds only 41% of robots. Precision is very high (0.963) because it
only fires when confident. This is the signature of a confidence distribution shifted low by
84-way classification: the softmax mass spreads across many classes, so per-detection scores
drop and the 0.6 gate rejects most true boxes. The generic baseline reproduces the documented
numbers exactly (agnostic recall 0.675).

## Confidence sweep (indiv model, agnostic)

| conf | mAP50 | precision | recall |
|---|---|---|---|
| 0.05 | 0.785 | 0.546 | 0.871 |
| 0.10 | 0.755 | 0.659 | 0.828 |
| 0.25 | 0.644 | 0.814 | 0.674 |
| 0.40 | 0.551 | 0.903 | 0.561 |
| 0.60 | 0.412 | 0.963 | 0.410 |

To reach the baseline's 0.675 recall the indiv model must drop to conf ~0.25, where its
precision is 0.814 (vs the baseline's 0.943 at 0.6) and its mAP50 is 0.644 (vs 0.664). So even
at a re-tuned operating point that matches recall, the indiv model is slightly worse and
noticeably less precise.

## Per-class coarse identity

Recall by GT class, where "correct" means the right coarse label and "localized" means a box
was found regardless of label:

| class | model / conf | correct recall | localized recall | total |
|---|---|---|---|---|
| opponent | generic 0.6 | 0.375 | 0.518 | 461 |
| opponent | indiv 0.6 | 0.095 | 0.204 | 461 |
| opponent | indiv 0.25 | 0.252 | 0.538 | 461 |
| mrs_buff_mk3 | generic 0.6 | 0.680 | 0.752 | 359 |
| mrs_buff_mk3 | indiv 0.6 | 0.384 | 0.415 | 359 |
| mrs_buff_mk3 | indiv 0.25 | 0.588 | 0.713 | 359 |
| house_bot | generic 0.6 | 0.893 | 0.893 | 206 |
| house_bot | indiv 0.6 | 0.864 | 0.864 | 206 |
| house_bot | indiv 0.25 | 0.908 | 0.913 | 206 |

Two things stand out:

1. **House Bot survives the split, opponents and our robot do not.** House Bot holds ~0.86 to
   0.91 recall at both thresholds because it has 24,992 training instances, by far the most.
   Opponent and mrs_buff_mk3, with far fewer instances per class after the split, collapse at
   0.6 and only partly recover at 0.25.

2. **The indiv model localizes opponents but mislabels their identity.** At conf 0.25 its
   localized-opponent recall (0.538) actually edges out the generic model (0.518), yet its
   correct-opponent recall is far lower (0.252 vs 0.375). More than half of the opponent boxes
   it finds get a wrong coarse label, meaning it tags a generic opponent as our robot or the
   house bot. For aim-assist that specific confusion, opponent vs our own robot, is the worst
   kind of error. The generic model separates the two much better (72% of localized opponents
   labeled correctly vs 47% for indiv).

## Why the split hurt

- **84-way softmax dilutes confidence.** More classes spread the score mass, so detections
  need a lower confidence gate, which lets in more false positives at any matched recall.
- **Severe class imbalance.** House Bot has 24,992 instances; many opponents have 200 to 600
  (`Cavaco` 264, `Elytra` 164, `Iron Warrior` 174, `Flight Controller 2` 111). The model
  learns the dominant classes and starves the rare ones.
- **Fewer examples per class weaken discrimination.** The generic `robot` class pooled all
  opponents into one well-sampled category; splitting it fragments that signal.
- **The test set cannot pay the split back.** Fine naming is the only upside of the indiv
  model, and coarse GT makes it unmeasurable here.

## Caveats

- The GT is coarse. This experiment measures only the downside of the split. To measure the
  upside (correct per-robot naming) the test set would need per-robot opponent labels.
- x86 sm89 engines, not the Jetson aarch64 engines.
- conf 0.6 is the generic model's deployed operating point. The indiv model has no tuned
  threshold; the sweep is the fair way to read it.
- GT labeling was corrected from the deployed generic model's pre-labels, so the baseline has
  a mild home-field advantage on the exact boxes. This does not explain a gap this large.

## Next steps

1. Keep the generic blob model deployed. It is the stronger detector by every measure here.
2. If per-robot identity is still wanted, decide it against the fact that this test set cannot
   score it. Either label a test set with per-robot opponent identities, or drop the per-robot
   ambition for the blob stage.
3. Before any retrain of a per-robot model, fix the imbalance: cap House Bot, and augment or
   oversample the rare opponents. The current 84-class distribution spans 111 to 24,992
   instances.
4. Consider a two-stage design instead of one 84-way head: the generic model for detection
   (it is better at it) plus a separate fine classifier on the cropped box, trained and
   thresholded independently. That keeps detection recall high and isolates the naming problem.

Scores and plots: `assets/2026-07-09_indiv/`. Reproduce with, for the indiv model:

```bash
python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate blob_indiv=data/models/yolo26n-seg_nhrl_robots_indiv_2026-07-09_x86_64_sm89.engine \
  --labels "$(cat docs/experiments/perception_performance/assets/2026-07-09_indiv/indiv_labels_map.txt)" \
  --conf 0.6 --taxonomy training/model_eval/taxonomy.yaml
```

and for the generic baseline:

```bash
python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate blob_generic=data/models/yolo26n-seg_nhrl_robots_2026-04-27_x86_64_sm89.engine \
  --labels opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3 \
  --conf 0.6 --taxonomy training/model_eval/taxonomy.yaml
```
