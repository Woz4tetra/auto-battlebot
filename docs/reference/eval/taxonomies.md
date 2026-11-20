# Eval taxonomies

`training/model_eval/score.py` reads one taxonomy YAML per scoring run (`--taxonomy`). Each file has two keys, `archetypes` and `exclude`, and defines a scoring view over the same ground truth.

## `training/model_eval/taxonomy.yaml`

The default view.

| Key | Meaning |
| --- | --- |
| `archetypes` | Maps a fine (instance-level) label to its NHRL archetype for the `archetype` scoring level. Unlisted labels map to themselves. Label GT at the finest granularity, one label per robot instance; this file collapses it for coarser comparisons. A per-instance entry looks like `lynx: horizontal_spinner` or `deathroll: vertical_spinner`. |
| `exclude` | Labels removed from both GT and predictions before scoring. `object` is GT-only debris and no deployed model has an `object` class. Add our own robots here to score opponent detection only. |

## `training/model_eval/taxonomy_keypoint.yaml`

Scores the keypoint (YOLO-pose) model, which only detects our robots. Excludes `house_bot` and `object` and keeps `opponent`.

## `training/model_eval/taxonomy_keypoint_ours.yaml`

Compares keypoint models with different class vocabularies. It also excludes `opponent`.

`taxonomy_keypoint.yaml` keeps `opponent`, and every opponent box in `nhrl_keypoints_eval_test` carries keypoints. That is fine for grading one 3-class model against another. It is unfair once a 2-class model is in the grid: that model cannot emit an opponent detection, so its matched-box count and recall are capped by its class list instead of its keypoint quality.

Excluding `opponent` scores every arm on our own robots, which is the deployment task: the keypoint model estimates our robot's heading and the opponent is the other branch's job. In this eval set that leaves 670 `mrs_buff_mk3` boxes and no `mr_stabs_mk2` boxes.

## `training/model_eval/taxonomy_merged.yaml`

Scoring view for 2-class models (`robot`, `house_bot`). The eval GT declares `mr_stabs_mk2` and `mrs_buff_mk3` as separate classes, but a 2-class model has no such heads and predicts `opponent` for both. `archetypes` maps both to `opponent` so the GT vocabulary matches what the model can emit. Without it, every correct detection of our own robot scores as a wrong class. `object` stays excluded because it is GT-only debris no model here predicts.

## `training/model_eval/taxonomy_new_class.yaml`

Scoring view for the category-addition experiment. It excludes everything except `mrs_buff_mk3`, the category added in run 2, so acquisition is measured on its own and not diluted by the far more numerous `opponent` and `house_bot` boxes. It applies to the 5-class runs only. The 4-class run has no such head and would score a trivial zero.

## `training/model_eval/taxonomy_opponent.yaml`

Scores opponent detection alone. The domain-mix plan (`docs/experiments/perception/synthetic_domain_mix_plan_2026-09-12.md`) adopts a mix on agnostic opponent recall. `taxonomy.yaml` scores our robots and the house bot alongside opponents, so its recall mixes three tasks. `taxonomy_merged.yaml` folds our robots into `opponent`, which counts a Mrs Buff box as an opponent hit. Excluding everything but `opponent` from both GT and predictions leaves the agnostic level as opponent recall: every GT opponent box, matched by any remaining prediction.
