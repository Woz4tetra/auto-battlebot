# Synthetic + real bbox opponent detector, 2026-07-22

Analysis date: 2026-07-22 (val split); **independent eval added 2026-07-23**. Question: does adding the
available synthetic opponent diversity to the real bbox training set improve opponent detection without
regressing our robots? **Definitive answer (independent eval of unseen fights): no.** Adding the
synthetic gives **no statistically significant change** on the held-out eval — all 12 paired-bootstrap
tests are `ns` — and the point estimates trend **slightly negative** on opponents. The apparent gain seen
on the same-corpus val split **did not generalize**. Recommendation: **do not adopt `mix_all`; keep the
real-only detector.**

Plan: `synthetic_plus_bbox_plan.md`.

## Summary

- **Independent eval is a wash, trending negative** (score.py, 372 reviewed frames of real mrs_buff
  fights, conf 0.5, paired bootstrap 1000×, Δ = mix_all − real_bbox):
  - **Agnostic recall** ("can it find the robot") 0.742 → 0.729, **Δ −0.013, CI [−0.036, +0.010], ns.**
  - Agnostic precision −0.006 (ns), F1 −0.010 (ns). Archetype recall −0.019 (ns).
  - **Opponent AP50-95** 0.305 → 0.272 (**−0.033**, point estimate — per-class AP is not bootstrapped).
  - Every significance-tested metric's CI includes 0 → **no reliable difference either way.**
- **Only our own robot improved:** `mrs_buff_mk3` AP50-95 0.452 → 0.478 (+0.026, directional). `house_bot`
  fell 0.704 → 0.646 (−0.058, directional).
- **The val split told the opposite story** — and was misleading. On the same-corpus real val (5,454
  frames), `mix_all` showed opponent `robot` recall **+0.0195**. That gain **did not transfer** to unseen
  fights; on the independent eval the opponent point estimates flip negative. Same-corpus held-out data
  over-credited the synthetic.

Verdict: at a **0.36× synthetic dose** the available synthetic diversity **does not improve** the
opponent detector on the eval that matters (unseen fights), and slightly hurts opponent point estimates.
This is the plan's **"Neutral / keep real-only"** outcome. A negative result — and a methodology lesson:
the same-corpus val A/B was not a safe proxy for generalization.

## Setup

- **Head/regime held constant.** Both models are `yolo26n` detect, 500 epochs, batch 128, imgsz 640, DDP
  across 3 A6000s, identical Ultralytics 8.4.9 settings (`close_mosaic=10`, linear LR, `lrf=0.01`). Only
  the training mix differs.
- **`real_bbox` (baseline):** existing `nhrl_robots_bbox` 500-epoch model, **reused not retrained**
  (`data/models/yolo26n_nhrl_robots_bbox_2026-07-16.pt`). 5 classes `[object, robot, house_bot,
  mr_stabs_mk2, mrs_buff_mk3]`, train 49,086.
- **`mix_all`:** real bbox train + **all** converted synthetic, train 67,533 (49,086 real + 18,447
  synth), val kept real-only. `data/models/yolo26n_mix_all_2026-07-22.pt`, 500 epochs, 19.9 h.
- **Synthetic source (no rendering).** `training/data/all_robot_keypoints` (BlenderProc `synthgen`, ~97%
  synthetic) converted to bbox by `training/yolo/pose_to_bbox.py` (keypoints dropped, classes remapped:
  `nhrl_robot` → generic `robot`, our robots → 3/4), output `training/data/synth_bbox_from_keypoints`.
- **Dose.** Real train holds 99,655 opponent boxes; synthetic adds 36,108 (`robot`) = a **0.36× dose**.
  1×/3× is not reachable from the local synthetic without duplicating frames, so this is the single
  achievable `mix_all` arm.
- **Two evals.** *Independent* (`nhrl_keypoints_eval_test`, 372 hand-reviewed frames of unseen real
  fights, integer stamp_ns names, scored by `score.py` on sm86 TensorRT engines built on megamind) — the
  honest generalization test, uploaded to megamind 2026-07-23. *Same-corpus val* (`nhrl_robots_bbox/val`,
  5,454 frames of the same recordings as training, scored by `model.val()`) — a weaker proxy.

## Result: independent eval (definitive)

`score.py`, `--labels "opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3"` (engine's `object`+`robot`
generic blobs → GT `opponent`), `taxonomy.yaml`, conf 0.5. Paired bootstrap, 1000 resamples, Δ = mix_all
− real_bbox.

Agnostic level (all robots → one blob; the load-bearing "did it find a robot" metric):

| metric | real_bbox | mix_all | Δ | 95% CI | verdict |
|---|---|---|---|---|---|
| recall | 0.742 | 0.729 | −0.013 | [−0.036, +0.010] | **ns** |
| localization_recall | 0.742 | 0.729 | −0.013 | [−0.036, +0.010] | **ns** |
| precision | 0.962 | 0.957 | −0.006 | [−0.022, +0.011] | **ns** |
| f1 | 0.838 | 0.827 | −0.010 | [−0.027, +0.007] | **ns** |

Archetype level (per-class AP; point estimates, **not** bootstrapped):

| class | real_bbox | mix_all | Δ | note |
|---|---|---|---|---|
| **opponent** AP50-95 | 0.305 | 0.272 | **−0.033** | opponent detection down |
| mrs_buff_mk3 (our robot) AP50-95 | 0.452 | 0.478 | +0.026 | our robot up |
| house_bot AP50-95 | 0.704 | 0.646 | −0.058 | down |
| mr_stabs_mk2 AP50-95 | N/A | N/A | — | no GT instances (both) |
| archetype recall | 0.657 | 0.638 | −0.019 | ns (bootstrapped) |
| wrong_class_rate | 0.114 | 0.124 | +0.010 | slightly worse |

**Reading:** every significance-tested delta is `ns` — `mix_all` and `real_bbox` are statistically
indistinguishable on unseen fights. Where the point estimates do move, they move *against* the synthetic
on opponents (agnostic recall −0.013, opponent AP −0.033) and *for* it only on our own robot (+0.026,
the synthetic mrs_buff CAD renders). Net: no opponent-detection benefit, a small non-significant cost.

## Result: same-corpus val split (the misleading proxy)

`model.val()` on `nhrl_robots_bbox/val` (5,454 real frames, same recordings as training), conf 0.001.
This is what was measured before the independent eval arrived:

| class | metric | real_bbox | mix_all | Δ |
|---|---|---|---|---|
| **robot** (9,191 inst) | recall | 0.877 | 0.896 | **+0.0195** |
| object (1,714) | mAP50-95 | 0.393 | 0.403 | +0.010 |
| mrs_buff_mk3 (1,983) | mAP50-95 | 0.789 | 0.794 | +0.005 |
| mr_stabs_mk2 (1,451) | mAP50-95 | 0.734 | 0.746 | +0.012 |
| house_bot (2,804) | mAP50-95 | 0.913 | 0.892 | −0.021 |

On same-corpus val, `mix_all` looked like a modest opponent-recall win. **It was not real.** The
independent eval (different fights) shows the opponent gain does not exist out-of-corpus — it reverses to
a −0.013 recall / −0.033 AP point estimate. The one consistent signal across both evals is `mrs_buff_mk3`
up (the synthetic includes exact-CAD renders of our own robot), and `house_bot` down.

## Interpretation — why the two evals disagree

- **Val and eval are drawn from different fights.** The `nhrl_robots_bbox` val split is other frames of
  the *same* recordings the model trained on; the opponents, arenas, and lighting are in-distribution. The
  independent eval is *different* fights. A gain that appears only on same-corpus data and vanishes (or
  reverses) on unseen data is the signature of a change that fit the training distribution rather than
  adding transferable signal.
- **The synthetic opponents are generic, not the eval's opponents.** The `nhrl_robot` synthetic is generic
  CAD/`synthgen` geometry, not the specific robots in the eval fights (`meshy_grade` already showed named
  synthetic transfer is per-opponent and fragile). Diluting 99,655 real opponent boxes with 36,108 generic
  synthetic ones apparently shifted the model slightly toward synthetic-opponent appearance, which does not
  match — and marginally costs recall on — real unseen opponents.
- **Our own robot is the exception, in both evals.** The synthetic carries exact-CAD `mrs_buff_mk3`
  renders, and `mrs_buff` AP rises on val (+0.005) and eval (+0.026). Synthetic-to-real transfer works when
  the mesh is exact (consistent with `meshy_grade`'s sphinx-at-ceiling finding); it does not for generic
  opponents.

## Reasoning / theory: why synthetic doesn't generalize "what an NHRL opponent looks like"

Synthetic data generalizes an **instance** (a specific object you can render exactly), not an **open
category** (a class whose real members you don't have and can't render faithfully). "An NHRL opponent" is
the second kind, and that is the whole problem. The chain of reasoning:

### The central point: the detector learns *context*, not opponent appearance

> **An NHRL opponent has no canonical appearance, so the model cannot and does not learn "opponent = this
> shape/texture." It learns "opponent" from _contextual and relational_ cues that generalize across the
> open set: a compact, fast-moving object on the arena floor, inside the cage, that is _not_ our robot and
> _not_ the house bot. It detects opponents largely by scene context and negative space, not by the
> opponent's own looks.**

This is the load-bearing insight, and it explains the whole result:

- **Synthetic teaches the wrong feature.** Rendering opponent *appearance* (geometry + texture) adds
  variety along an axis the model barely uses to generalize opponents. More synthetic opponent looks →
  little gain on the cue that actually does the work (context / "not-us, not-house-bot, on the floor,
  moving").
- **Worse, synthetic gets the *context* wrong.** The renderer's arena, lighting, floor reflections, lack
  of real motion blur, and different negative-space distribution mean the synthetic frames corrupt exactly
  the contextual signal the detector relies on. So the marginal effect is neutral-to-negative — which is
  what the eval shows (agnostic recall −0.013, opponent AP −0.033, all `ns`).
- **This is why abundant real data already suffices for opponents.** The context cue is cheap to learn from
  the 99,655 real opponent boxes and generalizes across robot types; it does not need more *appearance*
  variety, which is the only thing synthetic can add.

### Why the category is unrenderable in the first place

- **Open, custom, rebuilt set.** "Opponent" is hundreds of custom robots — spinners, wedges, hammers,
  drums, control bots — each with different geometry, materials, paint, and damage, and frequently rebuilt
  between events. There is no single shape to render, and the next opponent is unknown and unbuilt.
- **The synthetic is a proxy, not samples from the real distribution.** `nhrl_robot` is generic
  CAD/`synthgen` geometry, not the actual robots faced. Training on it nudges the model toward a
  synthetic-opponent mode real opponents don't occupy — a domain gap on top of the wrong-feature problem.

### Why our own robot is the exception

For `mrs_buff_mk3` the task is **instance** recognition: one fixed object with exact CAD. There,
*appearance is* a reliable discriminative cue, so learning it from exact renders helps — and it did, on
both evals (+0.005 val, +0.026 eval), matching `meshy_grade`'s sphinx-at-ceiling result (transfer works
precisely when the mesh *is* the real robot). Synthetic wins instance tasks and loses open-category ones.

### Falsifiable prediction

If the barrier is "generic appearance, wrong context," then (a) exact-mesh renders of the *specific*
opponents faced should transfer (as sphinx did), and (b) synthetic that fixed the *context* (real arena
backgrounds, real motion statistics, correct negative space) rather than the opponent geometry would help
more than more opponent shapes. Neither is worth pursuing for a deployed opponent detector, since NHRL
never gives faithful meshes of upcoming opponents — but both are testable if synthetic is revisited.

## Caveats

- **Different operating points across the two evals.** Val used `model.val()` at conf 0.001 (mAP-oriented);
  the independent eval used `score.py` at conf 0.5 (the deployed threshold). The cross-eval comparison is
  qualitative (does the gain survive out-of-corpus), not a like-for-like metric diff. Within each eval the
  A/B is apples-to-apples.
- **Per-class AP is not bootstrapped.** Opponent/our-robot/house_bot AP deltas are point estimates on
  small per-class samples (`mr_stabs_mk2` has no GT instances here). Only the agnostic/archetype aggregate
  recall/precision/F1 carry significance verdicts — and those are all `ns`.
- **One dose, one seed.** A 0.36× dose, single training run. The result rules out a *large* benefit at this
  dose; it does not prove a higher dose or a different synthetic (exact-mesh opponents, more variety)
  couldn't help. But the direction here does not motivate spending that compute on generic synthetic.
- **score.py bug fixed to produce this run.** The headline-plot stage used `df.query("... level == @level")`
  inside a list comprehension, which fails pandas frame inspection on this pandas version
  (`UndefinedVariableError: level`). Replaced with boolean-mask indexing (`training/model_eval/score.py`).
  Scoring/summary/significance are unaffected by the fix; it only unblocked plotting.

## Decision & follow-up

- **Do not adopt `mix_all`.** No significant opponent gain on unseen fights; keep the real-only
  `nhrl_robots_bbox` detector deployed.
- **If synthetic is revisited for opponents**, use *exact-mesh* opponents (the only transfer that worked,
  per our-robot and `meshy_grade` sphinx), not generic `synthgen` geometry, and grade on the independent
  eval from the start — the same-corpus val is not a safe proxy.
- **Our-robot lift is the one reusable signal**: exact-CAD synthetic reliably helps the class it depicts.
  Relevant to the keypoint/our-robot model, not this opponent detector.

## Artifacts

- Converter: `training/yolo/pose_to_bbox.py`
- Synthetic bbox set: `training/data/synth_bbox_from_keypoints` (from `all_robot_keypoints`, unmodified)
- Pooled dataset: `training/data/mix_all` (real train + all synthetic; val real-only)
- Models: `data/models/yolo26n_mix_all_2026-07-22.{pt,onnx,_x86_64_sm86.engine}`,
  `data/models/yolo26n_nhrl_robots_bbox_2026-07-16.{pt,onnx,_x86_64_sm86.engine}` (baseline, reused)
- Training run: `runs/projects/auto_battlebots_2026-07-22_02-40-51_yolo26n` (500 epochs, 19.9 h)
- Independent-eval scores: `training/data/nhrl_keypoints_eval_test/scores_synth_plus_bbox/`
  (`summary.csv`, `significance.csv`, `headline.png`, `confusion_*.png`)
- Same-corpus val comparison: `runs/valcmp_real_bbox`, `runs/valcmp_mix_all`
