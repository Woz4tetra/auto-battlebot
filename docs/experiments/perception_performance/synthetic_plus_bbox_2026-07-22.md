# Synthetic + real bbox opponent detector, 2026-07-22

> # ⛔ INVALID — do not cite. Retracted 2026-07-31.
>
> **The `real_bbox` baseline was not real.** 16,997 of its 49,086 train frames (34.6 %) are BlenderProc
> renders, carried in under the name `nhrl_robots_bbox`. Verified by counting the archived pooled
> dataset (`/media/storage/auto-battlebots-archive/mix_all`), whose filenames record each frame's
> source: `real__` totals exactly 49,086 (this report's real train), of which `real__synthetic__*` is
> 16,997. Sample `real__synthetic__004436.jpg` is a render — an Objaverse speaker labeled `object`, two
> robots, blank backdrop.
>
> Consequences, each fatal on its own:
>
> - **The contrast the experiment claims to test does not exist.** This is not real-only vs
>   real+synthetic. It is **~1/3 synthetic (49,086 frames: 32,089 real + 16,997 rendered) vs ~1/2
>   synthetic** (67,533 frames: 32,541 real + 34,992 rendered). `mix_all` is majority synthetic; so is
>   the conclusion "synthetic does not help."
> - **The dose is wrong.** The 0.36× figure is 36,108 added opponent boxes over a 99,655-box
>   denominator that already contains **31,679 synthetic opponent boxes**. A baseline already saturated
>   on generic renders is expected to show no gain from more of them — which is the result obtained.
> - **The val split is 35 % synthetic** (1,925 of 5,454 frames), not "real-only" as stated in Setup. The
>   +0.0195 opponent-recall gain reported there was measured on rendered frames.
> - **The val split is also frame-level random**, so near-duplicates of training frames sit in val. The
>   `nhrl_robots_bbox_2class` README documents this defect for every experiment before 2026-07-25.
>
> The independent-eval numbers (`nhrl_keypoints_eval_test`, 372 hand-reviewed real frames) are
> themselves sound — that eval is real and unseen. What they cannot support is any claim about
> *synthetic vs real*, because neither arm is what its name says.
>
> A separate defect in the cut-paste probe was found and fixed on 2026-07-31; that correction is kept
> below for the record, but it is subordinate to this one.
>
> **Replacement:** `synthetic_arms_plan.md` — three arms built from the audited zero-synthetic
> `nhrl_robots_bbox_2class` corpus, with provenance verified before training rather than after.

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

## Interpretability probes: context or appearance? (empirical confirmation)

The reasoning above is a claim about *what the model keys on*. Four probes test it directly on the
independent-eval frames, comparing both models. Script:
`training/model_eval/interpret_context_vs_appearance.py`; figures + `metrics.json` under
`scores_synth_plus_bbox/interpretability/`, with the corrected 2026-07-31 re-run alongside in
`interpretability_donorfix_2026-07-31/`. The probes confirm the claim — the detector does key on context
— but the cut-paste probe, once fixed, narrows *which* context counts (see the correction note there).

### How to read the saliency probes

Two of the four probes (RISE, Grad-CAM) produce a heat map rather than a score, and both are reduced to
the same three numbers, so it is worth stating what those numbers mean before reading them.

**The shared metric: concentration.** `mass_fractions()` splits the 640×640 heat map into three regions
— *inside* (the GT box), *ring* (the box grown by half its width and height on each side, minus the box
itself), and *background* (everything else) — then reports two things per region: `frac`, its share of
the total heat, and `concentration`, that share divided by the region's share of the pixels.
Concentration is the one to read:

- **1.0** = the region holds exactly the heat its size entitles it to — no preference at all.
- **2.0** = twice its fair share; **0.5** = half.

The normalization is not a nicety. An opponent box is roughly 1 % of the frame, so the raw fractions are
always lopsided — `background_frac` is 0.97 in every run simply because the background is 97 % of the
pixels. Raw mass would say "the evidence is in the background" for any model whatsoever. Only the
area-normalized number compares regions of wildly different size.

For calibration: a detector that keyed purely on the robot's own pixels would show an inside
concentration in the tens with ring ≈ background ≈ 1.0. Everything reported below is far short of that.

**RISE** (Randomized Input Sampling for Explanation) is the black-box probe — it never looks inside the
network, it just hides parts of the image and watches the score:

1. Draw 400 random masks per frame on an 8×8 grid, each cell kept with probability 0.5, bilinearly
   upsampled to 640 and randomly shifted — so the occluders are smooth blobs, not hard squares.
2. Multiply the frame by each mask (hidden pixels go black) and re-score the opponent at the *same*
   target box.
3. Accumulate `Σ (mask × score) / (400 × 0.5)`. A pixel scores high if the detection survived whenever
   that pixel was visible — i.e. the model needed it.

Only boxes the model actually detects (score ≥ 0.15) are explained, hence n = 77 / 74 of 100 frames.
So `inside 1.28` means: the robot's own pixels carry 28 % more evidence than an equal-area patch of
arbitrary frame — and the ring around it carries 23 % more, nearly the same. That near-tie is the
finding.

**Grad-CAM** is the white-box probe — one backward pass, no resampling:

1. Forward the unmodified frame; the target is the max opponent probability among head anchors inside
   the GT box (the same target every probe uses).
2. Backpropagate to neck layer 16 (P3, stride 8, an 80×80 feature grid), weight each channel of that
   layer by its mean gradient, sum the weighted channels, and ReLU — keeping only evidence that pushed
   the score *up*.
3. Normalize, upsample to 640, and measure with the same three regions.

n = 34 / 31 boxes (detected boxes among the first `--gradcam-k 40` frames).

**Why the two disagree in degree, and which to trust.** RISE measures a causal effect at input
resolution, but its mask cells are 640/8 = 80 px — comparable to the robot boxes themselves (~90×60 px)
— so it cannot cleanly separate "inside the box" from "just outside it"; some of the ring's 1.23 is
blur from the box. Blacking pixels out is also off-distribution. Grad-CAM has 8 px feature resolution
and no occlusion artifacts, but it only linearizes the model around a single point and discards negative
evidence, which systematically sharpens the map onto the object. Their disagreement (1.28 vs 2.01
inside) is the expected signature of those two biases, not a contradiction. Read the direction — modest
appearance focus, not dominant — rather than the exact multiple. Four example overlays per model are in
`rise_*.png` and `gradcam_*.png`.

### Cut-paste context swap (decisive)

> **Corrected 2026-07-31.** The `crop_on_arena` row below originally read 0.474 / 0.467 — identical to
> `original` — and that identity was reported as the headline evidence. It was an aliasing bug, not a
> measurement: the probe indexed its background list with the frame's own index, pasting each crop back
> at its own coordinates on its own frame, so the "context swap" scored the unmodified image. Fixed by
> `donor_index()` (`interpret_context_vs_appearance.py`), re-run on the same 100 frames and models. RISE,
> Grad-CAM and feature-space reproduce their original values exactly; only this row moved. Numbers below
> are the corrected run (`scores_synth_plus_bbox/interpretability_donorfix_2026-07-31/`). The conclusion
> changes: a real arena background does **not** restore the score.

Opponent-score at the box (max opponent-class prob among head anchors inside it), same target box, only
the pixels around/inside it changed:

| condition | real_bbox | mix_all | isolates |
|---|---|---|---|
| original frame | 0.474 | 0.467 | full scene |
| **same crop on a different frame's arena** | **0.281** | **0.354** | appearance + mismatched context |
| **same crop on neutral gray** | **0.238** | **0.324** | appearance only |
| robot removed (blurred), arena kept | 0.033 | 0.018 | context only |

Read the two ends first. Stripping the surroundings to neutral gray — same robot pixels — **halves** the
score (0.474 → 0.238), while removing the robot and keeping the arena gives **≈0** (0.033). Neither the
robot's appearance alone nor its surroundings alone carries a detection: the model needs both, and the
surroundings are worth roughly half the score for a fixed set of robot pixels. That much survives the
correction intact, and it is the basis for the context claim.

The middle row is what changed, and it sharpens the claim rather than supporting it as originally
written. Pasting the identical crop onto a *different* frame's real arena recovers almost nothing:
0.281 against 0.238 for bare gray, which is **18 %** of the gap between gray and the original
(`(0.281 − 0.238) / (0.474 − 0.238)`). Generic arena pixels — the same floor, the same NHRL logo, the
same lighting, from a different moment of a different fight — are *not* a substitute for the frame the
robot was actually in. So the cue is not "arena-ness" in the abstract; the model needs the robot
embedded in a scene that is coherent with it. **Object-in-arena-context is better stated as
object-in-*this*-scene.**

`mix_all` still leans more on appearance than the baseline: gray retains **69 %** of its original score
(0.324 / 0.467) versus **50 %** for `real_bbox` (0.238 / 0.474). The synthetic nudged it toward the robot
pixels, and since that appearance is off-distribution, the nudge was net-negative on real opponents —
consistent with the eval regression. The corrected arena row does not disturb that reading.

**What the swap does not isolate.** The crop is pasted at its original coordinates into the donor frame,
so a robot from the near floor can land where that frame shows a wall, a distant floor patch, or the
arena edge — wrong scale and wrong perspective for its new surroundings. Donor selection skips frames
whose own labeled robots sit under the paste rect, but does not match camera pose. The 0.281 therefore
mixes *lost* context with *implausible placement*, and should be read as a lower bound on what a
pose-matched swap would give. Testing that separation needs donors chosen by camera pose, which has not
been run. Example inputs for all four conditions: `assets/cutpaste_mosaic.png`
(`training/model_eval/make_cutpaste_mosaic.py`).

### RISE occlusion-saliency

Area-normalized saliency concentration (1.0 = uniform / no preference for that region):

| region | real_bbox | mix_all |
|---|---|---|
| inside box | 1.28 | 1.26 |
| context ring (2× box) | 1.23 | 1.21 |
| background | 0.99 | 1.00 |

The box is only weakly favored (1.28×), and the **immediate context ring is almost equally salient
(1.23×)** — a pure appearance detector would show the box concentration far above the ring. Evidence for
the opponent is spread across the robot *and* its surroundings. (n = 77 / 74 detected boxes explained.)

### Grad-CAM (corroboration)

Gradient saliency at the P3 neck layer (layer 16), same three regions and the same concentration metric:

| region | real_bbox | mix_all |
|---|---|---|
| inside box | **2.01** | **1.52** |
| context ring (2× box) | 0.63 | 0.71 |
| background | 1.00 | 1.00 |

Gradient attribution localizes on the robot somewhat more than RISE (~2× its area share rather than
1.28×), but still only modestly — and `mix_all`'s evidence is *less* focused on the robot than the
baseline's. The ring falls below 1.0 here where RISE put it at 1.23; per the resolution argument above,
Grad-CAM's sharper features and its ReLU pull mass onto the object, so the truth about the ring sits
between the two probes. CAM methods are coarse, so this is corroboration of "modest, not dominant,
appearance focus," not a standalone proof. (n = 34 / 31.)

### Feature-space domain gap

Backbone embeddings (deepest neck feature, GAP) for real-opponent vs synthetic-opponent vs background
crops (191 / 200 / 200):

| metric | real_bbox | mix_all |
|---|---|---|
| real-vs-synth opponent linear-probe acc (5-fold) | **0.969** | **0.954** |
| real↔synth opponent centroid cos-dist | 0.012 | 0.008 |
| real↔background centroid cos-dist | 0.052 | 0.074 |

A linear probe separates real from synthetic opponents at **~96 %** — they occupy near-disjoint regions
of feature space despite both being "opponents." Training on the synthetic (`mix_all`) barely closes the
gap (0.969 → 0.954): even the model that trained on it still tells synthetic from real opponents almost
perfectly. This is the mechanistic confirmation of "the synthetic is a persistent off-distribution mode
that doesn't merge into the real-opponent distribution" — so it can't add real-opponent coverage.

### What the probes establish

- **Context is a first-class cue, not a tie-breaker** — it carries ~half the opponent score (cut-paste
  gray) and is nearly as salient as the robot itself (RISE). The detector recognizes "opponent" as an
  object-in-scene gestalt, as the theory predicted.
- **But the context has to be the robot's own scene** — swapping in a different frame's arena recovers
  only 18 % of what gray removed. The cue is scene coherence, not the presence of arena-like pixels.
  This is stronger than the original theory, which expected any real arena background to do.
- **The synthetic stays off-distribution** (96 % separable) — confirming why generic synthetic opponents
  add no transferable signal and slightly hurt.
- **`mix_all` shifted marginally toward appearance-reliance** (cut-paste gray-bg, Grad-CAM), and that
  shift is *toward* the off-distribution synthetic mode — a coherent mechanism for the eval regression.

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
- **Interpretability-probe caveats.** The neutral-gray paste is out-of-distribution at its boundary (no
  shadow/contact), so `crop_on_gray` slightly *under*-states appearance-only detection. The arena-swap
  condition shares that seam and adds a placement confound (crop pasted at its own coordinates into a
  frame with different geometry), so it is not the clean control it was originally claimed to be — see
  the section for what it does and does not isolate. Grad-CAM/CAM
  saliency is coarse and known to be unreliable alone — treated here as corroboration of RISE + cut-paste,
  not standalone proof. Probe targets use the raw one2one head scores (not post-NMS), and the score is a
  max-anchor-in-box readout, not the deployed detection pipeline.
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
- Interpretability: `training/model_eval/interpret_context_vs_appearance.py` →
  `scores_synth_plus_bbox/interpretability/` (`metrics.json`, `rise_*.png`, `gradcam_*.png`, `embed_*.png`),
  original 2026-07-22 run with the aliased `crop_on_arena` — kept for the record
- Corrected re-run (2026-07-31, `donor_index()` fix): `scores_synth_plus_bbox/interpretability_donorfix_2026-07-31/`
  (`--frames 100 --crops 200 --gradcam-k 40`, both models). Needs `training/data/synth_bbox_from_keypoints`,
  regenerable from `all_robot_keypoints` via `pose_to_bbox.py` with `--class-map 2:1,0:3,1:4`
- Cut-paste condition figure: `assets/cutpaste_mosaic.png` ← `training/model_eval/make_cutpaste_mosaic.py`
- Same-corpus val comparison: `runs/valcmp_real_bbox`, `runs/valcmp_mix_all`
