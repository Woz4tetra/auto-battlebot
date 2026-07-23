# Synthetic + real bbox opponent detector — experiment plan

Status: **executed & graded 2026-07-23.** Follows the pipeline in `experiment_runbook.md`. This document
is the design; the results are written up in **`synthetic_plus_bbox_2026-07-22.md`**.

> **Outcome — DO NOT ADOPT (definitive, independent eval).** On the independent eval of *unseen* fights
> (`nhrl_keypoints_eval_test`, 372 frames, paired bootstrap), the `mix_all` arm (0.36× synthetic dose) is
> **statistically indistinguishable from the real-only baseline — all 12 tests `ns`** — and trends
> **slightly negative** on opponents: agnostic recall 0.742 → 0.729 (Δ −0.013, CI [−0.036, +0.010]),
> opponent AP50-95 0.305 → 0.272 (−0.033, point est). Only our own robot improved (`mrs_buff_mk3` AP
> +0.026 — exact-CAD synthetic transfers). **The same-corpus val split was misleading**: it showed
> opponent recall +0.0195, which **did not generalize** — the gain reversed on unseen fights. Verdict:
> the plan's **"Neutral / keep real-only"** branch. Lesson: same-corpus val is not a safe proxy for
> generalization; grade on the independent eval. See §Next steps.

## Question

The opponent/blob detector's weak class is `opponent`. Real opponent data is scarce and low-diversity
(a handful of robots, recorded in a handful of fights), and that ceiling shows in the numbers: the
real-only bbox detector tops out at **opponent AP50-95 ≈ 0.30** and **agnostic recall ≈ 0.74**
(`seg_vs_bbox_2026-07-18.md`). Synthetic renders are unlimited and diverse but have a domain gap:
synthetic-*only* opponents floor at **AP 0.084** (`meshy_grade_2026-07-16.md`).

**Does pooling synthetic opponent renders into the real bbox training set improve opponent detection
(recall + AP) over the real-only bbox detector, without regressing our-robot / house_bot boxes — and
does more synthetic help or saturate/hurt?**

This is the natural follow-on to seg-vs-bbox (which fixed the *head*: bbox-only wins) — here the head
is held constant at `yolo26n` detect and the only variable is the **training mix**.

**No new rendering.** All the synthetic data this experiment needs already exists locally as the
synthetic *keypoint* dataset `training/data/all_robot_keypoints` (~97% synthetic, 3 classes
`[mr_stabs_mk2, mrs_buff_mk3, nhrl_robot]`; `nhrl_robot` is the generic synthetic opponent, ~36k boxes
in the train split alone). The only build step is a **label-only conversion** of that dataset to
bbox — no BlenderProc run.

## Hypothesis

Synthetic adds pose / lighting / background / opponent-shape diversity that the small real opponent set
lacks. Expected: a modest 1:1 synthetic addition **lifts agnostic recall and holds or lifts opponent
AP** (better generalization to opponents outside the real set); a heavy synthetic addition **saturates
and may cost precision** as the synthetic domain gap injects false boxes. The interesting output is the
shape of that curve, not a single number.

> **Result vs hypothesis — not confirmed.** The hypothesis (synthetic diversity lifts opponent recall)
> held only on the *same-corpus* val (+0.0195) and **failed on the independent eval**: opponent recall and
> AP trended negative (both `ns` / point-est), so at 0.36× dose generic synthetic did **not** add
> transferable opponent signal. The predicted domain-gap cost showed up as the direction of the eval
> deltas. The *saturation* half is untested (no higher-dose arm reachable), and the result does not
> motivate building one from generic synthetic.

## Design — one variable, paired scoring

Everything is held identical to the seg-vs-bbox winner except the training mix:

- Head: `yolo26n` detect (no seg, no pose keypoints).
- Classes: the 5-class `nhrl_robots_bbox` vocabulary `[object, robot, house_bot, mr_stabs_mk2,
  mrs_buff_mk3]`, in that order. The converted synthetic classes map into it (step 1): synthetic
  `nhrl_robot` → generic **`robot`** (class 1) so opponents land in the `object,robot → opponent`
  bucket score.py collapses; synthetic `mr_stabs_mk2` → 3, `mrs_buff_mk3` → 4 keep their vocab indices.
- Real substrate constant: `training/data/nhrl_robots_bbox` (train 49086 / val 5454) is the fixed
  base in every arm. Only the synthetic *addition* changes.

Arms — **implementation-adjusted** (see the box below on why 1×/3× is not achievable from the local
synthetic data):

| arm | training data | isolates |
|---|---|---|
| `real_bbox` (baseline) | real bbox only — **reuse** the existing 500-epoch `yolo26n_nhrl_robots_bbox_2026-07-16` best.pt (no retrain) | control |
| `mix_all` | real bbox train + **all** converted synthetic (opponent dose ≈ 0.36× real), val kept real-only | does adding the available synthetic diversity help |

> **Why not the 1×/3× arms in the original plan.** The real base already holds **99,655** opponent boxes
> in train (`object` 15,568 + `robot` 84,087) — high count, low diversity (a few distinct opponents). The
> converted synthetic set has only **36,108** opponent (`robot`) boxes in train. So even adding *all* of
> it is a **0.36× dose**, not 1×; 3× is impossible without duplicating synthetic frames (pure reweight,
> no new diversity). The honest, achievable test is therefore a single `mix_all` arm — does the available
> synthetic diversity move the needle at all. This kept the experiment to **one** ~500-epoch training run
> (the baseline is reused, not retrained).
>
> **It did not move the needle on the independent eval** (Outcome, above). So the conditional higher-dose
> arm is **not** pursued for *generic* synthetic — more of a non-transferring signal won't help. If
> revisited, use exact-mesh opponents (the only transfer that worked) and grade on the independent eval.

**Pooling refinement (val stays real-only).** Rather than pool-then-resplit (which would leak easy
synthetic frames into val and make the val curve non-comparable to the reused baseline), `mix_all`
**adds synthetic only to the train split** and keeps `nhrl_robots_bbox`'s **real val unchanged**. Val is
then identical across baseline and `mix_all`, so the val mAP curve is a clean apples-to-apples plateau
signal. The verdict still comes from the external real eval, not val.

Both arms share the identical 5-class order (a prerequisite for a paired `score.py` run on the dev-box
follow-up). Latency is identical across arms (same head, same class count, same output tensor
`[1, 9, 8400]`), so no latency measurement is needed.

> **What was actually scored (2026-07-22).** `score.py` + the independent eval were unavailable on
> megamind (§5 box), so the on-megamind comparison used Ultralytics `model.val()` on the real held-out
> val split — a per-class point-estimate A/B, not the paired bootstrap. The bootstrap grade moves to the
> dev-box follow-up (§Next steps).

## 1. Convert the existing synthetic keypoints → a new bbox dataset

No rendering. Source is the already-local `training/data/all_robot_keypoints` (synthetic pose rows
`cls cx cy w h kx ky kv kx ky kv`, classes `0 mr_stabs_mk2 / 1 mrs_buff_mk3 / 2 nhrl_robot`). Produce a
**new** dataset `training/data/synth_bbox_from_keypoints` — **do not overwrite or modify
`all_robot_keypoints`** (it is the keypoint model's training set).

Two label-only transforms, geometry otherwise untouched, split structure preserved:

1. **Strip the keypoint columns** — keep only `cls cx cy w h` from each pose row. `seg_to_bbox.py` is
   *not* usable here: it treats a >4-coord row as a polygon (x,y pairs) and would fold the keypoint
   values into a bogus box. A pose row is exactly `cls` + 4 box coords + 3·`num_keypoints`; keep the
   first five fields. Hardlink the images (instant, no copy) into the new dir alongside the stripped
   labels, preserving `train/ val/ test/`. (Add a small `pose_to_bbox.py`, mirroring `seg_to_bbox.py`'s
   image-hardlink + split-preserving structure, or a guarded `awk '{print $1,$2,$3,$4,$5}'` per label
   file — but verify every row has ≥5 fields first so a malformed row is not silently truncated.)

2. **Remap class ids** into the `nhrl_robots_bbox` vocabulary with `remap_labels.py` (explicit
   simultaneous source→target map, applied to originals so there is no chaining collision):

   - `2 → 1`  (synthetic `nhrl_robot` → generic `robot`, the opponent bucket)
   - `0 → 3`  (`mr_stabs_mk2`)
   - `1 → 4`  (`mrs_buff_mk3`)

   Verify the map — a wrong id silently poisons the mix. There is no `object`(0) or `house_bot`(2) in
   the synthetic source, so those vocab slots simply get no synthetic boxes (fine; the real base
   supplies them).

Author the new `data.yml` (detect form, 5 classes, no `kpt_shape`/`flip_idx`) and validate:

```bash
venv/bin/python training/yolo/validate_yolo_integrity.py training/data/synth_bbox_from_keypoints --strict
```

Expect: only the 5 vocab classes present (with `object`/`house_bot` at zero), every row exactly 4-coord
boxes (no stray keypoint columns), no malformed rows. Sanity-check the class histogram — the bulk should
be class 1 (remapped `nhrl_robot`, ~36k in train).

## 2. Pool `mix_all` (Recipe C — synthetic into train only)

**Built (2026-07-22): `training/data/mix_all`.** Hardlink (`os.link`, instant) into train:
- all real `nhrl_robots_bbox/train` pairs (prefix `real__`, 49,086),
- all converted `synth_bbox_from_keypoints/train` pairs (prefix `synth__`, 18,447).

Val = real `nhrl_robots_bbox/val` only (prefix `real__`, 5,454) — **synthetic never enters val**, so the
val A/B stays comparable to the reused baseline. Do **not** re-split; do **not** use
`merge_yolo_datasets.py` (needs per-input `validation_state.json`, flat-only, drops metadata). Source
filenames don't collide, but the prefixes make provenance obvious. `.npy` caches are not linked (training
rebuilds them). This was scripted in a one-off pooling helper.

Result: train 67,533 (135,763 opponent boxes = 99,655 real + 36,108 synth), val 5,454.
`validate_yolo_integrity --strict`: **0 errors** (1 background-negative frame, expected).

`data.yml` is the **detect** form (5 classes, no `kpt_shape` / `flip_idx`), names/colors copied from
`nhrl_robots_bbox`.

> **For the higher-dose follow-up arm** (§Next steps): build `mix_2x`/`mix_3x` the same way but oversample
> the synthetic frames (link each synthetic pair 2–3× under distinct prefixes) or add newly-rendered
> synthetic, sizing by counted opponent-box ratio. Keep val real-only, same as here.

## 3. Train

> **This run executes on megamind**, so everything below is local — no rsync, no `ssh megamind`, no
> gvfs. Datasets, `best.pt`, ONNX, engines, and scoring all live and run on megamind directly. The
> cross-machine notes (upload the split from a dev box, download `best.pt` over gvfs) apply only to a
> *future* run driven from the dev box; skip them this time.

Datasets are already local (the conversion in §1 and pooling in §2 wrote them under
`training/data/` on megamind). Build in a local tmux session:

```bash
tmux new-session -d -s train -c /home/ben/auto-battlebot \
  'venv/bin/python training/yolo/train.py training/data/<arm>/data.yml yolo26n 2>&1 | tee /tmp/train_<arm>.log'
```

Match the baseline's regime for a fair comparison: `yolo26n` detect head, ~500 epochs, batch ~128, DDP
across the 3 GPUs (as `yolo26n_nhrl_robots_bbox_2026-07-16`). Monitor `runs/projects/<run>/results.csv`
`metrics/mAP50-95(B)`; cancel at plateau. **Because scoring also runs on megamind this time, do not run
the scoring pass (heavy inference IO) while a training arm is still going** — it evicts the training's
page cache and spikes epoch time ~30×. Score after the training arms finish, or on an idle GPU.

## 4. Export engines — **skipped on megamind** (only needed for the dev-box follow-up)

The on-megamind comparison used `model.val()`, which runs directly on the `.pt`, so **no engine was
built here**. Engines are needed only for the dev-box `score.py` follow-up, where they must be
`_x86_64_sm89` (built on the dev box), not sm86. The recipe below is retained for that follow-up / for a
future megamind-scored run; ignore the sm86 tag when building on the dev box.

`best.pt` is already under `runs/projects/<run>/weights/` locally — no gvfs copy:

```bash
cp runs/projects/<run>/weights/best.pt data/models/yolo26n_<arm>_<date>.pt
venv/bin/python training/yolo/convert_to_onnx.py data/models/yolo26n_<arm>_<date>.pt
venv/bin/python training/yolo/convert_to_tensorrt.py data/models/yolo26n_<arm>_<date>.onnx --workspace 1
# -> data/models/yolo26n_<arm>_<date>_x86_64_sm86.engine   (arch tag auto-appended = megamind's sm86)
```

Detect ONNX disables the end2end NMS head → raw `[1, 9, 8400]` the C++ model and `score.py` expect.

**Rebuild the baseline engine for sm86 too.** The existing `yolo26n_nhrl_robots_bbox_2026-07-16` engine
is tagged `_x86_64_sm89` (dev box) and will not load on megamind's sm86 GPU. Convert its ONNX (or `.pt`)
to an sm86 engine on megamind so the whole paired run uses engines that actually run here:

```bash
venv/bin/python training/yolo/convert_to_tensorrt.py \
  data/models/yolo26n_nhrl_robots_bbox_2026-07-16.onnx --workspace 1
# -> data/models/yolo26n_nhrl_robots_bbox_2026-07-16_x86_64_sm86.engine
```

> Engines are GPU-arch-specific. This run scores on megamind, so build **sm86**. A dev-box run would
> build **sm89** instead (per the runbook), and the deployment engine for the Jetson is **aarch64_sm87**,
> built on the Jetson — never used for scoring.

## 5. Score

**On megamind (done):** Ultralytics `model.val()` on the real held-out val split — see the block below
and the writeup. **Independent-eval paired `score.py` (not run here — dev-box follow-up):** both models
share the identical 5-class order, so a single dev-box run gives paired-bootstrap significance vs the
real-only baseline. Build `_x86_64_sm89` engines on the dev box (per the runbook), then:

```bash
venv/bin/python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate real_bbox=data/models/yolo26n_nhrl_robots_bbox_2026-07-16_x86_64_sm86.engine \
  --candidate mix_all=data/models/yolo26n_mix_all_2026-07-22_x86_64_sm86.engine \
  --labels "opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3" \
  --taxonomy training/model_eval/taxonomy.yaml --conf 0.5 --baseline real_bbox \
  --output training/data/nhrl_keypoints_eval_test/scores_synth_plus_bbox
```

> **`--labels` is the canonical mapping, not identity.** The eval GT's opponent class is named
> `opponent`; the engine's two generic blob classes (`object`, `robot`) both map to it, so the label list
> is `opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3` (same mapping the seg-vs-bbox writeup used).
> Its length (5) equals the engine class count. Engines are `_x86_64_sm86` because scoring runs on
> megamind (the eval set was uploaded there 2026-07-23); build sm86, not sm89.

> **Two constraints forced a different scorer on megamind.**
> 1. The independent, hand-labeled `nhrl_keypoints_eval_test` (where the reference points 0.084 / 0.21 /
>    0.675 were measured) is **not on megamind** — a hand-reviewed artifact that lives on the dev box and
>    cannot be regenerated here.
> 2. `score.py` is **purpose-built for the MCAP-derived eval sets**: it reads a `data.yaml` and does
>    `int(label_path.stem)` to align GT to a playback by SVO stamp_ns. It therefore **cannot** score a
>    training dataset like `nhrl_robots_bbox/val`, whose frames are named `..._frame_004090` (not integer
>    stamps) and whose config file is `data.yml`. It would raise `No data.yaml found` / `ValueError`.
>
> So the achievable on-megamind real comparison is **Ultralytics `model.val()` on the real held-out val
> split** of `nhrl_robots_bbox` (5,454 real frames). Both models saw the identical real train, neither saw
> val, and `mix_all` kept val real-only → a fair A/B on real data, giving per-class box
> precision/recall/mAP50/mAP50-95. It is same-corpus-as-training (different frames of the same fights), so
> it answers *"does synthetic help on real held-out frames"* — real and valid, weaker than generalization
> to unseen fights. `model.val()` runs directly on the `.pt` (no engine needed).
>
> The definitive independent-eval grade — agnostic recall, opponent AP, paired bootstrap, against
> `nhrl_keypoints_eval_test` with the plan's reference points — is a documented **follow-up on the dev
> box**: copy the two `.pt`, build `_x86_64_sm89` engines there, run the paired `score.py` command shown
> above (unchanged) against the dev-box eval.

On-megamind real comparison (both `.pt`, identical real val split):

```bash
# per-class box AP/precision/recall on training/data/nhrl_robots_bbox val (real, held out from both)
venv/bin/python - <<'PY'
from ultralytics import YOLO
for name, pt in [
    ("real_bbox", "data/models/yolo26n_nhrl_robots_bbox_2026-07-16.pt"),
    ("mix_all",   "data/models/yolo26n_mix_all_2026-07-22.pt"),
]:
    print("====", name)
    YOLO(pt).val(data="training/data/nhrl_robots_bbox/data.yml", split="val",
                 imgsz=640, conf=0.001, iou=0.6, plots=False, verbose=True)
PY
```

Read per-class AP for `object`+`robot` (the opponent bucket) and `mrs_buff_mk3` (our robot); compare
`mix_all` vs `real_bbox` on the same rows. The final `results.csv` row of each training run also carries
the aggregate val metrics on this same real val, as a cross-check.

**score.py mechanics that gate this experiment:**

- `--labels` maps engine class index → GT label name, in class order, and **must equal the engine's
  class count (5)**. score.py infers `num_classes` from the label count; a wrong count misparses the
  output tensor and returns ~0 recall (looks like a broken engine, isn't). Check the printed
  `output [...] num_keypoints=0 num_classes=5` line — `num_keypoints=0` confirms these are detect
  (non-pose) engines.
- The canonical mapping `object,robot → opponent` is why synthetic opponents were remapped to `robot`
  in step 1: both generic blob classes collapse to `opponent`, so synthetic opponent training data
  scores against the GT's single generic `opponent`. Keep this mapping **identical across the whole
  run** — engines with different class counts cannot share a run, and cross-run absolute AP is not
  comparable (different mappings; see the seg-vs-bbox caveat on 0.244 vs 0.210).
- Levels: **`agnostic`** (all robots → one blob) is the load-bearing "can it find robots" metric and is
  frame-decomposable → gets recall / precision / F1 / localization_recall **significance verdicts** vs
  `real_bbox`. **`archetype`** gives per-class `ap50_95/opponent`, `ap50_95/house_bot`,
  `ap50_95/mrs_buff_mk3` — directional, **not** bootstrapped (small per-class samples), read as trend.
- No keypoint metrics apply (detect head, no `kpt_shape`). Use `taxonomy.yaml`, not
  `taxonomy_keypoint.yaml`.
- Eval set: `nhrl_keypoints_eval_test`, ~372 reviewed frames of real mrs_buff fights (only frames marked
  reviewed in `.edit_state.json` are scored); GT labels every opponent as a single generic `opponent`.

## Success criteria — how the result scored (independent eval, definitive)

Scored on `nhrl_keypoints_eval_test` with the paired `score.py` bootstrap (conf 0.5, 1000 resamples).

Primary — *agnostic-level significance vs `real_bbox`*:
- **Win** (recall up, CI excludes 0, precision/F1 held): **not met.** Agnostic recall Δ −0.013,
  CI [−0.036, +0.010] — includes 0.
- **Neutral** (recall CI includes 0): **→ this is the outcome.** All four agnostic tests (recall,
  precision, f1, localization_recall) are `ns`. Synthetic neither reliably helps nor hurts — **keep
  real-only for simplicity.**
- **Loss** (significant precision/F1 or recall drop): not met either — the negative trend is not
  significant.

Secondary — *opponent AP & regression guard* (point estimates, not bootstrapped):
- **Opponent AP50-95** 0.305 → 0.272 (**−0.033**): down, the wrong direction — the experiment's target
  metric did not improve.
- **Regression guard:** our robots not sacrificed (`mrs_buff_mk3` AP +0.026); `house_bot` −0.058 down.

Net: **Neutral → keep real-only.** The same-corpus val "Win" did not survive the independent eval.

## Next steps

The independent eval resolved the core question (no generic-synthetic benefit), so the follow-ups narrow:

1. **Independent-eval grade — DONE (2026-07-23).** Ran on megamind after the eval set was uploaded there:
   sm86 engines + paired `score.py` (§5). Result: all `ns`, keep real-only (Outcome/Success criteria).
   This *was* the decision gate; the val gain did not survive it.
2. **Do NOT build a higher-dose arm from generic synthetic.** More of a non-transferring signal won't
   help. Shelved.
3. **If synthetic is worth another attempt for opponents:** use **exact-mesh** opponents (the only transfer
   that worked — our-robot mrs_buff improved on both evals, matching `meshy_grade`'s sphinx-at-ceiling),
   not generic `synthgen` geometry, and grade on the independent eval from the start.
4. **Reusable finding for the keypoint model:** exact-CAD synthetic reliably lifts the class it depicts
   (mrs_buff AP +0.026 on the eval). Relevant to our-robot keypoint training, not this opponent detector.

## Risks / caveats

- **Domain gap is the whole risk.** Synthetic-only opponents floor at 0.084 alone; pooled they may still
  drag precision. On the val A/B this showed up as the small `robot` precision dip (−0.011); the
  `score.py` precision/F1 CI on the independent eval is the real test.
- **Class-balance confound.** Adding synthetic opponent boxes shifts the opponent:our-robot ratio, which
  by itself can move confidence calibration at conf 0.5 (see the `all_robots_pose` demotion effect).
  The converted synthetic set already carries our-robot boxes (remapped classes 3/4), so keep them in
  the mix and size arms by opponent-box count — that way the opponent:our-robot shift is the intended
  variable, not an accident.
- **Synthetic source is generic, not the eval's opponents.** The synthetic `nhrl_robot` boxes come from
  `all_robot_keypoints` (BlenderProc `synthgen`, generic CAD opponent geometry), not Meshy
  reconstructions of the specific robots in the eval. So the upside to expect is *general* opponent-shape
  / pose / background diversity, not a match to a named opponent. This is the right prior: the eval GT
  labels every opponent generically, and the `meshy_grade` result showed named-mesh transfer is
  per-opponent and fragile (sphinx faithful, clyde failed).
- **Per-class AP is not bootstrapped.** Opponent/house_bot/our-robot AP deltas are directional on small
  per-class samples; only agnostic aggregates carry significance verdicts. Do not over-read a per-class
  swing.
- **Same-run discipline.** Any absolute AP is only comparable *within one score.py run under one
  `--labels` mapping*. Do not cross-compare these numbers against other reports' opponent AP.

## Artifacts / locations (produced 2026-07-22)

- Converter: `training/yolo/pose_to_bbox.py`
- Synthetic source (unmodified): `training/data/all_robot_keypoints`
- Converted synthetic bbox set: `training/data/synth_bbox_from_keypoints` (label-only conversion)
- Pooled dataset: `training/data/mix_all` (real train + all synthetic; val real-only)
- Models (staged `.pt`): `data/models/yolo26n_mix_all_2026-07-22.pt`,
  `data/models/yolo26n_nhrl_robots_bbox_2026-07-16.pt` (baseline, reused). No engines built on megamind
  (§4); the dev-box follow-up builds `_x86_64_sm89`.
- Training run: `runs/projects/auto_battlebots_2026-07-22_02-40-51_yolo26n` (500 epochs, 19.9 h, batch 128)
- Val comparison: `runs/valcmp_real_bbox`, `runs/valcmp_mix_all`
- Writeup: `docs/experiments/perception_performance/synthetic_plus_bbox_2026-07-22.md`
- **Pending (dev box):** `training/data/nhrl_keypoints_eval_test/scores_synth_plus_bbox/` (independent-eval
  paired `score.py`)
