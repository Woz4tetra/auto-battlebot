# Synthetic + real bbox opponent detector — experiment plan

Status: **planned**. Follows the pipeline in `experiment_runbook.md` (collect → prepare megamind →
train → export → score). This document is the design; the writeup goes in
`synthetic_plus_bbox_<date>.md` once scored.

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
> synthetic diversity move the needle at all. If it clearly helps, an oversampled higher-dose arm becomes
> worth the compute; if it doesn't, higher dose of the same frames won't save it. This keeps the
> experiment to **one** ~500-epoch training run (the baseline is reused, not retrained).

**Pooling refinement (val stays real-only).** Rather than pool-then-resplit (which would leak easy
synthetic frames into val and make the val curve non-comparable to the reused baseline), `mix_all`
**adds synthetic only to the train split** and keeps `nhrl_robots_bbox`'s **real val unchanged**. Val is
then identical across baseline and `mix_all`, so the val mAP curve is a clean apples-to-apples plateau
signal. The verdict still comes from the external real eval, not val.

Both arms share the identical 5-class order, so **score them in one `score.py` run** — the paired
bootstrap cancels per-frame difficulty variance and resolves real differences on a few hundred frames.
Latency is identical across arms (same head, same class count, same output tensor `[1, 9, 8400]`), so no
latency measurement is needed — state that in the writeup rather than re-running trtexec.

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

## 2. Pool into each arm (Recipe C)

Hardlink (`os.link`, instant) the real `nhrl_robots_bbox` pairs plus a subset of the converted
`synth_bbox_from_keypoints` pairs into one flat `images/`+`labels/` with source-prefixed filenames. Do
**not** use `merge_yolo_datasets.py` (needs per-input `validation_state.json`, flat-only, drops metadata).

- `mix_1x`: link all real + a synthetic subset sized so synthetic opponent (`robot`) boxes ≈ real
  opponent boxes.
- `mix_3x`: link all real + a synthetic subset ≈ 3× real opponent boxes.
- `synth_only_opp` (optional): link real *non-opponent* pairs + all synthetic opponent pairs.

The converted synthetic set has ~36k opponent boxes in train — ample for 1× and likely 3×. Count opponent
boxes on both sides (real `object`+`robot` rows vs synthetic `robot` rows) to size each subset — do not
eyeball it. Sample synthetic *frames* (whole image/label pairs), not individual boxes, to keep labels
intact.

Split, author `data.yml`, validate (per runbook §1):

```bash
venv/bin/python training/yolo/split_yolo_dataset.py \
  training/data/<arm>/images training/data/<arm>/labels training/data/<arm> -t 0.9 -v 0.1
venv/bin/python training/yolo/validate_yolo_integrity.py training/data/<arm> --strict
```

`data.yml` is the **detect** form (no `kpt_shape` / `flip_idx`):

```yaml
path: /home/ben/auto-battlebot/training/data/<arm>   # location ON megamind
train: train/images
val:   val/images
test:  test/images
nc: 5
names: [object, robot, house_bot, mr_stabs_mk2, mrs_buff_mk3]
colors: [ "#...", "#...", "#...", "#...", "#..." ]
```

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

## 4. Export engines (on megamind — x86_64 **sm86**)

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

## 5. Score (one paired run)

All arms have the identical 5-class order, so a single run gives paired-bootstrap significance vs the
real-only baseline:

```bash
venv/bin/python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate real_bbox=data/models/yolo26n_nhrl_robots_bbox_2026-07-16_x86_64_sm86.engine \
  --candidate mix_1x=data/models/yolo26n_mix_1x_<date>_x86_64_sm86.engine \
  --candidate mix_3x=data/models/yolo26n_mix_3x_<date>_x86_64_sm86.engine \
  --labels "object,robot,house_bot,mr_stabs_mk2,mrs_buff_mk3" \
  --taxonomy training/model_eval/taxonomy.yaml --conf 0.5 --baseline real_bbox \
  --output training/data/nhrl_keypoints_eval_test/scores_synth_plus_bbox
```

All engines here are `_x86_64_sm86` because this run scores on megamind — including the rebuilt baseline
(§4). Do not mix an sm89 engine into this run; it will fail to load.

> **Two constraints force a different scorer on megamind.**
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
    ("real_bbox", "runs/projects/auto_battlebots_2026-07-16_20-50-57_yolo26n/weights/best.pt"),
    ("mix_all",   "runs/projects/<mix_all_run>/weights/best.pt"),
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

## Success criteria

Primary (significance-tested, agnostic level, paired vs `real_bbox`):
- **Win:** a mix arm raises **agnostic recall** with a CI excluding 0, and **does not** significantly
  drop precision/F1. That is synthetic adding genuine opponent-finding ability.
- **Neutral:** recall CI includes 0 — synthetic neither helps nor hurts the detector the pipeline reads;
  keep real-only for simplicity.
- **Loss:** precision/F1 drops significantly (synthetic domain gap injecting false boxes), or recall
  drops — reject that mix ratio.

Secondary (directional, archetype level):
- **`ap50_95/opponent`** trend across `real_bbox → mix_1x → mix_3x`: rising then flat/falling is the
  expected diversity-then-saturation curve. Reference points: floor 0.084 (synthetic-only), current
  real-only bbox ≈ 0.30 in-run.
- **Regression guard:** `ap50_95/house_bot` and `ap50_95/mrs_buff_mk3` must not fall materially — a mix
  that lifts opponents by wrecking our-robot boxes is not deployable (target selection needs our robot).

## Risks / caveats

- **Domain gap is the whole risk.** Synthetic-only opponents floor at 0.084 alone; pooled they may still
  drag precision. That is exactly what the precision/F1 significance test is for.
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

## Artifacts / locations

- Synthetic source (unmodified): `training/data/all_robot_keypoints` (existing synthetic keypoints)
- Converted synthetic bbox set: `training/data/synth_bbox_from_keypoints` (new; label-only conversion)
- Pooled datasets: `training/data/{mix_1x,mix_3x,synth_only_opp}` (all local on megamind this run)
- Models: `data/models/yolo26n_<arm>_<date>.{pt,onnx,engine}` (engine `_x86_64_sm86`, this run)
- Baseline engine (rebuilt for sm86): `data/models/yolo26n_nhrl_robots_bbox_2026-07-16_x86_64_sm86.engine`
- Scores: `training/data/nhrl_keypoints_eval_test/scores_synth_plus_bbox/{summary.csv,
  significance.csv, headline.png, confusion_*.png}`
- Writeup: `docs/experiments/perception_performance/synthetic_plus_bbox_<date>.md`
