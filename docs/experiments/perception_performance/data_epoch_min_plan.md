# Minimum data + epochs to match baseline (warm-start checkpoints) — experiment plan

Status: **planned**. Follows the pipeline in `experiment_runbook.md` (collect → prepare megamind →
train → export → score). This document is the design; each phase's writeup goes in
`data_epoch_min_<phase>_<date>.md` once scored.

## Question

Every retrain today is a from-scratch ~500-epoch run on the full dataset. Most of that is probably
wasted. Three coupled levers set the cost of a retrain:

- **real image count** — scarce, low-diversity, expensive to label
- **synthetic image count** — unlimited but domain-gapped
- **epochs** — 500 by default, but box and pose plateau at different (earlier) epochs

**What is the smallest (real images, synthetic images, epochs) that still scores at parity with the
current baseline model — and can a single reusable warm-start checkpoint let every future training
reach that parity in a fraction of the epochs and data?**

The deliverable is not one number but three things: (1) a Pareto frontier of images-to-parity and
epochs-to-parity, (2) a committed, documented **base checkpoint** to fine-tune all future models from,
and (3) a one-line minimal recipe ("from `C*`, fine-tune `E` epochs on `N` real images → parity").

## Scope — two phases, one methodology

The same four sub-experiments run twice, once per detector family. The methodology (§Method) is
identical; only the baseline, dataset, `--labels`, and parity metric change.

| | Phase A — bbox opponent detector | Phase B — keypoint pose model |
|---|---|---|
| baseline model | `yolo26n_nhrl_robots_bbox_2026-07-16.pt` (detect, 500 ep) | `yolo26n-pose_our_robots_2026-05-01.pt` (pose, 500 ep) |
| model key | `yolo26n` (batch 128) | `yolo26n-pose` (batch 96) |
| real substrate | `nhrl_robots_bbox` (train 49086 / val 5454, real) | real our-robot keypoint frames in `all_robot_keypoints` (~3% of it) |
| synthetic source | `synth_bbox_from_keypoints` (converted, ~36k opp boxes) | synthetic renders in `all_robot_keypoints` (~97%) |
| `--labels` | `object,robot,house_bot,mr_stabs_mk2,mrs_buff_mk3` | `mr_stabs_mk2,mrs_buff_mk3` |
| taxonomy | `taxonomy.yaml` | `taxonomy_keypoint.yaml` |
| primary parity metric | agnostic **recall** (+ precision/F1 guard) | our-robot **recall** + **heading acc @10°** |
| secondary | opponent AP50-95 (directional) | `kp_err_px`, PCK@0.1, heading error |

Phase A's real/synthetic split is a clean "add synthetic to a real base" question. Phase B is the
inverse and arguably the more interesting one: `all_robot_keypoints` is already ~97% synthetic, so the
question becomes *how little real our-robot data is needed on top of abundant synthetic to hold heading
accuracy.* Run Phase A first (cheaper to reason about, active line of work), then Phase B.

**Run Phase A end to end before starting Phase B** — the methodology refinements you discover in A
(equivalence margin vs bootstrap width, how many checkpoints are worth scoring) carry into B.

## Baselines and parity definition

"Same results as baseline" is a **non-inferiority** claim, not equality, and must be measured paired.
In *every* scoring run, include the baseline engine as `--baseline` alongside the candidates so parity
is a paired-bootstrap delta under identical frames/thresholds — never compare a candidate against a
remembered number from another run (different `--labels`, different day; see the seg-vs-bbox caveat).

Reference baseline numbers on `nhrl_keypoints_eval_test` (direct-inference eval, for anchoring only —
re-measured live each run):

- **Phase A** (`seg_vs_bbox_2026-07-18`, bbox arm): agnostic recall **0.742**, precision **0.963**,
  F1 **0.838**, opponent AP50-95 **0.304**, agnostic mAP50-95 0.504.
- **Phase B** (`baseline_2026-07-07` addendum, keypoint): agnostic recall **0.852**, precision 0.939,
  mAP50 0.844, `kp_err` **9.6 px**, PCK@0.1 0.729, heading error **7.6°**, heading acc @10° **0.814**.

**Parity / equivalence margin.** A candidate reaches parity when, at the agnostic level vs the
baseline:

- primary recall delta CI **lower bound ≥ −δ**, and
- precision and F1 are **not significantly worse** (their delta CIs may include 0 or be positive).

Set **δ = 0.04** initially. The floor on δ is the bootstrap's own resolution: on ~372 reviewed
frames the seg-vs-bbox recall CI half-width was ~0.025, so an equivalence margin tighter than the CI
is unfalsifiable. **Compute the baseline-vs-itself-on-a-resample CI first (§Method step 0) and set δ to
max(0.04, 1.5 × CI half-width).** Phase B adds a second gate: heading acc @10° delta CI lower bound
≥ −0.03 (the signal navigation actually consumes; recall alone can hold while heading rots, as the
opponent-diluted combined model showed at 38.5°).

Record for each arm the *minimal* setting that clears parity — that point, not the best absolute score,
is the experiment's output.

## Method — the four sub-experiments (per phase)

Ordered so each cheap result shrinks the compute of the next. Later, more numerous arms run in the
reduced regime discovered earlier.

### Step 0 — instrument, and measure the parity noise floor

Two small enablers make the whole plan affordable; both are Ultralytics-native and currently unwired:

1. **`save_period`** — `model.train(..., save_period=N)` writes `weights/epoch{0,N,2N,...}.pt` in
   addition to `last.pt`/`best.pt`. This turns **one** long training run into an epoch-vs-metric
   curve: export and score a handful of the periodic checkpoints instead of launching one run per
   epoch count. Wire it through `train.py` as `--save-period N` (default 0 = current behavior).
2. **`fraction`** — `model.train(..., fraction=f)` trains on the first `f` of the (shuffled) train
   split, val untouched. This is the cheap real-data lever for the real-only sweeps (no dataset
   rebuild). Wire it as `--fraction f` (default 1.0). *Caveat:* `fraction` subsamples one dataset; it
   **cannot** set a real:synthetic ratio — the mixed-dose grid (Exp 3) still needs pooled datasets
   (Recipe C). Use `fraction` only where the training set is single-source.

Both are ~5-line additions to `train.py`'s arg parsing + the `settings`/`train()` call. Add them, run
`./scripts/check_and_fix`, and confirm a 2-epoch `--save-period 1 --fraction 0.1` smoke run emits the
extra checkpoints before committing compute.

Then measure the **noise floor**: score the baseline engine against itself via a second
bootstrap-resampled scoring of the same GT (or simply read the CI half-widths from a baseline-only
`score.py` run). This sets δ (above). Do this once per phase.

### Exp 1 — epoch floor, cold start

*How few epochs does a from-scratch (COCO-pretrained) run need to reach parity on the full real data?*

- **Recon (1 run).** Launch the standard full-data run (`train.py <data.yml> <model> --save-period 25`),
  full 500 epochs, but emit checkpoints every 25 epochs. Watch `results.csv`
  (`metrics/mAP50-95(B)` for A, `metrics/mAP50-95(P)` for B) live and note where val plateaus.
- **Score the ladder.** Export + score `epoch{50,100,150,200,300,500}.pt` (subset around the val
  plateau) against the baseline in one paired run per epoch value is wasteful — instead score them as
  separate candidates in a **single** `score.py` invocation (they share class order): the run reports
  each candidate's paired delta vs baseline. The smallest epoch whose recall clears the δ gate is the
  **cold-start epoch floor, long-schedule estimate** `E_cold^500`.
- **Confirm (2 runs) — the LR-schedule caveat.** Ultralytics anneals LR cosine over the *total* epoch
  budget, so `epoch150.pt` from a 500-epoch run is **not** what a dedicated 150-epoch run produces
  (the latter fully anneals; the former is mid-decay, LR still high). The ladder gives an *upper
  bound* on epochs and brackets the region. Confirm by launching **dedicated** runs at
  `E ∈ {E_cold^500, ~0.6·E_cold^500}` (e.g. `-e 150`, `-e 100`) — these anneal correctly and usually
  reach parity *at or below* the ladder estimate. The confirmed value is `E_cold`.

Output: `E_cold` (cold-start epochs-to-parity, full data).

### Exp 2 — warm-start base checkpoint (the crux)

*Does starting from a domain checkpoint instead of COCO cut epochs-to-parity and data-to-parity?*

Two warm bases to build/choose, both fine-tuned with `fine_tune_train.py` (lr0 0.001, warmup_bias_lr
0.01 — already the warm-start recipe):

- **`C_synth`** — pretrain `yolo26n`/`yolo26n-pose` on the **abundant synthetic** corpus only (unlimited,
  no real-label cost), to convergence. This is the intended *reusable* base: synthetic is free, so a
  synthetic-pretrained checkpoint costs nothing to regenerate and encodes robot-shape/pose priors.
- **`C_base`** — the existing baseline `.pt` itself, as an upper-anchor. Fine-tuning from it trivially
  starts at parity, so it does **not** answer "reach parity cheaply" for a reproduce-the-baseline task;
  it *does* bound how fast any warm start can converge and is the right base when the goal is *adapting*
  the baseline to new data (new robot/event). Report it as the ceiling, not the recommendation.

For each base, run the `fine_tune_train.py` convergence curve with `--save-period` on the full real
data and score the ladder exactly as Exp 1. Produce **epochs-to-parity** for cold (`E_cold`, from
Exp 1) vs warm-from-`C_synth` (`E_warm`). The hypothesis: `E_warm ≪ E_cold` (tens vs hundreds).

Output: recommended reusable base checkpoint `C*` (expected `C_synth`), committed to
`data/models/<model>_warmbase_<date>.pt` with its recipe, and `E_warm`.

### Exp 3 — data floor (real × synthetic grid), run in the cheap regime

*Holding epochs at the cheap warm regime from Exp 2, how little real (and how much synthetic) still
clears parity?*

Now that Exp 2 fixed a cheap regime (warm-start from `C*`, `E_warm` epochs), the data grid becomes
affordable. Coarse geometric ladder, val fixed to the **real held-out** split throughout (identical
across arms → comparable curves; verdict still from the external eval):

- **real axis** (via `--fraction`, real-only arms): `{12.5%, 25%, 50%, 100%}` of the real train split.
- **synthetic axis** (via pooled datasets, Recipe C): add `{0, 1×}` synthetic, sized by **box count**
  on the scarce class (opponent boxes for A, our-robot frames for B) — sample whole frames, not boxes.

Run the informative diagonal first (small-real + synthetic-added, and large-real + no-synthetic) rather
than the full 4×2 grid; fill in only where the frontier is ambiguous. Each arm: warm-start from `C*`,
`E_warm` epochs, export, score paired vs baseline. The smallest `(N_real, N_synth)` clearing parity is
the **data floor**; whether adding synthetic lets a smaller `N_real` clear it is the substitution
result (does free synthetic buy down scarce real labels).

Output: images-to-parity Pareto frontier `(N_real, N_synth)`.

### Exp 4 — combined minimal recipe (one confirmation run)

Take the corner of the frontier: warm-start from `C*`, the smallest `(N_real, N_synth)` from Exp 3, and
re-confirm the epoch floor *at that data size* (small data may need a few more or fewer epochs than
`E_warm`, which was measured at full data). One end-to-end run at the proposed minimal recipe, scored
paired vs baseline, is the headline result: "this recipe matches the 500-epoch full-data baseline."

Output: the one-line recipe + its scored parity verdict.

## Execution notes (per `experiment_runbook.md`)

- **Where it runs:** training on megamind (3× A6000), scoring on whichever box holds
  `nhrl_keypoints_eval_test` (the dev box — the eval set is a hand-reviewed artifact and is not on
  megamind). Engines are arch-specific: build `_x86_64_sm89` on the dev box for scoring; never score
  desktop-playback detections against live-labeled GT (rectification warp, see `baseline_2026-07-07`).
- **Do not run scoring/inference IO on megamind while a training arm is live** — it evicts the page
  cache and spikes epoch time ~30×. Score after arms finish or on an idle GPU.
- **Export per scored checkpoint** is `convert_to_onnx.py` → `convert_to_tensorrt.py --workspace 1`
  (`yolo26n` ~1 GiB). Scoring many ladder checkpoints means many exports — score a sensible subset
  (6–8 per curve), not every `save_period` dump.
- **Datasets:** real subsampling via `--fraction` needs no rebuild; mixed-dose arms via Recipe C
  hardlink pooling + `split_yolo_dataset.py`, `validate_yolo_integrity.py --strict`. Keep val real-only
  and identical across arms. Do **not** overwrite `all_robot_keypoints` or `nhrl_robots_bbox`.
- **score.py discipline:** `--labels` length must equal the engine class count (check the printed
  `num_keypoints=N num_classes=M` line); candidates in one run must share class order; always pass
  `--baseline <baseline_name>` and `--conf` matching the deployed config (blob 0.6 / keypoint 0.5).

## Success criteria

- **Primary deliverable met** if Exp 4's minimal recipe scores at parity (recall δ-gate + precision/F1
  not-worse; Phase B also heading-acc gate) against the baseline in a paired run — i.e. a documented
  training regime materially cheaper than 500-epoch-full-data reproduces the baseline.
- **Warm-start value quantified** if Exp 2 shows `E_warm` with CI clearly below `E_cold` (the
  epochs-to-parity reduction from the reusable checkpoint) — the direct answer to "can I start all
  training from a checkpoint and reach the same result in minimal epochs."
- **Data floor quantified** if Exp 3 yields a defensible smallest `(N_real, N_synth)` on the frontier,
  and reports whether synthetic substitutes for real (smaller `N_real` clears parity with synthetic
  added than without).
- **Neutral / negative results are results:** if `E_warm ≈ E_cold`, the checkpoint buys nothing and
  future retrains should stay cold-start (simpler); if no reduced regime clears δ, report the cheapest
  that comes closest and the gap.

## Risks / caveats

- **LR-schedule ≠ epoch-prefix.** The `save_period` ladder from one long run is a *bracketing upper
  bound*, not the answer — cosine LR is stretched over the total budget. Every reported epoch floor
  must be a **dedicated-schedule** confirmation run (Exp 1/2 confirm step), not a mid-run checkpoint.
- **Warm-from-baseline is circular.** `C_base` trivially matches at epoch 0; it measures adaptation
  speed, not cheap reproduction. The recommended reusable base is `C_synth` (free to regenerate,
  encodes no eval leakage). State this explicitly in the writeup.
- **Eval set is small (~372 frames)** → wide parity CIs. δ must exceed the bootstrap CI half-width
  (Step 0); do not claim parity inside the noise. Per-class AP (opponent) is directional only, not
  bootstrapped — read as trend.
- **Augmentation × short schedules.** Heavy aug (mosaic 0.4, `close_mosaic` last epochs, mixup,
  copy_paste) is tuned for long runs; at very low epochs it can *slow* convergence. Hold aug fixed for
  the primary sweep (one variable), but flag it as the obvious next lever if the epoch floor is
  disappointingly high.
- **Tiny-real overfitting.** At 12.5% real the val curve may look fine while the external eval drops
  (memorized fights). The verdict is always the external `nhrl_keypoints_eval_test`, never val.
- **Phase B heading is the load-bearing signal, not recall.** A synthetic-heavy low-real arm can hold
  detection recall while heading accuracy collapses (front/back keypoint from mostly-synthetic poses).
  The heading-acc @10° gate is mandatory in Phase B, not optional.
- **Same-run comparability.** Absolute AP is comparable only within one `score.py` run under one
  `--labels` mapping. Never cross-compare arm AP against other reports.

## Artifacts / locations

- Instrumentation: `--save-period` / `--fraction` passthrough in `training/yolo/train.py`
  (and `fine_tune_train.py` for `--save-period`).
- Warm base checkpoint: `data/models/<model>_warmbase_<date>.{pt,onnx,engine}` (the reusable `C*`).
- Sweep models: `data/models/<model>_<arm>_<date>.{pt,onnx,engine}` (engine `_x86_64_sm89`, dev-box scoring).
- Pooled datasets: `training/data/<arm>/` (Recipe C, val real-only). Real substrates unmodified.
- Scores: `training/data/nhrl_keypoints_eval_test/scores_data_epoch_min_<phase>/{summary.csv,
  significance.csv, headline.png, confusion_*.png}`.
- Writeups: `docs/experiments/perception_performance/data_epoch_min_<phase>_<date>.md` (Phase A first,
  then Phase B).
