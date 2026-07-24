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
| synthetic source | — **removed** (bbox trains real-only; see 2026-07-23) | synthetic renders in `all_robot_keypoints` (~97%) |
| `--labels` | `object,robot,house_bot,mr_stabs_mk2,mrs_buff_mk3` | `mr_stabs_mk2,mrs_buff_mk3` |
| taxonomy | `taxonomy.yaml` | `taxonomy_keypoint.yaml` |
| primary parity metric | agnostic **recall** (+ precision/F1 guard) | our-robot **recall** + **heading acc @10°** |
| secondary | opponent AP50-95 (directional) | `kp_err_px`, PCK@0.1, heading error |

> **Update (2026-07-23): the synthetic axis differs by phase, and Phase A's is already settled.**
> `synthetic_plus_bbox_2026-07-22.md` graded generic synthetic opponents on *this same independent eval*
> and found **no benefit** — all 12 paired-bootstrap tests `ns`, opponent point estimates trend negative
> (agnostic recall −0.013, opponent AP −0.033). Opponent is an *open category* the detector recognizes by
> scene context ("compact fast object on the floor, not us, not the house bot"), not by appearance, so
> adding synthetic opponent *appearance* cannot buy down real opponent data. **Phase A therefore drops the
> synthetic-substitution question — it is answered (no) — and studies only the real-image floor and the
> epoch floor.** The one synthetic signal that *did* transfer, on both val and eval, is **exact-CAD renders
> of our own robot** (`mrs_buff_mk3` AP +0.026 eval): synthetic works for the *instance* it depicts, not
> the open opponent class. That is exactly Phase B's regime.

Phase B is therefore the load-bearing half. `all_robot_keypoints` is already ~97% synthetic exact-CAD
our-robot renders, so the real question is *how little real our-robot data is needed on top of abundant
exact-CAD synthetic to hold recall and heading accuracy* — the substitution the 2026-07-22 result predicts
should work (instance transfer), unlike Phase A's open-category opponents. Run Phase A first (cheaper —
settles the epoch and real-count floors and the warm-start method), then Phase B (where synthetic×real
substitution is live and worth a grid).

**Run Phase A end to end before starting Phase B** — the methodology refinements you discover in A
(equivalence margin vs bootstrap width, how many checkpoints are worth scoring) carry into B.

## Baselines and parity definition

"Same results as baseline" is a **non-inferiority** claim, not equality, and must be measured paired.
In *every* scoring run, include the baseline engine as `--baseline` alongside the candidates so parity
is a paired-bootstrap delta under identical frames/thresholds — never compare a candidate against a
remembered number from another run (different `--labels`, different day; see the seg-vs-bbox caveat).

Reference baseline numbers on `nhrl_keypoints_eval_test` (direct-inference eval, for anchoring only —
re-measured live each run):

- **Phase A** (`synthetic_plus_bbox_2026-07-22`, independent eval of the `real_bbox` baseline; consistent
  with `seg_vs_bbox_2026-07-18`): agnostic recall **0.742**, precision **0.962**, F1 **0.838**, opponent
  AP50-95 **0.305**, agnostic mAP50-95 0.504. Baseline `.pt`: `yolo26n_nhrl_robots_bbox_2026-07-16`.
- **Phase B** (`baseline_2026-07-07` addendum, keypoint): agnostic recall **0.852**, precision 0.939,
  mAP50 0.844, `kp_err` **9.6 px**, PCK@0.1 0.729, heading error **7.6°**, heading acc @10° **0.814**.

**Parity / equivalence margin.** A candidate reaches parity when, at the agnostic level vs the
baseline:

- primary recall delta CI **lower bound ≥ −δ**, and
- precision and F1 are **not significantly worse** (their delta CIs may include 0 or be positive).

Set **δ = 0.04** initially. The floor on δ is the bootstrap's own resolution: on ~372 reviewed
frames the *measured* agnostic-recall delta CI was **[−0.036, +0.010]** (half-width ~0.023) in
`synthetic_plus_bbox_2026-07-22`, so an equivalence margin tighter than ~0.023 is unfalsifiable and 0.04
gives headroom. **Confirm the CI half-width for this baseline in §Method step 0 and set δ to
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
   **cannot** set a real:synthetic ratio, and it cannot hold real fixed while varying synthetic — that
   is lever 3. Use `fraction` only where the training set is single-source.
3. **`--synth-fraction f` + `--synth-order {sequential,random}`** (Phase B only) — a dataset-build lever
   that holds the **real** image set fixed and pools in only a fraction `f` of the synthetic our-robot
   renders, selected either in **image-numbered order** (`sequential`) or by **random** sampling.
   Ultralytics' `fraction` cannot do this — it subsamples one merged dataset and cannot tell real from
   synthetic — so this is wired into the Recipe C pooling step (hardlink only the selected synthetic
   frames; real frames always fully included), **not** `train.py`. Recipe C is currently a *manual*
   `os.link` hardlink procedure (`experiment_runbook.md` §Recipe C; `merge_yolo_datasets.py` is explicitly
   unusable here — it needs a per-input `validation_state.json`, ingests only flat datasets, and drops
   `flip_idx`), so this lever lands as a **new small pooling helper** (e.g.
   `training/yolo/pool_datasets.py`) that hardlinks the full real set plus the `--synth-fraction`/
   `--synth-order`-selected synthetic frames into one flat `images/`+`labels/`, then feeds the existing
   `split_yolo_dataset.py` → `validate_yolo_integrity.py --strict` steps. Sequential-vs-random at matched `f`
   is exactly the "how much synthetic *scene variation* do I need" probe (Exp 3B): `sequential` renders
   are scene-correlated (consecutive frames of the same render sequence), `random` spreads coverage
   across scenes, so a gap between the two accuracy-vs-`f` curves means variation — not raw synthetic
   count — is what buys accuracy.

Levers 1–2 are ~5-line additions to `train.py`'s arg parsing + the `settings`/`train()` call; lever 3 is
the new pooling helper below. Add them, run `./scripts/lint`, and confirm (a) a 2-epoch
`--save-period 1 --fraction 0.1` smoke run emits the extra checkpoints and (b) a `--synth-fraction 0.25
--synth-order sequential` pooled build contains the full real set plus exactly 25% of the synthetic
frames, before committing compute.

**Helper to write — `training/yolo/pool_datasets.py`** (Recipe C has no committed script; it is a manual
`os.link` procedure today). Small, deterministic, hardlink-based:

- **CLI:** `pool_datasets.py --real <dir> --synth <dir> --out <dir> --synth-fraction f
  --synth-order {sequential,random} [--seed S]`. `--real`/`--synth` each point at an
  `images/`+`labels/` source (or a flat dir where the label is a stem-matched sidecar).
- **Selection:** always hardlink **every** real pair. From synth, keep `round(f · N_synth)` frames —
  `sequential` = sort by filename then take the first `f` (filenames carry a `…T17-01-18-000050…`
  timestamp+frame index, so filename order **is** render-sequence order → scene-correlated); `random` =
  seeded `random.sample` (require `--seed`, print it, so an arm is reproducible and comparable across the
  sequential/random pair).
- **Hardlink `os.link`, both members of each pair together** — the image and its stem-matched label
  sidecar, whatever the extension (YOLO `.txt` for bbox; keypoint sidecars such as `.npy` for pose — do
  not assume `.txt`). Source-prefix output filenames (`real__…`, `synth__…`) so the two corpora never
  collide, matching the runbook's Recipe C convention.
- **Output** is one flat `images/`+`labels/` ready to hand to `split_yolo_dataset.py` →
  `validate_yolo_integrity.py --strict`. **Pool into the train split only:** to honor "val real-only and
  identical across arms," carve a fixed real-only val split **once** and reuse it, or run
  `split_yolo_dataset.py` on the real set alone for val and add synthetic to train only — never let
  synthetic frames land in val.
- **Print** the counts it linked (`real N_r, synth N_s (f=…, order=…, seed=…)`) so the smoke check above
  is a one-line verification and every arm's provenance is in its log.

Then measure the **noise floor**: score the baseline engine against itself via a second
bootstrap-resampled scoring of the same GT (or simply read the CI half-widths from a baseline-only
`score.py` run). This sets δ (above). Do this once per phase.

### Exp 1 — epoch floor, cold start

*How few epochs does a from-scratch (COCO-pretrained) run need to reach parity on the full real data?*

- **Recon (1 run).** Launch the standard full-data run (`train.py <data.yml> <model> --save-period 50`),
  full 500 epochs, but emit checkpoints every 50 epochs. Watch `results.csv`
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

Warm bases to build/choose, all fine-tuned with `fine_tune_train.py` (lr0 0.001, warmup_bias_lr 0.01 —
already the warm-start recipe). **The candidate set differs by phase, because bbox trains on no synthetic
(2026-07-23):**

- **`C_synth`** *(Phase B only)* — pretrain `yolo26n-pose` on the **abundant exact-CAD synthetic**
  our-robot corpus only (unlimited, no real-label cost), to convergence. This is the intended *reusable*
  base for the keypoint model: synthetic is free, so a synthetic-pretrained checkpoint costs nothing to
  regenerate and encodes robot-shape/pose priors. **Not a Phase A candidate** — the opponent detector
  trains real-only, and the 2026-07-23 result shows synthetic-opponent appearance is
  wrong-feature/wrong-context (it cost `mix_all` −0.033 opponent AP), so there is no synthetic bbox base
  to build.
- **`C_real`** *(Phase A warm candidate)* — an early checkpoint from the real-only cold-start run
  (Exp 1), reused as a warm base for the real-only data-floor sweep. This is Phase A's only non-baseline
  warm option now that synthetic is off the table; "warm-start buys nothing over COCO cold-start" is a
  live and acceptable outcome, in which case Phase A simply stays cold-start.
- **`C_base`** *(both phases, ceiling anchor)* — the existing baseline `.pt` itself. Fine-tuning from it
  trivially starts at parity, so it does **not** answer "reach parity cheaply" for a
  reproduce-the-baseline task; it *does* bound how fast any warm start can converge and is the right base
  when the goal is *adapting* the baseline to new data (new robot/event). Report it as the ceiling, not
  the recommendation.

For each phase's candidates, run the `fine_tune_train.py` convergence curve with `--save-period 50` on the
full real data and score the ladder exactly as Exp 1. Produce **epochs-to-parity** for cold (`E_cold`,
from Exp 1) vs the phase's warm base (`E_warm`): warm-from-`C_synth` for Phase B, warm-from-`C_real` for
Phase A. The hypothesis: `E_warm ≪ E_cold` (tens vs hundreds) — strongest for Phase B (instance transfer),
weakest and possibly null for Phase A.

Output: recommended reusable base checkpoint `C*` (expected `C_synth` for Phase B; `C_real` or "none, stay
cold-start" for Phase A), committed to `data/models/<model>_warmbase_<date>.pt` with its recipe, and
`E_warm`.

### Exp 3 — data floor (run in the cheap regime from Exp 2)

*Holding epochs at the cheap warm regime, how little training data still clears parity?* Val fixed to the
**real held-out** split throughout (identical across arms → comparable curves; **verdict always from the
external eval, never val** — see the val-misled-us caveat below). Each arm: warm-start from `C*`,
`E_warm` epochs, export, score paired vs baseline. The design differs by phase because the 2026-07-23
result settled Phase A's synthetic axis:

**Phase A — real-only 1-D sweep (no synthetic).** The synthetic-substitution question is answered (no),
so do **not** re-run a real×synthetic grid for the opponent detector. Sweep the single real axis via
`--fraction`: `{12.5%, 25%, 50%, 100%}` of the real train split. Smallest fraction clearing parity is the
**real-image floor**. No synthetic arm is run for the opponent detector — the substitution question is
settled (no), and bbox trains real-only.

**Phase B — real×synthetic 2-D grid (the live substitution study).** This is where synthetic is expected
to substitute for scarce real. Grid:

- **real axis** (real our-robot keypoint frames, sampled whole-frame): `{12.5%, 25%, 50%, 100%}` of the
  available real our-robot set.
- **synthetic axis** (exact-CAD our-robot renders from `all_robot_keypoints`, pooled via Recipe C):
  `{0, 1×, abundant}`, sized by **real-frame count** — sample whole frames, not boxes.

Run the informative diagonal first (small-real + abundant-synthetic, and large-real + no-synthetic); fill
in only where the frontier is ambiguous. The load-bearing Phase B question: **does abundant exact-CAD
synthetic let a much smaller `N_real` still hold recall *and* heading acc @10°** (heading is the gate that
a synthetic-only pose can silently fail — see risks).

**Exp 3B — synthetic scene-variation probe (sequential vs random).** Hold real fixed at a chosen `N_real`
(keep the real image count the same across every arm) and use the `--synth-fraction`/`--synth-order` lever
to sweep synthetic fraction `f ∈ {0.1, 0.25, 0.5, 1.0}` **twice**: once `--synth-order sequential`
(image-numbered, scene-correlated) and once `--synth-order random` (scene-spread). Two accuracy-vs-`f`
curves; the gap between them is the answer to *how much synthetic scene variation I need*. If `random`
clears parity at a much smaller `f` than `sequential`, scene **variation** (not raw synthetic count) is the
binding resource — future synthetic generation should prioritize diverse scenes over volume; if the two
curves coincide, count is what matters and cheap sequential renders suffice.

Output: Phase A real-image floor `N_real*`; Phase B images-to-parity Pareto frontier `(N_real, N_synth)`
plus the scene-variation verdict (sequential vs random `f`-to-parity at fixed `N_real`).

### Exp 4 — combined minimal recipe (one confirmation run)

Take the corner of the frontier: warm-start from `C*`, the smallest `(N_real, N_synth)` from Exp 3, and
re-confirm the epoch floor *at that data size* (small data may need a few more or fewer epochs than
`E_warm`, which was measured at full data). One end-to-end run at the proposed minimal recipe, scored
paired vs baseline, is the headline result: "this recipe matches the 500-epoch full-data baseline."

Output: the one-line recipe + its scored parity verdict.

## Execution notes (per `experiment_runbook.md`)

- **Where it runs:** training on megamind (3× A6000). `nhrl_keypoints_eval_test` was **uploaded to
  megamind on 2026-07-23** (`synthetic_plus_bbox_2026-07-22`), so the full train→export→score loop can now
  run **locally on megamind** with **`_x86_64_sm86`** engines (megamind's arch) — no dev-box round-trip.
  Score on the dev box instead only if you specifically want `_x86_64_sm89`. Engines are arch-specific;
  build to match the box you score on, and never score desktop-playback detections against live-labeled GT
  (rectification warp, see `baseline_2026-07-07`).
- **Reference compute anchor:** a full 500-epoch `yolo26n` detect run on ~67.5k images took **19.9 h** on
  the 3× A6000s (`synthetic_plus_bbox_2026-07-22`). That is the cost this experiment is trying to cut, and
  the yardstick for how much the epoch/data/warm-start floors actually save.
- **Do not run scoring/inference IO on megamind while a training arm is live** — it evicts the page
  cache and spikes epoch time ~30×. Score after arms finish or on an idle GPU.
- **Export per scored checkpoint** is `convert_to_onnx.py` → `convert_to_tensorrt.py --workspace 1`
  (`yolo26n` ~1 GiB). Scoring many ladder checkpoints means many exports — score a sensible subset
  (6–8 per curve), not every `save_period` dump.
- **Datasets:** real subsampling via `--fraction` needs no rebuild (Phase A, and Phase B's real axis);
  Phase B mixed-dose and scene-variation arms via Recipe C hardlink pooling (real always full; synthetic
  selected by `--synth-fraction`/`--synth-order`) + `split_yolo_dataset.py`,
  `validate_yolo_integrity.py --strict`. **Bbox (Phase A) trains real-only — no synthetic pooling.** Keep
  val real-only and identical across arms. Do **not** overwrite `all_robot_keypoints` or
  `nhrl_robots_bbox`.
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
- **Data floor quantified** — Phase A: a defensible smallest real fraction `N_real*` clearing parity for
  the opponent detector. Phase B: an `(N_real, N_synth)` frontier plus the substitution answer — does
  abundant exact-CAD synthetic let a smaller `N_real` still clear both the recall and heading gates
  (expected yes, per the 2026-07-23 instance-transfer result), and by how much.
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
- **Same-corpus val is a proven-unsafe proxy — grade on the independent eval from the start.** In
  `synthetic_plus_bbox_2026-07-22` the real val split showed a +0.0195 opponent-recall "win" that
  **reversed** to −0.013 on unseen fights: a change that fit the training distribution, not transferable
  signal. Every parity verdict here comes from `nhrl_keypoints_eval_test`. Use val only for live
  plateau-watching during a run, never to declare an arm at parity.
- **Tiny-real overfitting.** At 12.5% real the val curve may look fine while the external eval drops
  (memorized fights) — the same-corpus trap above, in its most acute form. Verdict is always the external
  eval.
- **Phase B heading is the load-bearing signal, not recall.** A synthetic-heavy low-real arm can hold
  detection recall while heading accuracy collapses (front/back keypoint from mostly-synthetic poses).
  The heading-acc @10° gate is mandatory in Phase B, not optional. Note the 2026-07-23 result is
  *encouraging* here: exact-CAD our-robot renders transfer (unlike generic opponents), so synthetic
  substitution is expected to hold heading better in Phase B than the opponent analogy would suggest —
  but confirm it, do not assume it.
- **Same-run comparability.** Absolute AP is comparable only within one `score.py` run under one
  `--labels` mapping. Never cross-compare arm AP against other reports.

## Artifacts / locations

- Instrumentation: `--save-period` / `--fraction` passthrough in `training/yolo/train.py`
  (and `fine_tune_train.py` for `--save-period`); `--synth-fraction` / `--synth-order` in a new Recipe C
  pooling helper (e.g. `training/yolo/pool_datasets.py`; Recipe C has no committed script today — it is a
  manual `os.link` procedure per `experiment_runbook.md`), Phase B only.
- Warm base checkpoint: `data/models/<model>_warmbase_<date>.{pt,onnx,engine}` (the reusable `C*`).
- Sweep models: `data/models/<model>_<arm>_<date>.{pt,onnx,engine}` (engine `_x86_64_sm86` if scoring on
  megamind — the default now that the eval set is there; `_x86_64_sm89` if scoring on the dev box).
- Pooled datasets: `training/data/<arm>/` (Recipe C, val real-only). Real substrates unmodified.
- Scores: `training/data/nhrl_keypoints_eval_test/scores_data_epoch_min_<phase>/{summary.csv,
  significance.csv, headline.png, confusion_*.png}`.
- Writeups: `docs/experiments/perception_performance/data_epoch_min_<phase>_<date>.md` (Phase A first,
  then Phase B).
