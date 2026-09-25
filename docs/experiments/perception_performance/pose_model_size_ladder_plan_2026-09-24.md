# Pose model size ladder: all five `yolo26` sizes on a leak-free `d50000`

This plan trains `yolo26n-pose`, `s`, `m`, `l` and `x` on one arm, with one schedule and one seed,
and scores all five on the cage-high eval set (`training/data/cage_high_x50_conf044`) in a single
paired bootstrap. It expands
[synthetic_domain_mix_2026-09-18.md](synthetic_domain_mix_2026-09-18.md), which only ever
trained `s` and `x`. Its results feed
[pose_model_ml_60hz_plan_2026-09-24.md](pose_model_ml_60hz_plan_2026-09-24.md). That plan needs an
`m`-against-`s` read to decide whether the ZED Box can run something bigger than `s` at 60 Hz, and
every arm it would compare against saw the eval fights in training.

**Status, 2026-09-25.** Prep is done and all five arms are queued (jobs 33 to 37); `s` and `m`
have finished. The eval set changed on 2026-09-25, before any cage-high number was read: see
[Eval set: cage-high, not the ZED 2i](#eval-set-cage-high-not-the-zed-2i).

## Eval set: cage-high, not the ZED 2i

The first draft scored on `nhrl_keypoints_eval_test`, the robot's own ZED 2i footage. That is the
wrong target. The domain renders this arm is built on were modelled on cage-high broadcast
footage, so the camera the ladder should be measured on is the cage mount, and the ZED 2i images
are not used to score anything here.

The eval set is `training/data/cage_high_x50_conf044/d40000`, described in
[cage_high_eval_scoring_2026-09-18.md](cage_high_eval_scoring_2026-09-18.md): 636 hand-corrected
frames, `pass` in `validation_state.json`, over six NHRL Brettzone `Cage-N-Overhead-High`
recordings (516 frames) and three MassD broadcast recordings (120 frames). Every match is Mrs Buff
against one opponent.

Two properties of this set bear on every number read from it:

- **Recall saturates and precision separates.** A cage mount shows a robot about 5.5 times the
  area the ZED does. On the grid, `base` recalled 0.856 here against 0.339 on the ZED, fifteen of
  eighteen arms landed between 0.81 and 0.93 recall, and precision spanned 0.348 to 0.969. Q1 to
  Q3 still read recall first, as registered, but F1 and precision carry most of the signal here.
- **The ground truth was seeded by `x_d40000_ep50`** (`yolo26x-pose_d40000_2026-09-16_last.pt`,
  conf 0.44, no blind hold-out) and then corrected by hand. No arm in the ladder is that run, but
  its bias toward an `x`-shaped output is in the labels, most of all the keypoints. Read Q4 with
  that in mind, and treat the `x_d40000_ep50` anchor as scored partly against itself.

The ZED read of `m` against `s` taken on 2026-09-25, before this change, stays out of the
questions. It is reported in the results write-up as an unregistered cross-camera row.

## Why `d50000_cagehigh` is out

Every `_cagehigh` arm carries the 636 frames of `training/data/cage_high_x50_conf044`, which is
the eval set itself. An arm trained on those frames has seen every frame it would be scored on.
(They are also broadcast frames of the same `nhrl_may26` fights the ZED set recorded, which is
why the first draft ruled them out there too.)

Checked on megamind, 2026-09-25, resolving every path: all 636 eval images are in
`d50000_cagehigh.txt`, none are in `d50000.txt`, and none are in `val.txt`. The renders are 3D
Blender scenes with sampled camera mounts, not composites over broadcast frames, so no eval frame
reaches `d50000` through them.

Checked on megamind, 2026-09-24, in `training/data/domain_mix_arms_2026-09-19`:

| List | Lines | `cage_high_x50_conf044` | Other sources |
| --- | ---: | ---: | --- |
| `d50000_cagehigh.txt` | 69,083 | 636 | 18,447 corpus, 20,000 NHRL, 20,000 MassD, 10,000 basement |
| `d40000.txt` | 58,447 | 0 | 18,447 corpus, 20,000 NHRL, 20,000 MassD |
| `val.txt` | 2,049 | 0 | corpus only |

`d40000_cagehigh` and `swap_half_cagehigh` carry the same 636 frames, so they are out too. The
corpus is clean: its 452 real frames are `mrs-buff-mk3-keypoints` sessions from the plywood test
box, and none resolves to an eval image.

This also means every `d50000_cagehigh` or `cagehigh` engine already in `data/models`
(`s_d50000_cagehigh`, `x_d50000_cagehigh`, `s1280`, and the two controls) cannot be scored on this
eval set. The leak does not stop an engine from being deployed. It stops it from being measured.

## The arm

`d50000`: `d50000_cagehigh` minus the cage-high frames.

| Source | Frames |
| --- | ---: |
| Randomized synthetic (via `all_robot_keypoints`) | 17,995 |
| Domain render, `nhrl_cage`, a third per view | 20,000 |
| Domain render, `massd_arena`, a third per view | 20,000 |
| Domain render, `meatball_basement`, a third per view | 10,000 |
| Real, `all_robot_keypoints` | 452 |
| Total | 68,447 |

Real share is 0.66 percent, against `d40000`'s 0.77. `real3x` showed that real share was not the
cause of the recall collapse, so this arm does not oversample to compensate.

Why this arm and not `swap_half`, which won the grid's pre-registered rule on ZED footage:

- It is the mix the 60 Hz plan and the deployed ZED Box engine already use, minus the leak. The
  ladder's answer then carries straight over to the model that ships.
- The basement render is the only venue that shares a scene with the real frames and the only one
  with a low mount near the ZED One S's 1.2 m. The grid never scored it on ZED footage.
- The grid's domain-heavy loss on ZED footage was a schedule effect: `d40000` scored 0.606 at
  epoch 50 and 0.295 at epoch 100, and `d40000_s50` reached 0.485. This arm trains to the ~2 M
  presentation peak, not past it.

`swap_half` stays as a reference row in scoring (below), so the ladder can still be read against
the grid's recall leader.

## Arms

| Arm | Model | Epochs | Presentations | Queue estimate | `--eta` |
| --- | --- | ---: | ---: | --- | --- |
| `s_d50000` | `yolo26s-pose` | 30 | 2.05 M | 2 h 18 min (job 28, same size list) | none, profile has history |
| `m_d50000` | `yolo26m-pose` | 30 | 2.05 M | ~3 h 20 min (GFLOPs interpolation, 60 Hz plan) | `4h` |
| `x_d50000` | `yolo26x-pose` | 30 | 2.05 M | 5 h 59 min (job 29) | none, profile has history |
| `n_d50000` | `yolo26n-pose` | 30 | 2.05 M | ~2 h, likely loader-bound like `s` | `3h` |
| `l_d50000` | `yolo26l-pose` | 30 | 2.05 M | ~3 h 40 min (GFLOPs interpolation) | `4h30m` |

About 17 h 20 min of queue in all. Everything else is the grid's constants: `imgsz 640`, `-b 96`,
3-GPU DDP, `--seed 0`, `--cache ram`, pretrained start, `--save-period 10` (epoch10, epoch20,
last), square and `384x640` exports. Ship `last.pt`.

The queue order is deliberate:

1. **`s` first.** It checks the new list and the RAM cache in 2 h 18 min before anything
   expensive inherits them, and it is the bootstrap baseline.
2. **`m` second**, because it is the arm the 60 Hz plan is waiting on. Once `m` is scored against
   `s`, the 60 Hz plan can proceed without waiting for the other three.
3. **`x`, `n`, `l`.** `l` goes last because gate L already dropped it from deployment
   (54.7 Hz, `runner.tick` p95 20.07 ms). It is trained only to fill in the size curve. If the
   queue is contended, `l` is the arm to cut.

## Prep, before submitting

1. **Add the arm to `make_domain_mix_arms.py`.**
   `Arm("d50000", None, 50000, venues=ALL_VENUES)`, and add `("d40000", "d50000")` to `NESTED`.
   Rebuild the whole grid into a scratch directory and confirm every existing list, `val.txt`
   included, reproduces byte for byte, the check the `nodamage_swap_half` and `_cagehigh`
   additions both passed.
2. **Check the new list for leaks.** It must be 68,447 lines, and
   `grep -c -E 'cage_high|eval_test' d50000.txt` must print 0. Diffing it against
   `d50000_cagehigh.txt` must show exactly the 636 cage-high lines.
3. **Add `yolo26m-pose` and `yolo26l-pose` presets to `train.py`.** `configs` holds
   `n`, `s` and `x` pose but not `m` or `l`, and `settings = dict(configs[model_key])` raises
   `KeyError` on either. The 60 Hz plan's `m` submit would have failed on this too. Copy the
   `yolo26s-pose` entry. `run_domain_mix_arm.sh` passes batch, epochs, image size and cache
   explicitly, so the preset values only matter as single-GPU defaults.
4. **Clear the GPU 2 blocker.** On 2026-09-24 `gpu_queue.py status` still shows
   `VLLM::EngineCore` (pid 1021286) holding 33.5 GiB outside the queue, with the queue empty and
   the worker stopped. Submitting is fine, since jobs wait for idle GPUs. The process is not ours,
   so ask whoever owns it.

## Commands

On megamind, from the repo root, after prep:

```bash
venv/bin/python training/yolo/make_domain_mix_arms.py \
  --corpus training/data/all_robot_keypoints \
  --domain training/data/synth_cage_nhrl_2026-09-13_v2 training/data/synth_cage_massd_2026-09-13 \
    training/data/synth_cage_basement_2026-09-19 \
  --extra-real training/data/cage_high_x50_conf044 \
  --out training/data/domain_mix_arms_2026-09-19 --only d50000

venv/bin/python training/gpu_queue.py status

A=training/data/domain_mix_arms_2026-09-19
W=2053410   # 68,447 frames x 30 epochs

venv/bin/python training/gpu_queue.py submit --name sz_s_d50000 --by <agent> -d 0 1 2 \
  --work $W --profile yolo26s-pose@640 -- \
  bash training/yolo/run_domain_mix_arm.sh $A d50000 yolo26s-pose 30 --save-period 10
venv/bin/python training/gpu_queue.py submit --name sz_m_d50000 --by <agent> -d 0 1 2 \
  --work $W --profile yolo26m-pose@640 --eta 4h -- \
  bash training/yolo/run_domain_mix_arm.sh $A d50000 yolo26m-pose 30 --save-period 10
venv/bin/python training/gpu_queue.py submit --name sz_x_d50000 --by <agent> -d 0 1 2 \
  --work $W --profile yolo26x-pose@640 -- \
  bash training/yolo/run_domain_mix_arm.sh $A d50000 yolo26x-pose 30 --save-period 10
venv/bin/python training/gpu_queue.py submit --name sz_n_d50000 --by <agent> -d 0 1 2 \
  --work $W --profile yolo26n-pose@640 --eta 3h -- \
  bash training/yolo/run_domain_mix_arm.sh $A d50000 yolo26n-pose 30 --save-period 10
venv/bin/python training/gpu_queue.py submit --name sz_l_d50000 --by <agent> -d 0 1 2 \
  --work $W --profile yolo26l-pose@640 --eta 4h30m -- \
  bash training/yolo/run_domain_mix_arm.sh $A d50000 yolo26l-pose 30 --save-period 10
```

Check the `--only` flag and the `--extra-real` handling against the script before running: the
new arm must not pick up the extra real frames even though the flag is passed to keep the
manifest's other arms reproducible.

Each job writes `data/models/yolo26{n,s,m,l,x}-pose_d50000_<date>_{epoch10,epoch20,last}.pt`,
square and `rect384x640` ONNX, and megamind's `sm86` engines.

## Engines

Score on megamind's `sm86` engines, which `run_domain_mix_arm.sh` builds at the end of each job,
rectangular and square. The cage-high set lives on megamind, and the grid scored it there on
`sm86`. The first draft named pathfinder's `sm89` engines; the three grid anchors built there
reproduced their `sm86` recall to within 0.001 (0.562, 0.486, 0.419 against 0.562, 0.485,
0.419 on the ZED set), so the engine platform does not move a number at the precision these
questions read.

For the ZED Box, build `m` (and `n`, if it passes Q3) from the same ONNX on the box's
TensorRT 10.3. Rename on copy to `yolo26m-pose_d50000_rect384x640_<date>_aarch64_sm87.engine`,
since a rebuild does not change the filename.

## Questions, written before anything runs

Do not move these afterwards. The noise floor is about 0.05 opponent recall: `base` spanned 0.284
to 0.367 over its own checkpoints, and `int8_quantization_2026-09-06.md` measured ~0.048
run-to-run. Every arm here is one seed, so a point difference under 0.05 is not a finding, even
when the bootstrap CI excludes zero. The bootstrap resamples frames, not training runs.

The questions below were written for the ZED set and moved to the cage-high set on 2026-09-25,
before any cage-high number was read. Their metrics and thresholds are unchanged; only the frames
and the reference numbers moved. The 0.05 floor was measured on the ZED set, and no run-to-run
figure exists for cage-high, so it is kept as the floor there too.

- **Q1, the size curve.** Does opponent recall rise with model size on cage-high footage? The
  grid had two points on this set: `x_d40000_ep50` 0.910 against `d40000_ep50` 0.895, with
  precision 0.917 against 0.792. `model_size_2026-09-04.md` found the bbox detector flat from `s`
  to `l`. Report recall, precision, F1 and heading error for all five and read the shape. No
  adoption rule hangs on Q1.
- **Q2, `m` for the 60 Hz plan.** This is the 60 Hz plan's accuracy criterion, unchanged:
  `m`'s opponent recall above `s` with a CI excluding zero, and our-robot heading error no worse
  than `s`. Pass: the 60 Hz plan ships `m_d50000` if its live latency check also passes. Fail:
  the ZED Box stays on `s`.
- **Q3, `n` as the cheap option.** Does `n` hold opponent recall within 0.03 of `s` and heading
  error within 1 degree? Pass: `n` becomes the fallback if the motion detector or anything else
  claims part of the 16.7 ms frame budget.
- **Q4, does `x`'s keypoint lead survive?** On this set `x_d40000_ep50` scored opponent keypoint
  error 6.9 px and heading 5.3 degrees against 13 to 24 px and 8 to 20 degrees for every `s` arm,
  but it seeded the ground truth, so part of that lead is its own output. Does `x_d50000` hold a
  heading lead over `s_d50000` with a CI excluding zero? `x_d50000` did not seed the labels, but it
  shares the seed's model, so a lead under about 1 degree is not evidence. This decides whether the
  latency cost of `x` still has a reason to exist on hardware that can afford it.

Anything else is unregistered. `s_d50000_cagehigh` trained on all 636 eval frames, so it no longer
bounds a leak here; scored beside `s_d50000`, it is a trained-on-the-test ceiling. Report it as
that and base nothing on it.

## Scoring

Pre-registered: conf 0.5, `rect384x640` engines, 1000-sample paired bootstrap, `s` as baseline,
all five in one `score.py` call so the bootstrap pairs them. `--labels` has four entries, and every
run must print `num_keypoints=2 num_classes=4`. A wrong count misparses the tensor and looks like a
broken engine (`score-py-labels-count-drives-numclasses`). Check the printed frame count reads 636,
which means `validation_state.json` gated it (516 and 120 on the venue runs).

`training/model_eval/score_size_ladder.sh` runs all of it: `opponent/` and `heading/` over the 636
frames, `venue_nhrl/` and `venue_massd/` from the symlink roots in
`training/data/cage_high_x50_conf044_by_venue/`, and one `rec_<name>/` per recording, each one
opponent. Output lands in `training/data/cage_high_x50_conf044/scores_size_ladder/`.

```bash
bash training/model_eval/score_size_ladder.sh <date> n s m l x
```

Secondary, unregistered, same script with small changes:

- **Conf 0.25.** `CONF=0.25` writes `scores_size_ladder_conf0.25/`. Report both. Pick the
  deployment threshold on a held-out half of the frames, not on the whole set:
  `training/data/cage_high_x50_conf044_halves/{a,b}` split each recording in two with a fixed seed
  (320 and 316 frames). Choose the conf per size on `a` by opponent F1 and report it on `b`.
- **Grid anchors.** Square engines of `s_d50000` and `x_d50000` scored beside the grid's
  `swap_half`, `d40000_s50` and `x_d40000_ep50` square engines, baseline `s_d50000`
  (`SHAPE=square EXTRA=...`). This places the basement frames against the grid's rows.
- **Trained-on-test ceiling.** `s_d50000_cagehigh` beside `s_d50000` (see above).

## What this hands the 60 Hz plan

- Its `po_m_d50000_cagehigh` arm becomes `m_d50000` from this ladder. Nothing was submitted
  under the old name, since the queue was empty on 2026-09-24.
- Its scoring section compares against `s_d50000_cagehigh` and `x_cagehigh`, and both are leaked.
  It should compare against `s_d50000` and `x_d50000` from here instead.
- Its decision rule stands as written. Q2, now read on the cage-high set, is its accuracy half,
  and its live 59 Hz check on the box is its latency half, now run on the trained `m` engine
  rather than the untrained one.
- Gate L stays failed. `l_d50000` informs the curve and does not ship.

## Next steps

1. Done 2026-09-24: prep items 1 to 3 (`097847a`). GPU 2 needed nothing: the queue's
   `yield.d/megamind` hook pauses the vLLM stack before each job and resumes it after.
2. Done 2026-09-24: jobs 33 to 37 submitted in the order above, `ARM_DATE=2026-09-24`.
3. When `m` lands, score `s` against `m` on the cage-high set, build `m`'s `aarch64_sm87` engine,
   and run the 60 Hz plan's live latency check on the box. That unblocks the 60 Hz decision.
4. When all five land, run the full scoring pass and write
   `pose_model_size_ladder_<date>.md` with the Q1 to Q4 answers and the leak delta.
