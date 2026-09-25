# Pose model size ladder: all five `yolo26` sizes on a leak-free `d50000`

This plan trains `yolo26n-pose`, `s`, `m`, `l` and `x` on one arm, with one schedule and one seed,
and scores all five on `nhrl_keypoints_eval_test` in a single paired bootstrap. It expands
[synthetic_domain_mix_2026-09-18.md](synthetic_domain_mix_2026-09-18.md), which only ever
trained `s` and `x`. Its results feed
[pose_model_ml_60hz_plan_2026-09-24.md](pose_model_ml_60hz_plan_2026-09-24.md). That plan needs an
`m`-against-`s` read to decide whether the ZED Box can run something bigger than `s` at 60 Hz, and
every arm it would compare against saw the eval fights in training.

Nothing here has run.

## Why `d50000_cagehigh` is out

Every `_cagehigh` arm carries the 636 frames of `training/data/cage_high_x50_conf044`. Those are
BrettZone `Cage-N-Overhead-High` broadcast frames of `nhrl_may26` Mrs Buff fights: clyde,
ironwarrior, sphinx, wreckcreation and the rest. `nhrl_keypoints_eval_test` is the robot's own ZED
footage of the same May 2026 fights (see `eval-recording-opponent-map`: 10-06 clyde, 11-45 sphinx,
14-12 wreckcreation, 15-35 ironwarrior). The camera is different, but the opponent, the arena, the
lighting and the moment are the same. So an arm trained on those frames has seen every eval
opponent it gets scored on.

Checked on megamind, 2026-09-24, in `training/data/domain_mix_arms_2026-09-19`:

| List | Lines | `cage_high_x50_conf044` | Other sources |
| --- | ---: | ---: | --- |
| `d50000_cagehigh.txt` | 69,083 | 636 | 18,447 corpus, 20,000 NHRL, 20,000 MassD, 10,000 basement |
| `d40000.txt` | 58,447 | 0 | 18,447 corpus, 20,000 NHRL, 20,000 MassD |
| `val.txt` | 2,049 | 0 | corpus only |

`d40000_cagehigh` and `swap_half_cagehigh` carry the same 636 frames, so they are out too. The
corpus is clean: none of `all_robot_keypoints`' 20,497 image stems matches any of the eval set's
688, and its 452 real frames are `mrs-buff-mk3-keypoints` sessions from the plywood test box.

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

Score on pathfinder's `sm89` engines, built from the rectangular ONNX:

```bash
for m in n s m l x; do
  venv/bin/python training/yolo/convert_to_tensorrt.py \
    data/models/yolo26${m}-pose_d50000_<date>_last_rect384x640.onnx
done
```

Also build the square 640 engines of `s` and `x`, so the ladder can be placed against the grid's
rows, which were all scored square.

For the ZED Box, build `m` (and `n`, if it passes Q3) from the same ONNX on the box's
TensorRT 10.3. Rename on copy to `yolo26m-pose_d50000_rect384x640_<date>_aarch64_sm87.engine`,
since a rebuild does not change the filename.

## Questions, written before anything runs

Do not move these afterwards. The noise floor is about 0.05 opponent recall: `base` spanned 0.284
to 0.367 over its own checkpoints, and `int8_quantization_2026-09-06.md` measured ~0.048
run-to-run. Every arm here is one seed, so a point difference under 0.05 is not a finding, even
when the bootstrap CI excludes zero. The bootstrap resamples frames, not training runs.

- **Q1, the size curve.** Does opponent recall rise with model size on ZED footage? The grid had
  two points and they pointed the wrong way: `x_d40000_ep50` 0.419 against `s_d40000_ep50` 0.485.
  `model_size_2026-09-04.md` found the bbox detector flat from `s` to `l`. Report recall,
  precision, F1 and heading error for all five and read the shape. No adoption rule hangs on Q1.
- **Q2, `m` for the 60 Hz plan.** This is the 60 Hz plan's accuracy criterion, unchanged:
  `m`'s opponent recall above `s` with a CI excluding zero, and our-robot heading error no worse
  than `s`. Pass: the 60 Hz plan ships `m_d50000` if its live latency check also passes. Fail:
  the ZED Box stays on `s`.
- **Q3, `n` as the cheap option.** Does `n` hold opponent recall within 0.03 of `s` and heading
  error within 1 degree? Pass: `n` becomes the fallback if the motion detector or anything else
  claims part of the 16.7 ms frame budget.
- **Q4, does `x`'s keypoint lead survive?** `d40000_x50` cut ZED keypoint error to 5.81 px against
  the best `s` arm's 7.65, and heading error to 5.28 degrees. Does `x_d50000` hold a heading lead
  over `s_d50000` with a CI excluding zero? This decides whether the latency cost of `x` still has
  a reason to exist on hardware that can afford it.

Anything else is unregistered. One unregistered read worth taking: `s_d50000` against the leaked
`s_d50000_cagehigh` on the same eval, which bounds how much the 636 frames inflated the
`_cagehigh` numbers. Report the delta and do not base any decision on it.

## Scoring

Pre-registered: conf 0.5, `rect384x640` engines, 1000-sample paired bootstrap, `s` as baseline,
all five in one `score.py` call so the bootstrap pairs them. `--labels` has four entries, and every
run must print `num_keypoints=2 num_classes=4`. A wrong count misparses the tensor and looks like a
broken engine (`score-py-labels-count-drives-numclasses`). Check the printed frame count reads 688,
which means `validation_state.json` gated it.

```bash
EVAL=training/data/nhrl_keypoints_eval_test
D=<date>
C=()
for m in n s m l x; do
  C+=(--candidate "$m=data/models/yolo26${m}-pose_d50000_${D}_last_rect384x640_x86_64_sm89.engine")
done
LABELS=mr_stabs_mk2,mrs_buff_mk3,opponent,house_bot

venv/bin/python training/model_eval/score.py $EVAL "${C[@]}" --labels $LABELS \
  --taxonomy training/model_eval/taxonomy_opponent.yaml --conf 0.5 --baseline s --bootstrap 1000 \
  --output $EVAL/scores_size_ladder/opponent
venv/bin/python training/model_eval/score.py $EVAL "${C[@]}" --labels $LABELS \
  --taxonomy training/model_eval/taxonomy_keypoint_ours.yaml --conf 0.5 --baseline s --bootstrap 1000 \
  --output $EVAL/scores_size_ladder/heading
```

Then the same opponent run per venue and per recording, as `score_domain_mix.sh` does:
`nhrl_keypoints_eval_test_by_venue/{nhrl_may,massd_aug}` and each recording directory. The by-venue
symlink roots are not on pathfinder, so copy them from megamind or rebuild them first.
`score_domain_mix.sh` itself cannot run this ladder, since it takes one model and square engines,
so either call `score.py` directly as above or give the script a size mode. Per-recording results
matter here because each recording is one opponent, and pooled recall hid `16-18-05` moving
against every other recording in the grid.

Secondary, unregistered, same call with small changes:

- **Conf 0.25.** The grid found most ZED arms score better at 0.25 than 0.5. Report both. Pick
  the deployment threshold on a held-out half of the frames, not on the whole set.
- **Grid anchors.** Square engines of `s_d50000` and `x_d50000` scored beside the grid's
  `swap_half`, `d40000_s50` and `x_d40000_ep50` square engines, baseline `s_d50000`. This places
  the basement frames against the grid's rows, and it is the only direct test the basement render
  gets on ZED footage.

## What this hands the 60 Hz plan

- Its `po_m_d50000_cagehigh` arm becomes `m_d50000` from this ladder. Nothing was submitted
  under the old name, since the queue was empty on 2026-09-24.
- Its scoring section compares against `s_d50000_cagehigh` and `x_cagehigh`, and both are leaked.
  It should compare against `s_d50000` and `x_d50000` from here instead.
- Its decision rule stands as written. Q2 is its accuracy half, and its live 59 Hz check on the
  box is its latency half, now run on the trained `m` engine rather than the untrained one.
- Gate L stays failed. `l_d50000` informs the curve and does not ship.

## Next steps

1. Prep items 1 to 3: arm, leak check, `train.py` presets. Commit them with the rebuilt manifest.
2. Clear GPU 2 with the vLLM owner, then submit the five jobs in the order above.
3. When `m` lands, build its `sm89` and `aarch64_sm87` engines, score `s` against `m`, and run the
   60 Hz plan's live latency check on the box. That unblocks the 60 Hz decision.
4. When all five land, run the full scoring pass and write
   `pose_model_size_ladder_<date>.md` with the Q1 to Q4 answers and the leak delta.
