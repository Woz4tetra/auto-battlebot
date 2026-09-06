# Does model size matter, and is the latency worth it? - 2026-09-04

Five yolo26 detect sizes (n/s/m/l/x) trained 100 epochs on `nhrl_robots_bbox_2class`
(25,914 train / 6,573 val, scene-disjoint, zero synthetic), batch 96, then scored on the
expanded `nhrl_keypoints_eval_test` (688 frames, 8 scenes) with `score.py`,
`--labels "opponent,house_bot"`, `taxonomy_merged.yaml`, conf 0.5, paired bootstrap
1000x, baseline `n`. Latency measured on the dev A6000 and then live on the Jetson for
`n`, `s`, and `x`.

Predecessor: `data_scaling_2026-07-27.md` (corpus floor), `synthetic_arms_2026-07-31.md`
(the `mixed` arm currently deployed).

## Headline

1. **Model size matters, and more than expected.** Every arm beats `yolo26n` on agnostic
   recall by a significant margin: `s` +0.059, `m` +0.057, `l` +0.057, `x` +0.088
   (all `better`, 95% CI excludes 0).
2. **`yolo26s` is the efficient point.** +0.059 recall and +0.033 precision over `n` for
   1.21x the inference time. It captures two thirds of `x`'s recall gain for a fifth of
   its added latency.
3. **`m` and `l` are dominated by `s`.** They match its recall (0.837, 0.838 vs 0.839) but
   their precision gain over `n` is `ns` where `s` is `better`, and they cost 1.4x and
   1.7x `s`'s inference time. There is no configuration in which either is the right
   choice on this corpus.
4. **`yolo26x` is the accuracy winner and is unaffordable.** +0.088 recall and +0.052
   precision, but on the Jetson it puts `runner.tick` at 58.22 ms against a 33.3 ms frame
   period. The loop drops to 17.0 Hz and end-to-end latency to 152.8 ms mean.
5. **`s` costs no tick time on the Jetson.** `runner.tick` is 32.67 ms for `n` and 32.72
   ms for `s`. The extra 3.72 ms of perception work comes out of the block in
   `runner.camera.get`, which falls from 17.52 ms to 13.32 ms. End-to-end still rises
   +6.8 ms mean, so the work is not free, it just does not cost frame rate.
6. **Val and eval disagree about *where* the gain is, not whether there is one.** On val,
   recall saturates at `m` and never recovers. On eval, recall saturates from `s` through
   `l` and then jumps again at `x`. Ranking arms on val would have picked `m`, which the
   eval shows is a strictly worse choice than `s`.
7. **The 100-epoch stopping point still holds.** Every arm peaked at epoch 96-100 on val;
   none was overfitting within the schedule.

## Setup

| | |
|---|---|
| Corpus | `training/data/nhrl_robots_bbox_2class` - 25,914 train / 6,573 val, 71 scenes, scene-disjoint, zero synthetic |
| Classes | `robot`, `house_bot` |
| Arms | `yolo26n` 2.50 M params / `s` 9.95 M / `m` 21.78 M / `l` 26.18 M / `x` 58.81 M |
| Schedule | 100 epochs, batch 96, imgsz 640, `--save-period 25`, seed 0, single seed |
| Effective weight decay | 0.00075 (`trainer.py` scales the declared 0.0005 by `batch/nbs`) |
| Eval | `nhrl_keypoints_eval_test`, 688 frames / 8 scenes, conf 0.5, `taxonomy_merged.yaml`, bootstrap 1000x, baseline `n` |
| Hardware | megamind, 3x RTX A6000 sm86, `NCCL_P2P_DISABLE=1` |
| Run dirs | `runs/projects/auto_battlebots_2026-09-04_00-56-05_yolo26{n,s,m,l,x}` |

```bash
NCCL_P2P_DISABLE=1 venv/bin/python training/yolo/train.py \
  training/data/nhrl_robots_bbox_2class/data.yml yolo26n yolo26s yolo26m yolo26l yolo26x \
  -e 100 -b 96 -d 0 1 2 --save-period 25
```

19.50 h total on 3 GPUs: n 1.83 h, s 2.34 h, m 3.68 h, l 4.72 h, x 6.94 h.

## Results - eval set, agnostic level

"Did it find a robot", on the deployment camera. This is the level the decision rule uses.

| arm | precision | recall | F1 | mAP50 | mAP50-95 |
|---|---:|---:|---:|---:|---:|
| n | 0.858 | 0.780 | 0.817 | 0.754 | 0.481 |
| s | 0.891 | **0.839** | 0.864 | 0.815 | 0.563 |
| m | 0.858 | 0.837 | 0.847 | 0.809 | 0.586 |
| l | 0.865 | 0.838 | 0.851 | 0.808 | 0.595 |
| x | **0.910** | **0.868** | **0.889** | **0.850** | **0.630** |

### Paired bootstrap vs `n`

| arm | metric | n | arm | delta | 95% CI | verdict |
|---|---|---:|---:|---:|---|---|
| s | recall | 0.780 | 0.839 | +0.059 | [+0.045, +0.073] | better |
| s | precision | 0.858 | 0.891 | +0.033 | [+0.020, +0.046] | better |
| m | recall | 0.780 | 0.837 | +0.057 | [+0.041, +0.073] | better |
| m | precision | 0.858 | 0.858 | -0.000 | [-0.012, +0.012] | **ns** |
| l | recall | 0.780 | 0.838 | +0.057 | [+0.042, +0.074] | better |
| l | precision | 0.858 | 0.865 | +0.007 | [-0.005, +0.020] | **ns** |
| x | recall | 0.780 | 0.868 | +0.088 | [+0.071, +0.107] | better |
| x | precision | 0.858 | 0.910 | +0.052 | [+0.035, +0.071] | better |

The `ns` precision verdicts for `m` and `l` are the reason claim 3 holds: `s` gets a real
precision gain, the two models 2-2.6x its size do not.

## Results - eval set, archetype level

| arm | precision | recall | F1 | wrong_class_rate | AP50-95 opponent | AP50-95 house_bot |
|---|---:|---:|---:|---:|---:|---:|
| n | 0.852 | 0.775 | 0.812 | 0.007 | 0.413 | 0.676 |
| s | 0.890 | 0.839 | 0.864 | 0.001 | 0.500 | 0.762 |
| m | 0.857 | 0.836 | 0.846 | 0.001 | 0.520 | 0.784 |
| l | 0.865 | 0.837 | 0.851 | 0.001 | 0.528 | 0.793 |
| x | 0.910 | 0.868 | 0.889 | 0.000 | 0.568 | 0.821 |

Opponent AP50-95 rises monotonically (0.413 -> 0.568) even where recall is flat, so the
larger models keep improving box quality on robots the smaller ones already found. Every
arm from `s` up clears the ~0.21 real-trained ceiling in `experiment_runbook.md` by a wide
margin, which is a corpus effect, not a size effect - `n` clears it too.

## Results - val, and why it misleads

| arm | val mAP50-95 | val recall | eval recall |
|---|---:|---:|---:|
| n | 0.6495 | 0.7709 | 0.780 |
| s | 0.7039 | 0.8252 | 0.839 |
| m | 0.7666 | 0.8457 | 0.837 |
| l | 0.7636 | 0.8313 | 0.838 |
| x | 0.7939 | 0.8427 | 0.868 |

Val recall peaks at `m` and never beats it. Eval recall is flat from `s` to `l` and then
jumps at `x`. Picking on val gives `m`; picking on eval gives `s` or `x`, and `m` is the
one arm both agree is not worth its cost. This is the same val/eval divergence
`category_addition_2026-07-25.md` recorded, in a new form: here val does not just
mis-scale the gain, it mis-orders the arms.

## Latency - dev box

`benchmark_engines.py`, 300 iterations after 50 warmup, one real 1280x720 eval frame,
A6000 sm86, FP16 engines. `gpu` is H2D + `execute_async_v3` + D2H + sync; `total` adds
letterbox preprocess and NMS, and is the number that maps onto the C++ `update()` call.

| arm | gpu median | total median | total p90 | total vs n |
|---|---:|---:|---:|---:|
| n | 1.211 ms | 2.242 ms | 2.382 ms | 1.00x |
| s | 1.524 ms | 2.703 ms | 2.990 ms | 1.21x |
| m | 2.297 ms | 3.766 ms | 4.016 ms | 1.68x |
| l | 2.884 ms | 4.473 ms | 4.634 ms | 2.00x |
| x | 4.267 ms | 5.758 ms | 5.999 ms | 2.57x |

**These are not Jetson numbers and must not be treated as such.** `yolo26n` runs ~1.2 ms
here and ~9.5-11 ms inside the Jetson pipeline, and that gap is not a constant across
sizes. The ordering transfers; the magnitudes do not.

## Latency - Jetson

Three live runs on the Jetson with `mr_stabs_mk2`, roughly two minutes each on the same
scene, same keypoint engine (`yolo26n-pose_our_robots_2026-05-01`), swapping only
`[robot_mask_model.engine]`. Reported by `scripts/mcap_latency_report.py` over the window
after field init. `m` and `l` were not run: the eval already dominated them with `s`.

| arm | tick mean | tick p95 | batch mean | bbox inference mean | `camera.get` mean | loop rate | e2e mean | e2e p95 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| n (deployed `mixed`) | 32.67 ms | 37.34 ms | 14.27 ms | 7.95 ms | 17.52 ms | 30.4 Hz | 64.4 ms | 69.7 ms |
| s | 32.72 ms | 37.55 ms | 17.99 ms | 11.98 ms | 13.32 ms | 30.3 Hz | 71.2 ms | 74.9 ms |
| x | 58.22 ms | 62.97 ms | 56.63 ms | 49.96 ms | 0.11 ms | 17.0 Hz | 152.8 ms | 177.5 ms |

Decision rule (b) - the arm must keep `runner.tick` under 33.3 ms - passes for `s` at
32.72 ms and fails for `x` at 58.22 ms.

`s` adds 3.72 ms to the perception batch and 0.05 ms to the tick. The loop is frame-period
bound, so the added work is absorbed by the block in `runner.camera.get`. That block is
the headroom, and `s` spends about a quarter of it. End-to-end latency does move, +6.8 ms
mean and +5.2 ms p95, because `pipeline.latency` measures capture to command send and does
not care where in the tick the time went.

`x` behaves the way `parallel_yolo_batch/comparison.md` predicted for crossing the frame
period. `camera.get` collapses to 0.11 ms because a frame is always already waiting, the
loop runs at 17.0 Hz, and end-to-end lands at 152.8 ms mean / 177.5 ms p95, 2.4x the `n`
baseline.

The Jetson magnitudes are nothing like the A6000 table. Dev box `n` -> `s` is +0.31 ms of
GPU time and `n` -> `x` is +3.06 ms; the Jetson gives +4.03 ms and +42.01 ms on the same
inference stage. Cross-arm ordering transferred and the per-arm cost did not.

The keypoint model was identical across all three runs, but its measured inference time
rose with the detector: 8.15 ms, 11.72 ms, 32.81 ms. The two models share the GPU in the
parallel batch, so a heavier detector slows the co-running keypoint pass. Budget a size
change against the whole batch, not against the detector row alone.

Per-run tables and plots are in `assets/2026-09-04_model_size/`:

- `n`: `auto_battlebot_mr_stabs_mk2_jetson_2026-09-05_11-13-33_latency.{md,png}`
- `s`: `auto_battlebot_mr_stabs_mk2_jetson_2026-09-05_12-00-59_latency.{md,png}`
- `x`: `auto_battlebot_mr_stabs_mk2_jetson_2026-09-05_12-14-44_latency.{md,png}`

## Answers

### Does model size matter for my application? - **strong**

Yes. Going from `yolo26n` to any larger arm buys 0.057-0.088 agnostic recall on the
deployment camera, every one significant against a 1000x paired bootstrap. This is a
larger effect than anything the data experiments produced: `data_scaling_2026-07-27`
found dropping 25% of the corpus was within noise, and `synthetic_arms_2026-07-31` found
adding synthetic data moved recall by less than this. On a fixed 71-scene corpus,
capacity was the binding constraint, not data volume.

The shape matters as much as the size. The gain is not monotonic in parameters: `s`
(9.95 M) and `l` (26.18 M) reach the same recall, and only `x` (58.81 M) breaks past it.

### Is the latency trade-off worth it? - **strong**, for `s` and only `s`

Yes for `s`. It holds `runner.tick` at 32.72 ms inside the 33.3 ms frame period, keeps the
loop at 30.3 Hz, and buys +0.059 agnostic recall for +6.8 ms of end-to-end latency.

No for `x`. It puts the tick at 58.22 ms, the loop at 17.0 Hz, and end-to-end at 152.8 ms
mean. +0.088 recall does not buy 88 ms.

The `parallel_yolo_batch/comparison.md` prediction held in shape: crossing the frame period
is a cliff, not a slope. It was wrong about where the headroom is. That report put it at
~1 ms of tick slack, which would have ruled `s` out. The real slack is the 17.52 ms the
loop spends blocked in `camera.get`, and `s` spends 4 ms of it.

What `s` does not fix is the 60 ms end-to-end budget, which `n` already misses at 69.7 ms
p95. That is a pipeline problem, not a model problem, and the candidates are unchanged:
move `publish_camera_data` after the command send (~10 ms) and merge the two YOLOs into
one multi-head engine.

### When do I stop training YOLO? - **moderate**, unchanged

Every arm peaked on val at epoch 96-100 with none still climbing steeply, so the existing
~100-epoch answer holds across a 24x parameter range. The `--save-period 25` ladders are
retained if an earlier checkpoint ever needs scoring.

## Caveats

- **Single seed.** One run per arm. `data_epoch_min` measured run-to-run recall spread
  around 0.048 at single seed, which exceeds the `s` -> `m` -> `l` differences entirely.
  The claim that `m` and `l` tie `s` is therefore a claim that they are *indistinguishable*,
  not that they are equal. The `n` -> `s` (+0.059) and `n` -> `x` (+0.088) gaps clear that
  bar; nothing else in the table does.
- **Jetson latency covers three arms, one run each.** `n`, `s`, `x` only, two minutes
  per arm, one scene. Enough to separate 32.7 ms from 58.2 ms; not enough to resolve the
  0.05 ms between `n` and `s`, which should be read as "no tick cost", not as a measured
  difference.
- **The Jetson `n` is the deployed `mixed` engine**, `yolo26n_..._mixed_2026-07-31`, not
  the sweep's real-only `n`. Same architecture, so the latency comparison holds; the
  recall numbers in the tables above do not come from that engine.
- **The `x` run re-initialized the field at 7.0 s**, after the report window opened at
  4.6 s, so its table carries one-shot `point_cloud_field_filter.*` rows and its 488 ms
  `pipeline.latency` max is that re-init. The mean and p95 over 2,335 ticks are unaffected.
- **Batch 96, not the deployed 128.** Effective weight decay is 0.00075 here against
  0.0010 for the deployed `yolo26n`, so the `n` arm is not a reproduction of the deployed
  model. It is an internally consistent baseline for this sweep only.
- **The eval set expanded.** 688 frames / 8 scenes, up from 372 / 5, including a
  2026-08-29 Jetson scene. Numbers here do not compare to eval numbers in any report
  before this one. `n` scores 0.780 recall here against 0.864 for the 2026-07-31
  `real_only` arm on the old eval set - that is the eval change plus the batch change, not
  a regression.
- **Engines are sm86**, built and scored on megamind rather than the sm89 dev box the
  runbook assumes. Fine for cross-arm comparison, irrelevant to the Jetson.
- **A6000 latency understates CPU-bound cost.** On the Jetson the per-model update is
  ~9.5-11 ms against 1-3 ms of GPU time, so the fixed CPU share is much larger there and
  the relative penalty for a big model is *smaller* than the `total vs n` column suggests.
  This cuts in favor of the larger arms and is another reason to measure rather than
  extrapolate.

## Recommendation

- **Do not deploy `m` or `l`.** They are dominated by `s` on the eval set at every level.
- **Deploy `yolo26s`.** On the Jetson it costs no tick time and no frame rate, and it is
  the only arm besides `n` that stays inside the frame period.
- **Do not deploy `x` on the current pipeline.** Measured, not extrapolated: 17.0 Hz and
  152.8 ms end-to-end. Revisit only after buying back tick time elsewhere. The named
  candidates in `parallel_yolo_batch/comparison.md` are moving `publish_camera_data` after
  the command send (~10 ms) and merging the two YOLOs into one multi-head engine.
- **Fix the 60 ms end-to-end budget separately.** `n` misses it at 69.7 ms p95 today, so
  the budget is not a reason to hold `s` back.
- **Stop tuning the detector on val.** It mis-ordered the arms here. Score on
  `nhrl_keypoints_eval_test` before drawing any conclusion about a model change.
- **Revisit `x` if the corpus grows.** Capacity is currently the binding constraint, so
  more scenes and a bigger model are complementary, not alternatives.

## How the Jetson numbers were produced

The engines must be built on the Jetson (`aarch64_sm87`).

```bash
# dev box: ship the ONNX files (deploy_to_jetson.sh rsyncs data/models/ separately)
./scripts/deploy_to_jetson.sh

# Jetson: build engines, then pin clocks before timing
cd ~/auto-battlebot
venv/bin/python training/yolo/convert_to_tensorrt.py \
  data/models/yolo26{n,s,m,l,x}_nhrl_robots_bbox_2class_2026-09-04.onnx --workspace 2
sudo jetson_clocks && sudo nvpmodel -q

# raw GPU time (JetPack ships trtexec; megamind's pip TensorRT does not)
for m in n s m l x; do
  /usr/src/tensorrt/bin/trtexec \
    --loadEngine=data/models/yolo26${m}_nhrl_robots_bbox_2class_2026-09-04_aarch64_sm87.engine \
    --noDataTransfers --iterations=300 --warmUp=500 --avgRuns=100
done

# full path, same script and same units as the dev-box table above
venv/bin/python training/model_eval/benchmark_engines.py \
  --candidate n=data/models/yolo26n_..._aarch64_sm87.engine \
  --candidate s=data/models/yolo26s_..._aarch64_sm87.engine \
  --frame <an eval frame> --iterations 300
```

Swap the arm into `config/_jetson.toml` `[robot_mask_model.engine] candidates` (a
one-line change - the output layout is `[1, 6, 8400]` for every arm) and run live with
`[mcap] enable = true`:

```bash
venv/bin/python scripts/mcap_latency_report.py data/recordings/<run>.mcap --csv
```

Read `runner.tick` mean against 33.3 ms and `pipeline.latency` p95 against the 60 ms
budget. The report trims everything up to field init by default, which is what makes it
comparable to the reports in `docs/experiments/parallel_yolo_batch/jetson/`; pass
`--include-field-init` to see the startup ticks instead.

## Artifacts

- Scores: `training/data/nhrl_keypoints_eval_test/scores_model_size/{summary.csv, significance.csv, headline.png, confusion_*.png}`
- Runs: `runs/projects/auto_battlebots_2026-09-04_00-56-05_yolo26{n,s,m,l,x}/` (`results.csv`, `weights/{best,last,epoch0,epoch25,epoch50,epoch75}.pt`)
- Models: `data/models/yolo26{n,s,m,l,x}_nhrl_robots_bbox_2class_2026-09-04.{pt,onnx,_x86_64_sm86.engine}`
- Benchmark tool: `training/model_eval/benchmark_engines.py`
- Jetson latency runs: `assets/2026-09-04_model_size/auto_battlebot_mr_stabs_mk2_jetson_2026-09-05_{11-13-33,12-00-59,12-14-44}_latency.{md,png}` (`n`, `s`, `x`)
- Jetson recordings: `data/recordings/auto_battlebot_mr_stabs_mk2_jetson_2026-09-05_{11-13-33,12-00-59,12-14-44}.mcap`

## Addendum 2026-09-05 - does `yolo26s` still gain from synthetic data?

The sweep above trained on `nhrl_robots_bbox_2class` (real only). The deployed `yolo26n` is
the `mixed` arm from `synthetic_arms_2026-07-31.md`, real plus 17,995 synthetic renders. This
addendum trains `yolo26s` on that same `mixed_2class` corpus with the sweep recipe, to check
whether the candidate upgrade should be trained the way the deployed model is.

Same recipe as the sweep: 100 epochs, batch 96 (effective weight decay 0.00075), imgsz 640,
seed 0, 3x A6000 DDP, `--save-period 25`. Corpus: 43,909 train (25,914 real + 17,995
synthetic), the same 6,573-frame real scene-disjoint val. The synthetic frames contain
`robot` boxes only, no `house_bot`. Wall clock 3.91 h against 2.34 h for real-only `s`.
Scored with the same `score.py` invocation, same 688 eval frames, baseline `s` (real-only).

```bash
NCCL_P2P_DISABLE=1 venv/bin/python training/yolo/train.py \
  training/data/mixed_2class/data.yml yolo26s -e 100 -b 96 -d 0 1 2 --save-period 25
```

### Headline

1. **At the full schedule, `mixed` trades recall for precision and gains nothing net.**
   `s_mixed` (best.pt, epoch 97) scores +0.049 precision (`better`) and -0.048 recall
   (`worse`) against real-only `s`; F1 delta is -0.005, `ns`. The 2026-07-31 `yolo26n`
   result had the same shape (+0.049 precision, -0.017 recall), but here the recall loss
   clears significance.
2. **Step-matched, `mixed` is a wash.** Real `s` at 100 epochs is 2.59 M frame-presentations.
   `mixed` at epoch 75 (3.29 M) is `ns` on every metric at every level. At epoch 50
   (2.20 M) it is +0.025 recall (`better`) and -0.020 precision (`worse`), F1 `ns`.
3. **Recall falls monotonically along the `mixed` schedule** on the eval set: 0.864 at
   epoch 50, 0.842 at 75, 0.791 at 100, while precision rises 0.871 -> 0.904 -> 0.939.
   Val picks epoch 97 as best. This is the late external-eval decline from
   `category_addition_2026-07-25.md`, and val misses it again.
4. **Synthetic data hurts `house_bot`.** Archetype AP50-95 on `house_bot` drops from 0.762
   (`s`) to 0.684 (`s_mixed`). The renders contain no house bots, so house-bot frames are
   59% of the per-epoch share they hold in the real corpus. Opponent AP is flat (0.500 vs
   0.494).
5. **The sweep's `s` recommendation stands.** Nothing here beats real-only `s` on F1 at any
   checkpoint with significance. Train the candidate upgrade on the real corpus.

### Results - eval set, agnostic level

| arm | frame-presentations | precision | recall | F1 | mAP50 | mAP50-95 |
|---|---:|---:|---:|---:|---:|---:|
| n (real) | 2.59 M | 0.858 | 0.780 | 0.817 | 0.754 | 0.481 |
| s (real) | 2.59 M | 0.891 | 0.839 | 0.864 | 0.815 | 0.563 |
| s_mixed @ep50 | 2.20 M | 0.871 | **0.864** | **0.867** | **0.847** | **0.574** |
| s_mixed @ep75 | 3.29 M | 0.904 | 0.842 | 0.872 | 0.830 | 0.569 |
| s_mixed (best, ep97) | 4.26 M | **0.939** | 0.791 | 0.859 | 0.784 | 0.538 |

### Paired bootstrap vs real-only `s`

| arm | metric | s | arm | delta | 95% CI | verdict |
|---|---|---:|---:|---:|---|---|
| s_mixed | recall | 0.839 | 0.791 | -0.048 | [-0.062, -0.034] | **worse** |
| s_mixed | precision | 0.891 | 0.939 | +0.049 | [+0.032, +0.065] | better |
| s_mixed | F1 | 0.864 | 0.859 | -0.005 | [-0.018, +0.007] | ns |
| s_mixed @ep50 | recall | 0.839 | 0.864 | +0.025 | [+0.011, +0.039] | better |
| s_mixed @ep50 | precision | 0.891 | 0.871 | -0.020 | [-0.034, -0.006] | **worse** |
| s_mixed @ep50 | F1 | 0.864 | 0.867 | +0.003 | [-0.008, +0.015] | ns |
| s_mixed @ep75 | recall | 0.839 | 0.842 | +0.003 | [-0.009, +0.015] | ns |
| s_mixed @ep75 | precision | 0.891 | 0.904 | +0.013 | [-0.001, +0.026] | ns |
| s_mixed @ep75 | F1 | 0.864 | 0.872 | +0.007 | [-0.004, +0.018] | ns |

### Results - eval set, archetype level

| arm | precision | recall | F1 | wrong_class_rate | AP50-95 opponent | AP50-95 house_bot |
|---|---:|---:|---:|---:|---:|---:|
| s (real) | 0.890 | 0.839 | 0.864 | 0.001 | 0.500 | 0.762 |
| s_mixed @ep50 | 0.868 | 0.860 | 0.864 | 0.004 | 0.519 | 0.740 |
| s_mixed @ep75 | 0.901 | 0.839 | 0.869 | 0.003 | 0.517 | 0.733 |
| s_mixed (best) | 0.936 | 0.789 | 0.856 | 0.003 | 0.494 | 0.684 |

### Val

| arm | best val mAP50-95 | at epoch | val recall |
|---|---:|---:|---:|
| s (real) | 0.7039 | 96-100 | 0.8252 |
| s_mixed | 0.7124 | 97 | 0.8203 |

Val prefers `mixed` by 0.0085 mAP50-95 and prefers its epoch-97 checkpoint over epoch 50
(0.7000) and 75 (0.7001). On the eval set epoch 97 is the worst of the three on recall by
0.05-0.07. Same verdict as the size sweep: do not rank on val.

### Reading the ep50 result

`s_mixed @ep50` recall (0.864) is within 0.004 of `x` (0.868) at `s` latency, and its mAP50
(0.847) is the best of any `s` checkpoint. It is tempting to call it the winner. Three
reasons not to:

- It is one of three checkpoints from a ladder, picked after seeing the eval. The registered
  comparison in `synthetic_arms_plan.md` was step-matched, which points at ep75, and ep75 is
  `ns` everywhere.
- Its precision is significantly *worse* than `s` (-0.020), and F1 is `ns`. It moves along
  the precision-recall trade-off, it does not shift the frontier.
- Single seed. `data_epoch_min` put run-to-run recall spread near 0.048. The +0.025 recall
  clears the bootstrap CI but not that bar.

If a recall-heavy operating point is wanted, lowering `conf` on real-only `s` is the cheaper
experiment and does not cost 1.7x the training time.

### Caveats

- Single seed, single arm. The `n` result on 2026-07-31 and this `s` result agree on the
  direction (precision up, recall down at full schedule), which is two seeds' worth of
  evidence for the shape, not for the magnitude.
- The step confound cuts the other way here. On 2026-07-31 `mixed @ep50` beat `real_only` on
  precision and F1; here it loses precision and ties F1. The two runs differ in model size,
  batch (128 vs 96), and eval set (372 vs 688 frames), so this is not a contradiction that
  can be attributed to any one cause.
- Latency is unchanged. Same architecture as the sweep's `s`, so both latency tables
  above apply as-is, Jetson included.

### Artifacts

- Scores: `training/data/nhrl_keypoints_eval_test/scores_model_size_mixed/{summary.csv, significance.csv, headline.png, confusion_*.png}` (`n`, `s`, `s_mixed`, `s_mixed_ep50`, `s_mixed_ep75`)
- Run: `runs/projects/auto_battlebots_2026-09-05_00-17-01_yolo26s/` (`results.csv`, `weights/{best,last,epoch0,epoch25,epoch50,epoch75}.pt`)
- Log: `training/projects/model_size_mixed_logs/yolo26s_mixed.log`, scoring script `post_train.sh` alongside
- Models: `data/models/yolo26s_mixed_2class_2026-09-05{,_ep50,_ep75}.{pt,onnx,_x86_64_sm86.engine}`
