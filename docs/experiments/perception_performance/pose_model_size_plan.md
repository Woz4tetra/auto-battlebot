# Does model size matter for the keypoint model?

Companion to `model_size_2026-09-04.md`, which answered the question for the bounding-box
detector. That sweep found capacity was the binding constraint on a fixed corpus: every
arm above `yolo26n` gained significant eval recall, and `yolo26s` captured most of it at
1.21x the inference time. The keypoint model has never been sized at all - every pose
report uses `yolo26n-pose`.

The keypoint model matters differently. It feeds aim assist, so the metric that decides it
is **heading error**, not box recall. `experiment_runbook.md` records 9.0 deg as the good
dedicated-model figure and 38.5 deg as unusable. A box-recall win that leaves heading
unchanged buys nothing.

## Question

Does a larger pose backbone reduce keypoint heading error enough to justify its latency,
given the keypoint model is the *other* half of the parallel perception batch and competes
for the same ~1 ms of Jetson headroom?

## Dataset - `all_robot_keypoints`

| | |
|---|---|
| Path | `training/data/all_robot_keypoints` |
| Size | 18,447 train / 2,049 val |
| Classes | `mr_stabs_mk2` (10,814), `mrs_buff_mk3` (16,447), `nhrl_robot` (36,108) |
| Keypoints | `kpt_shape: [2, 3]`, `flip_idx: [0, 1]` - front and back |

Class balance is skewed 3.3:1 toward `nhrl_robot`. That is fine here (it is the same for
every arm) but it means per-class keypoint metrics on `mr_stabs_mk2` rest on the fewest
instances and will be the noisiest column.

**Check before training:** this dataset has no README and its split provenance is
unrecorded. Confirm the val split is scene-disjoint the way `nhrl_robots_bbox_2class` is.
If it was split randomly, val is near-duplicate frames and every val number is
meaningless - the bbox corpus had exactly this defect before 2026-07-29. Grading is on the
external eval set either way, so a bad val split degrades monitoring, not the verdict.

## Arms

| arm | model | role |
|---|---|---|
| A | `yolo26n-pose` | baseline, matches the deployed keypoint model family |
| B | `yolo26s-pose` | the bbox sweep's efficient point, tested on pose |
| C | `yolo26x-pose` | ceiling |

`m` and `l` are deliberately skipped: the bbox sweep found both dominated by `s` - equal
recall, no significant precision gain, 1.4-1.7x the cost. Re-testing them here would cost
~9 h to re-derive a result we already have. If `s` and `x` bracket the pose answer
differently than they did the bbox answer, add them then.

`train.py` already carries `yolo26n-pose` and `yolo26x-pose`; **`yolo26s-pose` needs
adding**.

## Design

| | |
|---|---|
| Epochs | 200, `--save-period 50` |
| Batch | 96 constant (`-b 96`, 32/GPU across 3 GPUs) |
| imgsz | 640 |
| Devices | `-d 0 1 2`, submitted through `training/gpu_queue.py` (see "Running the arms") |
| Seed | 0, single seed |

**Why 200 epochs, not the bbox sweep's 100.** The existing pose configs use 500 where the
detect config uses 100, and `experiment_runbook.md` notes box and pose metrics plateau at
different epochs. 200 with a save-period-50 ladder covers the range without committing to
500. Score the baseline's ep100/150/200 checkpoints first to locate the pose plateau, then
use the same endpoint for all three arms. Do not pick a different endpoint per arm on eval
metrics - that is selection on the test set.

**Why batch 96.** Same reasoning as the bbox sweep: it divides evenly across 3 GPUs, and
`yolo26x` was measured at 32.4 GB/GPU at 32 img/GPU with a detect head. The pose head adds
little. Note batch scales weight decay (`trainer.py`: `wd * batch * accumulate / nbs`), so
96 gives an effective 0.00075.

## Running the arms

Several agents share the three GPUs and each arm takes all of them, so submit through
`training/gpu_queue.py` instead of running `train.py` directly. The queue sets
`NCCL_P2P_DISABLE=1` for multi-device jobs - it stays mandatory, it is just no longer
something to remember.

Queue arm A first. The val-split check above and the ep100/150/200 plateau scoring both
gate on it, and at ~2.6 h it is the cheapest way to find a corpus problem before spending
`x`'s ~9.9 h. B needs `yolo26s-pose` added to `train.py` first.

```bash
Q="venv/bin/python training/gpu_queue.py"
D="training/data/all_robot_keypoints/data.yml"   # the yaml, not the directory

$Q submit --name A_n_pose --by <agent> -- \
  venv/bin/python training/yolo/train.py $D yolo26n-pose -d 0 1 2 -b 96 -e 200 --save-period 50
$Q submit --name B_s_pose --by <agent> -- \
  venv/bin/python training/yolo/train.py $D yolo26s-pose -d 0 1 2 -b 96 -e 200 --save-period 50
$Q submit --name C_x_pose --by <agent> -- \
  venv/bin/python training/yolo/train.py $D yolo26x-pose -d 0 1 2 -b 96 -e 200 --save-period 50

$Q status
$Q logs <id> --tail 40
```

At ~16 h these three arms hold the box for most of a day, so check `$Q status` before
queueing and give a short scoring or export job `--priority 1` rather than waiting behind
`x`. The val-split check needs no GPU and should not be queued at all.

## Decision rule - register before looking

Adopt a larger pose model only if **both** hold:

- (a) `kp_heading_err_deg` on the eval set improves against `yolo26n-pose` by a margin
  whose paired-bootstrap 95% CI excludes 0;
- (b) its measured Jetson time keeps `runner.perception_batch.update` from pushing
  `runner.tick` over the 33.3 ms camera frame period.

Heading error is primary. `kp_pck@0.1` and `kp_err_px` are secondary and reported but do
not decide. Box recall on our robots is tertiary - the bbox model already handles finding
robots.

Criterion (b) is tighter here than in the bbox sweep. The two models run **concurrently**
in `ParallelModelBatch`, so batch time is roughly the slower of the two: on the Jetson,
keypoint 7.33 ms and blob 7.05 ms produce a 12.86 ms batch. Growing the keypoint model
raises the batch immediately, because it is already the slower branch.

## Scoring

```bash
venv/bin/python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate n=data/models/yolo26n-pose_all_robot_keypoints_<date>_x86_64_sm86.engine \
  --candidate s=data/models/yolo26s-pose_all_robot_keypoints_<date>_x86_64_sm86.engine \
  --candidate x=data/models/yolo26x-pose_all_robot_keypoints_<date>_x86_64_sm86.engine \
  --labels "mr_stabs_mk2,mrs_buff_mk3,nhrl_robot" \
  --taxonomy training/model_eval/taxonomy_keypoint.yaml \
  --conf 0.5 --baseline n --bootstrap 1000 \
  --output training/data/nhrl_keypoints_eval_test/scores_pose_size
```

- **`--labels` must have exactly 3 entries** to match `nc: 3`. A wrong count misparses the
  tensor to `num_keypoints=0` and returns ~0 recall while looking like a broken engine -
  this bit the first `our_robots` run (`deploy_keypoints_2026-07-16.md`). Check the printed
  `num_keypoints=N num_classes=M` line.
- `taxonomy_keypoint.yaml` excludes `house_bot` and `object`, so metrics reflect our robots
  only.
- Report keypoint metrics **at several confidences (0.05 / 0.3 / 0.5 / 0.6)**, following
  `all_robots_pose_2026-07-14.md`. That report found the ranking between two pose models
  flipped on heading between conf 0.5 and 0.6, purely because the weaker model discarded
  three quarters of its detections. A single-confidence table can invert the conclusion.

## Latency

```bash
venv/bin/python training/model_eval/benchmark_engines.py \
  --candidate n=... --candidate s=... --candidate x=... \
  --frame <an eval frame> --iterations 300
```

Dev box gives ordering only. The Jetson steps are the same as
`model_size_2026-09-04.md` "What is still missing": build `aarch64_sm87` engines on the
Orin, `sudo jetson_clocks`, `trtexec` plus `benchmark_engines.py`, then swap into
`config/_jetson.toml` `[keypoint_model.engine] candidates` and read
`mcap_latency_report.py --after-field-init`.

## Cost

Scaling the measured bbox arm times by the corpus ratio (18,447 / 25,914 = 0.71) and by
2x for 200 epochs: **n ~2.6 h, s ~3.3 h, x ~9.9 h, about 16 h total** on 3 GPUs. Engine
builds add ~10 min. If that is too long, cutting to 100 epochs halves it, at the cost of
possibly stopping a pose model before it plateaus.

## Risks / caveats

- **Single seed.** `data_epoch_min` measured ~0.048 run-to-run recall spread. The
  equivalent spread for heading error is unmeasured, which makes small heading deltas hard
  to call. If `s` beats `n` by less than a couple of degrees, the honest verdict is `ns`.
- **Unverified val split** on `all_robot_keypoints` (see above).
- **The eval set has few of our robots.** `taxonomy_keypoint.yaml` excludes `house_bot` and
  `object`, so the keypoint metrics rest on the `mr_stabs_mk2` and `mrs_buff_mk3` boxes
  only - a much smaller sample than the 688-frame agnostic figures. Report the matched-box
  count alongside every keypoint metric.
- **The keypoint model is already the slower parallel branch**, so any size increase costs
  tick time directly rather than hiding behind the blob model.
- **A pose win may not be a size win.** `all_robots_pose_2026-07-14.md` found a model
  trained on more classes localized keypoints worse than a dedicated one. If `s` and `x`
  both fail to beat `n` on heading, the answer is that pose accuracy is data-limited, not
  capacity-limited - the opposite of the bbox result, and worth reporting as such.

## Deliverable

`docs/experiments/perception_performance/pose_model_size_<date>.md`, structured like
`model_size_2026-09-04.md`, plus an answer under a new heading in `my_takeaways.md`.
