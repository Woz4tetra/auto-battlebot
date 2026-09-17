# Synthetic domain mix: how much cage data, and does randomized still earn its place

Plan for generating 20,000 NHRL-cage and 20,000 MassD-arena synthetic frames, then training
`yolo26s-pose` on ratios of those against the existing corpus, and one final `yolo26x-pose` arm on
the dataset that wins. Each venue renders a third of its
frames in each camera view: pinhole, rectified and distorted. Five questions, one render budget,
one eval set.

Writeup lands in `docs/experiments/perception_performance/synthetic_domain_mix_<date>.md`.

## Status, 2026-09-13

Step 3 is running. Everything it needed is built and passed a smoke render of both venues, and
the NHRL 20k render is queue job 1.

| Item | State |
| --- | --- |
| `"view"` in the manifest | Done, in `_append_manifest_row` (`synthgen/pipeline.py`) |
| Cutter white faces | Fixed, mechanism kept. Cutters carry a dark interior material and the boolean runs `material_mode = "TRANSFER"`, so the faces a cut opens take it |
| imgsz for step 4 | 640, unchanged. The deployed keypoint engine is `yolo26s-pose` at rect 384x640, so 640 is the scale the robot runs at. The pixel-size gap is what a cage mount sees; it goes in the writeup as a confound |
| Samples | 64 with OptiX, which `config.toml` already sets from the 2026-09-12 comparison (37.6 dB PSNR against 39.9 at 128) |
| A6000 timing probe | Dropped as a separate job. `render_shards.py` records seconds per frame for every run of the real render in `render_shards.json` |
| `CUDA_VISIBLE_DEVICES` passthrough | Already in `run_synthetic.sh`, translated to `--gpus device=N` |
| Sharded render | `training/synthetic/render_shards.py`, with the allocation and merge in `synthgen/shards.py` and `tests/test_shards.py` |
| Gate report | `training/synthetic/domain_render_report.py` |
| Arm lists | `training/yolo/make_domain_mix_arms.py`, built over both renders in `training/data/domain_mix_arms_2026-09-13` |
| NHRL 20k render | Done 22:48, gates passed: `training/data/synth_cage_nhrl_2026-09-13_v2` |
| MassD 20k render | Done 2026-09-14 12:20, gates passed: `training/data/synth_cage_massd_2026-09-13` |
| `base` arm, `yolo26s-pose` | Done 2026-09-14 14:25 after 2 h 05 min; weights, ONNX and sm86 engine in `data/models` |
| `base` scoring test | Queue job 7. The engine parses as `num_keypoints=2, num_classes=4` over 688 GT frames. All classes, agnostic: precision 0.698, recall 0.486, mAP50 0.444. Our robots only: recall 0.843, keypoint error 8.9 px, heading error 8.3 degrees. All 12 runs finished (pooled, heading, both venues, eight recordings). After it, `score_domain_mix.sh` gained a pooled `opponent/` run on `taxonomy_opponent.yaml`, and its venue and recording runs moved to that taxonomy; `base` is rescored with the grid |
| The other 13 grid arms | Queue jobs 8 to 20. Seven done by 2026-09-15 18:40, all exit 0 with weights, ONNX and engine: `d2500` 2:21, `d5000` 2:38, `d10000` 3:11, `d20000` 4:16, `d40000` 6:27, `swap_half` 2:18, `swap_all` 2:18. Training time tracks frame count, 2.3 h at 20k frames to 6.4 h at 58k. By 2026-09-16 00:41 `nhrl_only`, `massd_only` and `view_pinhole` had joined them, all exit 0; `view_rectified` is running, `view_distorted` and `view_mixed` are queued, and `nodamage_swap_half` was added as job 21, so the grid ends about 09:17 |
| Memory under `--cache ram` | `d40000` holds about 49 GB of cached images per DDP rank: 144 GB used and 81 GB available of 251 GB once caching finished, no OOM kill in the kernel log. Claude Code's background watchers were stopped for low memory at that point and restarted |

### The first eight arms, scored 2026-09-15

`score_domain_mix.sh` at the pre-registered `--conf 0.5`, 1000-sample paired bootstrap against
`base`. Opponent recall reads through `taxonomy_opponent.yaml`, heading through
`taxonomy_keypoint_ours.yaml`.

| Arm | Opponent recall | vs `base` | NHRL | MassD | Heading err | vs `base` |
| --- | --- | --- | --- | --- | --- | --- |
| `base` | 0.339 | | 0.376 | 0.066 | 8.27 deg | |
| `d2500` | 0.498 | +0.159 better | +0.126 | +0.396 | 6.87 | -1.41 better |
| `d5000` | 0.450 | +0.110 better | +0.086 | +0.286 | 6.53 | -1.74 better |
| `d10000` | 0.423 | +0.084 better | +0.048 | +0.352 | 6.71 | -1.57 better |
| `d20000` | 0.358 | +0.018 ns | +0.000 ns | +0.154 | 5.81 | -2.46 better |
| `d40000` | 0.295 | -0.045 worse | -0.064 worse | +0.099 | 5.56 | -2.71 better |
| `swap_half` | 0.562 | +0.223 better | +0.185 | +0.505 | 6.30 | -1.98 better |
| `swap_all` | 0.524 | +0.185 better | +0.149 | +0.451 | 7.11 | -1.17 ns |

- **The adoption criterion is met**, by `swap_half` most clearly: opponent recall +0.223 with a CI
  excluding zero, and heading error 1.98 degrees better rather than a degree worse.
- **The amount curve turns over at 100 epochs.** Past `d2500` every extra domain frame costs
  opponent recall, and `d40000` scores below `base`. Precision runs the other way, 0.415 at `base`
  to 0.966 at `d40000`, so the domain-heavy arms are conservative at this threshold rather than
  blind. A `--conf 0.25` pass is running to separate the two; it is unregistered and cannot carry
  the headline. The epoch trajectory below shows this turnover is a property of the fixed 100
  epochs and not of the amount of domain data: at each arm's own best checkpoint the order reverses.
- **Randomized frames are not carrying the result.** `swap_all` beats `d20000` by +0.149 on NHRL
  and +0.296 on MassD with half the frames and no randomized pool at all, so the pre-registered
  "within 0.01 recall" test for dropping randomized passes in the stronger direction.
- **MassD is where domain data pays.** `base` recalls 0.066 of MassD opponents; `swap_half`
  recalls 0.571.
- **Keypoints improve everywhere**, and in the opposite order to opponent recall: `d40000` has the
  best heading error (5.56 deg) and the worst opponent recall, while `d2500` is the reverse.

#### The same eight arms at `--conf 0.25`

Unregistered, run to separate lost detections from a conservative threshold. The headline stays at
0.5. Opponent recall against `base`, whose own recall rises from 0.339 to 0.519 as its precision
falls from 0.415 to 0.294:

| Arm | Recall @0.25 | vs `base` | NHRL | MassD |
| --- | --- | --- | --- | --- |
| `d2500` | 0.712 | +0.193 better | +0.155 | +0.473 |
| `d5000` | 0.642 | +0.123 better | +0.091 | +0.363 |
| `d10000` | 0.651 | +0.132 better | +0.092 | +0.429 |
| `d20000` | 0.623 | +0.104 better | +0.074 | +0.319 |
| `d40000` | 0.579 | +0.060 better | +0.039 | +0.220 |
| `swap_half` | 0.746 | +0.227 better | +0.185 | +0.538 |
| `swap_all` | 0.700 | +0.181 better | +0.131 | +0.549 |

- **The turnover was mostly calibration.** Every arm beats `base` here, `d40000` included, where at
  0.5 it scored 0.045 below. Domain data makes the detector more conservative rather than blind,
  and the fixed 0.5 threshold charged it for that.
- **The ranking does not change.** `swap_half` leads at both thresholds and `d2500` is second, so
  the answer to question 1 holds: opponent recall stops paying after a few thousand domain frames,
  while heading error keeps improving to 40,000.
- **Deployment threshold is now a live question.** The gap between the two thresholds is worth
  more than the gap between most arms, so whatever mix ships should have its confidence picked on
  this eval rather than inherited.

#### Why recall falls as the mix grows

Diagnosed 2026-09-16 from the score files already written plus one size split. Unregistered.

- **Not class confusion.** `wrong_class_rate` falls as domain data grows, 0.077 to 0.021 at conf
  0.5 and 0.133 to 0.083 at 0.25. Opponents are not being relabelled as our robots.
- **Partly the threshold.** At 0.25 every domain arm matches or beats `base` on class-blind recall;
  at 0.5 the domain-heavy arms fall below it, with precision from 0.513 (`base`) to 0.898
  (`d40000`). They are conservative, not blind.
- **The residual is the real-frame share.** Every arm carries the same 452 real frames, whose share
  falls from 2.45 percent (`base`) to 0.77 percent (`d40000`). At matched domain count, adding the
  17,995 randomized frames costs recall: `swap_all` 0.700 against `d20000` 0.623 at 20,000 domain
  frames, `swap_half` 0.746 against `d10000` 0.651 at 10,000.
- **Randomized frames are not the problem; total synthetic is.** At the same 20,452 frames and the
  same 2.21 percent real, half randomized and half domain beats all domain: `swap_half` 0.746
  against `swap_all` 0.700.
- **The loss lands on small robots.** Opponent recall by GT box size at conf 0.25, bins on
  sqrt(area) in source pixels:

  | Bin | GT boxes | `base` | `d2500` | `d20000` | `d40000` | `swap_half` |
  | --- | --- | --- | --- | --- | --- | --- |
  | 24-32 px | 18 | 0.278 | 0.556 | 0.389 | 0.222 | 0.611 |
  | 32-48 px | 245 | 0.473 | 0.706 | 0.588 | 0.486 | 0.739 |
  | 48-64 px | 127 | 0.402 | 0.677 | 0.583 | 0.606 | 0.717 |
  | >64 px | 373 | 0.601 | 0.735 | 0.670 | 0.649 | 0.767 |

  `d40000` hands back nearly all of `d2500`'s gain in the 32-48 px bin, a third of the eval's
  boxes, while holding its gain above 48 px. The cage render puts our robots at 34 to 70 px, so the
  sizes the mix adds most of are the sizes it ends up worst at: synthetic small robots crowd out the
  few real ones rather than teaching the same appearance.
- **Broad, not one opponent.** Seven of eight recordings decline from `d2500` to `d40000`. The
  exception, `16-18-05` at 70 boxes, is the hardest recording for every arm and rises instead.
- **The step count, after all.** This section first read the fixed 100 epochs as innocent, on the
  grounds that `d40000` gets 3.2x `base`'s gradient steps and still loses, so more training could
  not be what produces the drop. The trajectory below refutes that: `d40000` scores 0.606 at epoch
  50 and 0.295 at epoch 100, so more training is exactly what produces the drop. The premise was
  that extra steps can only help.

So question 1's answer is not "20,000 per venue is too much domain data". The dilution reading above
is one candidate and the training schedule is the other, and the trajectory backs the schedule: at
its own peak every arm ranks by domain count. The size-bin and real-share numbers in this section
still stand as measurements, but they were all taken at epoch 100, which is past the peak for the
two arms they indict. Retake them at each arm's peak checkpoint before treating dilution as the
cause, and the oversampled-real arm is the test only if that retake still shows the small-robot gap.

#### Damage, question 4, scored 2026-09-16

`swap_half` against `nodamage_swap_half`, the same 10,000 randomized, 10,000 domain and 452 real
frames, with the damage-free arm drawing its domain frames from the clean pool alone. Conf 0.5,
bootstrap against `swap_half`.

| Arm | Precision | Recall | F1 | Heading err |
| --- | --- | --- | --- | --- |
| `swap_half`, damage on | 0.740 | 0.562 | 0.639 | 6.30 deg |
| `nodamage_swap_half` | 0.627 | 0.600 | 0.614 | 6.89 deg |
| Delta | -0.112 worse | +0.038 better | -0.025 worse | +0.59 ns |

**The registered criterion is not met.** It reads "damage helps if damage-on beats `nodamage` on
opponent recall with a CI excluding zero", and recall runs the other way: the damage-free arm recalls
0.038 more opponents, with the CI excluding zero. Damage buys precision instead, 0.112 of it, which
is the same conservatism axis every domain arm moves along. F1 favours damage on by 0.025.

Keypoints do not move. Heading error is 0.59 degrees worse damage-free and pixel error 0.11 px worse,
both ns, so the protected-part rule did its job: damaged frames neither taught nor corrupted the
heading head.

Two things this does not settle. The eval frames carry no damage label, so this is damage
augmentation's effect on recall over all 688 frames, not its effect on damaged robots, which is what
question 4 actually asks. And the damage-free arm holds a different draw of domain frames rather than
the same scenes with damage switched off, since the clean pool is about 60 percent of the render.

#### Venue transfer, scored 2026-09-16

At the pre-registered conf 0.5, bootstrap against `base`. `swap_all`, `nhrl_only` and `massd_only`
all carry 20,000 domain frames, the same 452 real frames and the same 2.21 percent real share, so
the only difference between them is which venue the domain frames came from.

| Arm | Opponent recall | NHRL frames | MassD frames | Heading err | vs `base` |
| --- | --- | --- | --- | --- | --- |
| `base` | 0.339 | 0.376 | 0.066 | 8.27 deg | |
| `swap_half` | 0.562 | 0.561 | 0.571 | 6.30 | -1.98 better |
| `swap_all` | 0.524 | 0.525 | 0.516 | 7.11 | -1.17 ns |
| `nhrl_only` | 0.412 | 0.446 | 0.154 | 6.71 | -1.56 better |
| `massd_only` | 0.159 | 0.170 | 0.077 | 6.69 | -1.59 better |

- **Question 2's premise fails.** Neither single-venue arm beats the two-venue arm on its own
  venue: `nhrl_only` sits 0.079 below `swap_all` on the NHRL frames, `massd_only` 0.439 below on
  the MassD frames. Venue diversity is what makes domain data work, not venue match.
- **`massd_only` barely detects.** 20,000 MassD frames give 0.077 recall on MassD footage against
  the no-domain `base`'s 0.066, at precision 1.000: it fires rarely and is right when it does.
  `d2500`, holding 1,250 MassD frames, reaches 0.462 there. A single venue is a narrower appearance
  distribution than the randomized pool it replaced, and the detector overfits to it.
- **Heading runs the other way.** Every venue arm improves heading error over `base`, `massd_only`
  included. Our robot's keypoints tolerate a narrow venue; opponent detection does not.

#### Matched presentations, set up 2026-09-16

Step 4's step-count confound, read the way `synthetic_arms_2026-07-31` read it: anchor on `base` at
100 epochs, 1.845 M frame-presentations, and score every richer arm at the checkpoints bracketing
that number instead of at its own epoch 100.

| Arm | Frames | Checkpoints bracketing 1.845 M | Presentations |
| --- | --- | --- | --- |
| `base` | 18,447 | `last`, the anchor | 1.845 M |
| `d2500` | 20,947 | `epoch75`, `last` | 1.571, 2.095 M |
| `d5000` | 23,447 | `epoch75`, `last` | 1.759, 2.345 M |
| `d10000` | 28,447 | `epoch50`, `epoch75` | 1.422, 2.133 M |
| `d20000` | 38,447 | `epoch25`, `epoch50` | 0.961, 1.922 M |
| `d40000` | 58,447 | `epoch25`, `epoch50` | 1.461, 2.922 M |
| `swap_half`, `swap_all` | 20,452 | `epoch75`, `last` | 1.534, 2.045 M |
| view arms | 13,785 | none | `last` reaches 1.379 M |

The four view arms cannot reach the anchor in 100 epochs, so they carry the confound the other way
and their read is a ceiling rather than a match.

`score_domain_mix.sh` now takes an arm as `arm:ckpt`, which overrides `CKPT` for that arm alone and
names it `arm_ckpt` in the output, plus a `RUNS` subset. The matched table needs a different epoch
per arm and only the `opponent` run: twelve runs over ten candidates is an hour of GPU for one
column. The ten bracketing engines build from the `--save-period 25` checkpoints in `data/models`.

`make_domain_mix_arms.py` gained `nodamage_swap_half`, since `swap_half` won the grid. Rebuilding
every arm into a scratch directory reproduced all eighteen `.txt` files byte for byte, so adding
that arm did not move a list any earlier arm trained on. It is queue job 21, behind the two
remaining view arms.

#### What the trajectory says, scored 2026-09-16

Every `--save-period 25` checkpoint of five arms, conf 0.5 through `taxonomy_opponent.yaml`,
bootstrapped against `base` at epoch 100. Opponent recall, precision in brackets:

| Arm | Frames/epoch | ep25 | ep50 | ep75 | ep100 |
| --- | --- | --- | --- | --- | --- |
| `base` | 18,447 | 0.367 (0.297) | 0.284 (0.417) | 0.362 (0.423) | 0.339 (0.415) |
| `d2500` | 20,947 | 0.388 (0.863) | 0.495 (0.792) | 0.533 (0.755) | 0.498 (0.832) |
| `d20000` | 38,447 | 0.588 (0.628) | 0.592 (0.653) | 0.527 (0.776) | 0.358 (0.922) |
| `d40000` | 58,447 | 0.599 (0.617) | 0.606 (0.728) | 0.505 (0.826) | 0.295 (0.966) |
| `swap_half` | 20,452 | 0.385 (0.661) | 0.532 (0.687) | 0.557 (0.678) | 0.562 (0.740) |

- **The amount curve's turnover is a schedule artifact.** Read at its own best checkpoint the grid
  ranks by domain count, the reverse of the epoch-100 table: `d40000` 0.606, `d20000` 0.592,
  `swap_half` 0.562, `d2500` 0.533, `base` 0.367. At matched or lower presentations the domain-heavy
  arms lead everything, and `d40000` at ep25 sees 1.46 M presentations against `base`'s 1.85 M and
  still scores 0.599 to its 0.339.
- **Recall peaks near 2 M frame-presentations whatever the mix**, and falls past about 3 M: `d2500`
  peaks at 1.57 M, `swap_half` at 2.05 M, `d20000` at 1.92 M, `d40000` at 2.92 M. A fixed 100 epochs
  puts `d20000` at 3.84 M and `d40000` at 5.84 M, well past the peak, and puts every other arm at or
  under 2.1 M. The headline table charges the two biggest arms for a schedule the others never hit.
- **`base` has no trend**, 0.284 to 0.367 over its four checkpoints with flat precision. Whatever
  the domain-heavy arms are doing late in training, it is not what 100 epochs does to any detector.
  That spread is also the eval's noise floor, so read differences under 0.05 as nothing.
- **Precision rises monotonically on every domain arm** as recall falls, `d40000` from 0.617 to
  0.966. The arms are drifting into conservatism on real footage, gradually rather than off a cliff,
  and the drift is steeper the more domain data they carry.
- **Unregistered, and best-checkpoint selection reads the eval set.** Picking each arm's peak from
  the same 688 frames it is scored on inflates all five numbers. The pre-registered headline stays
  at epoch 100. To promote this, train `d40000` at the budget its own curve peaks at, near 50
  epochs, and score that as a fresh arm.

Both confirmatory arms are queued. Queue job 22 trains `d40000` for 50 epochs under
`ARM_DATE=2026-09-16`, so it lands beside the 100-epoch weights rather than over them, and tests the
schedule reading on a checkpoint nothing selected on the eval. Queue job 23 trains `d40000_real3x`,
the same 40,000 domain and 17,995 randomized frames with the 452 real ones written three times, which
holds the real share at 2.28 percent against `d40000`'s 0.77 and tests the dilution reading. It
trains for the registered 100 epochs so it is comparable to `d40000`, and `--save-period 25` gives it
the same trajectory the arms above have. If dilution is the cause, `real3x` recovers the small-robot
recall at epoch 100; if the schedule is the cause, it collapses like `d40000` and peaks near 2 M
presentations instead.

#### The view arms, question 5, scored 2026-09-16

All four hold 13,785 frames, no randomized pool, 100 epochs, so each sees 1.379 M presentations and
none reaches the peak the trajectory above puts near 2 M. The schedule effect does not touch this
comparison. Conf 0.5, bootstrap against `base`.

| Arm | Pooled recall | vs `base` | Precision | NHRL 590 | MassD 98 | Heading err |
| --- | --- | --- | --- | --- | --- | --- |
| `base` | 0.339 | | 0.415 | 0.376 | 0.066 | 8.27 deg |
| `view_pinhole` | 0.545 | +0.206 better | 0.461 | 0.542 | 0.571 | 6.88, better |
| `view_rectified` | 0.471 | +0.131 better | 0.512 | 0.479 | 0.407 | 6.82, better |
| `view_distorted` | 0.477 | +0.138 better | 0.704 | 0.464 | 0.571 | 7.89, ns |
| `view_mixed` | 0.486 | +0.147 better | 0.696 | 0.488 | 0.473 | 7.95, ns |

- **No view broke the detector**, which is all this eval was registered to show. Every arm beats
  `base` on pooled opponent recall with a CI excluding zero, and on MassD every one of them turns
  `base`'s 0.066 into 0.407 or better at precision 0.88 to 0.93.
- **`view_pinhole` leads, as predicted, and that is the confound.** The eval is ZED footage the
  camera rectified itself, zero distortion, no border, which is the `pinhole` view. The plan called
  this home-field advantage in advance, so the ranking here cannot answer question 5.
- **Training on distorted frames costs nothing on undistorted input.** `view_distorted` ties
  `view_pinhole` on MassD at 0.571, beats `view_rectified` everywhere, and carries much the highest
  precision of the three single-view arms at 0.704. Whatever the distortion model does to the
  training frames, it does not stop the detector reading frames that have none.
- **`view_rectified` is the weakest on recall**, 0.471 pooled and 0.407 on MassD, on footage that is
  nominally its own view. The render's `rectified` frames carry the alpha 1.0 black border, 36
  percent of the frame for the e-CAM25, and the ZED eval frames carry none. It trained on bordered
  frames and is scored on unbordered ones.
- **Heading splits with the lens.** `view_pinhole` and `view_rectified` improve heading error over
  `base` by about 1.4 degrees; `view_distorted` and `view_mixed` do not move it. Pixel error improves
  for all four.

The answer to question 5 still needs labelled e-CAM25 footage, raw and rectified. Nothing here
changes that, and none of these numbers should be read as a view ranking.

#### The final arms, scored 2026-09-17

Three arms finished after the grid: `d40000` at 50 epochs (queue job 22, 3 h 16 min),
`d40000_real3x` (job 23, 6 h 36 min) and `yolo26x-pose` on `d40000` at 50 epochs (job 25,
12 h 18 min). Conf 0.5, `taxonomy_opponent.yaml`, bootstrapped against `base`.

| Arm | Precision | Recall | vs `base` | NHRL | MassD | Heading err | Kp err |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `base` | 0.415 | 0.339 | | 0.376 | 0.066 | 8.27 deg | 8.90 px |
| `swap_half` | 0.740 | 0.562 | +0.223 better | 0.561 | 0.571 | 6.30 | 7.74 |
| `d40000_s50` | 0.869 | 0.485 | +0.145 better | 0.504 | 0.341 | 6.38 | 7.65 |
| `d40000_x50` | 0.967 | 0.419 | +0.080 better | 0.408 | 0.505 | 5.28 | 5.81 |
| `real3x_s100` | 0.914 | 0.333 | -0.007 ns | 0.350 | 0.209 | 5.87 | 8.05 |
| `d40000_s100` | 0.966 | 0.295 | -0.045 worse | 0.312 | 0.165 | 5.56 | 8.27 |

- **Question 1 is settled, and dilution loses.** `real3x_s100` holds `d40000`'s 40,000 domain
  frames and its 100 epochs, with the real share oversampled from 0.77 to 2.28 percent. It
  recalls 0.333, statistically indistinguishable from `base` and only 0.038 above the collapsed
  `d40000_s100`. Oversampling the real frames did not restore recall. Halving the epochs on the
  same data did: `d40000_s50` reaches 0.485. The collapse past about 3 M frame-presentations is
  the training schedule, not the 452 real frames being diluted, which is the opposite of what
  "Why recall falls as the mix grows" first concluded.
- **The `x` arm does not take rule 1.** Step 4 picks the final arm by highest pooled opponent
  recall, and `swap_half` keeps it at 0.562 against `d40000_x50`'s 0.419. Model size did not buy
  opponent recall.
- **Heading is where size paid.** `d40000_x50` cuts keypoint error to 5.81 px, 1.84 px better
  than the best `s` arm and 3.09 better than `base`, and heading error to 5.28 degrees. Both CIs
  exclude zero. That is the metric the aim-assist consumes.
- **It transfers to MassD far better than its own twin**, 0.505 against `d40000_s50`'s 0.341 on
  identical data and epochs, at precision 0.979.
- **The rule and the deployment question have come apart.** `swap_half` finds more opponents;
  `d40000_x50` is the most precise arm at 0.967, the best on keypoints, and the cleanest on the
  cage-high footage below. Which matters more depends on whether a missed opponent or a false
  lock costs more in a match, and the pre-registered criterion picked recall before any arm
  separated the two this far.

#### The same six at conf 0.25, unregistered

Run to separate a lost detection from a conservative threshold, as the eight-arm pass was. The
headline stays at 0.5.

| Arm | Recall | vs `base` | Precision | F1 |
| --- | --- | --- | --- | --- |
| `base` | 0.519 | | 0.294 | 0.375 |
| `swap_half` | 0.746 | +0.227 better | 0.533 | 0.622 |
| `d40000_s50` | 0.716 | +0.197 better | 0.621 | 0.665 |
| `real3x_s100` | 0.620 | +0.101 better | 0.666 | 0.642 |
| `d40000_x50` | 0.608 | +0.089 better | 0.901 | **0.726** |
| `d40000_s100` | 0.579 | +0.060 better | 0.752 | 0.654 |

- **The `x` arm's recall deficit is not a threshold artifact.** It trails `swap_half` by 0.138
  here against 0.143 at conf 0.5. Lowering the bar lifts every arm together and leaves the order
  intact, so `d40000_x50` genuinely finds fewer opponents at any threshold.
- **On F1 it is the best arm in the experiment.** 0.726 against `swap_half`'s 0.622, because its
  precision holds at 0.901 where `swap_half` falls to 0.533. Its +0.351 F1 over `base` is the
  largest gain any arm has posted.
- **`real3x` stays mid-table**, agreeing with the conf 0.5 read: oversampling the real frames did
  not recover what the schedule cost.
- **The deficit is an NHRL phenomenon.** On the 98 MassD frames `d40000_x50` recalls 0.703 against
  `swap_half`'s 0.714, at precision 0.985 against 0.812, for an F1 of 0.821. On the 590 NHRL frames
  it recalls 0.595 against 0.750. Whatever it misses, it misses at NHRL, where the house bot and a
  busier cage are what distinguish the venues.

#### Box counts on the cage-high footage, 2026-09-17

`nhrl_cage_high_eval`, 650 frames over nine NHRL recordings, copied from pathfinder to
`training/data/`. This is Brettzone broadcast footage from a cage mount, so it is neither the
ZED the eval set uses nor the e-CAM25 the render used. It has no ground truth worth scoring
against yet, so the read is how many boxes each arm produces against how many the footage
should hold. `predict_all_arms.sh` labels it with every arm into its own directory, hardlinking
the images; `conf_sweep.py` keeps each detection's confidence and sweeps thresholds offline.

What the footage should hold, counted per recording rather than assumed. Four recordings, 325
frames, carry the house bot at 0.82 to 0.98 per frame. The three `r*` recordings, 125 frames,
carry none at all: every arm reports 0.00 in every frame, and their names lack the
`Cage-N-Overhead-High` suffix the others have. `clyde` and `sphinx`, 200 frames, hold one that
every arm misses, which `clyde`'s eight hand-labelled frames confirm. That gives
325x3 + 125x2 + 200x3 = **1,825 boxes, 2.81 per frame**, within one box of the rate the ZED eval
set carries for a different reason.

Each arm read at the threshold matching the expected count per class, one Mrs Buff, one opponent
and one house bot per frame:

| Arm | Conf | Class error | buff | opp | house | mr_stabs |
| --- | --- | --- | --- | --- | --- | --- |
| `d40000_x50` | 0.61 | **417** | 622 | 647 | 264 | **0** |
| `d40000_s50` | 0.70 | 485 | 564 | 645 | 257 | 1 |
| `d40000_s100` | 0.49 | 544 | 579 | 719 | 253 | 7 |
| `real3x_s100` | 0.38 | 791 | 609 | 820 | 197 | 127 |
| `base` | 0.75 | 797 | 512 | 655 | 0 | 4 |
| `view_pinhole` | 0.72 | 989 | 442 | 644 | 0 | 125 |
| *expected* | | *0* | *650* | *650* | *650* | *0* |

- **Mr Stabs never fought in these matches, so every such box is a false positive** and needs no
  ground truth to count. `d40000_x50` emits none at its own threshold and 30 at the count-matched
  0.44, against 120 to 137 for nearly every `s` arm. That is the clearest thing model size bought.
- **Matching the total hides a wrong split.** `base` reaches 1,839 boxes at conf 0.57 by emitting
  1,147 opponents and zero house bots. Every arm can hit the total within 1.4 percent, so the
  count alone does not rank them.
- **Nobody finds the house bot**, 264 of 650 at best. Part is real, since `clyde` and `sphinx` are
  missed by everyone, and part is this expectation still crediting those 200 frames a house bot on
  the strength of eight labelled frames. It does not improve at `x`, so it is not model capacity.
- **The existing labels are our-robot-only**: 565 boxes over 549 labelled frames, 541 of them a
  single Mrs Buff box and 8 carrying all three classes. Seeding an eval set from them without
  adding opponents would score opponent recall against almost no ground truth. The set also has no
  `validation_state.json`, only `.edit_state.json`, so nothing records which frames were reviewed.

The tree is not committed. Each render attempt writes `source_<time>.patch` (HEAD plus the
`training/synthetic` diff) into its parts directory. Do not edit `training/synthetic` while a render
job runs: every container mounts the repo live, so a run that starts hours in reads the edit.

A disk cleanup removed `runs/` at 08:56, `runs/queue` included, while the render ran as job 53.
The render was stopped at 09:10 with its three pinhole runs at 788, 825 and 818 frames, each
consistent across images, labels and manifest rows, and resubmitted to a fresh queue at 09:12 as
job 1, which resumes one past the last frame of each run. Stopping `render_shards.py` did not stop
its containers; they had to be stopped with `docker stop`, and resubmitting before that would have
put two renders on the same frame indices. The job 53 log survives as
`runs/queue/logs/0053-render_cage_nhrl_20k.recovered.log`. The same cleanup removed the smoke renders;
the findings below were read before that.

An interim check of 3,457 pinhole frames passed every gate (0 integrity errors, 59.1 to 62.6
percent clean, 2.4 to 2.9 percent hidden keypoints, 0.7 percent dropped), but a 12-frame sample
showed labelled robots floating above the cage walls, Mrs Buff among them. From a wall mount, the
1.0 m airborne ceiling reads as a robot over the glass. `config.toml` now has
`[randomization] air_probability` 0.3 -> 0.1 and `air_height_range` and
`[distractors] robot_air_height_range` capped at 0.6 m instead of 1.0 m. The render restarted
from frame 0 as queue job 3 into `training/data/synth_cage_nhrl_2026-09-13_v2`. The old-config
frames are kept, not merged, in `synth_cage_nhrl_2026-09-13_parts/` (3,797 pinhole frames across
three consistent runs); do not resume or merge that directory into the v2 render.

Job 3 finished all 6,667 pinhole frames by 11:27 at 2.92 to 3.07 s per frame per GPU, then every
`rectified` run exited 2 within 58 s: BlenderProc could not read `depth_0004.exr` back from
`/dev/shm`. The container's `/dev/shm` was Docker's 64 MB default, which holds a pinhole scene's
ten 1280x720 frames and not a warped view's ten 2560x1442 frames. The smoke render passed only
because it ran two frames per scene. `run_synthetic.sh` now gives every container
`--shm-size 8g` (`SYNTH_SHM_SIZE` overrides), and the render resumes into the same v2 directory:
the pinhole runs are complete and the rectified runs wrote nothing.

The resumed render (queue job 4) merged 20,000 frames at 22:48, after 11.3 h from the resume.
Per GPU, pinhole took 2.9 to 3.1 s a frame, rectified 9.0 to 9.3 and distorted 8.8 to 9.0, so a
warped frame costs 3x a pinhole one here, not the 2x the two-GPU probe measured.
`validate_yolo_integrity.py --strict`: 0 errors, 0 warnings, 102,302 annotations (mr_stabs_mk2
11,386, mrs_buff_mk3 17,687, nhrl_robot 55,705, house_bot 17,524). Gate report:

| View | Frames | Clean | Hidden keypoints | Dropped |
| --- | --- | --- | --- | --- |
| pinhole | 6,667 | 58.4% | 2.0% | 1.1% |
| rectified | 6,667 | 60.9% | 7.4% | 0.6% |
| distorted | 6,666 | 58.9% | 7.4% | 0.8% |

The warped views hide keypoints at 7.4 percent against 5.1 in the randomized pool and 2.0 in
pinhole, since they cut robots at the frame edge and at the rectified border. Eighteen sampled
rectified and distorted frames showed boxes and keypoints following the warp and no robot above the
cage walls.

The MassD render (queue job 5) ran 2026-09-13 23:18 to 2026-09-14 12:20, 13.0 h, and merged 20,000
frames: rectified 6,667, distorted 6,667, pinhole 6,666. Per GPU, rectified took 8.9 to 9.1 s a
frame, distorted 8.8 to 9.1 and pinhole 2.9 to 3.0. `validate_yolo_integrity.py --strict`: 0
errors and the expected `house_bot` zero-instance warning, 86,801 annotations (mr_stabs_mk2 11,625,
mrs_buff_mk3 17,650, nhrl_robot 57,526). Gate report:

| View | Frames | Clean | Hidden keypoints | Dropped |
| --- | --- | --- | --- | --- |
| pinhole | 6,666 | 59.2% | 1.0% | 0.8% |
| rectified | 6,667 | 58.7% | 3.9% | 0.9% |
| distorted | 6,667 | 58.5% | 4.0% | 1.1% |

MassD hides fewer keypoints than NHRL in every view (4.0 against 7.4 percent warped): with no house
bot and a cleaner wall line, less of each robot sits at the frame edge. Twenty-seven sampled frames
across the three views showed labels fitting the floor, pit and kick rails. Some show unlabelled,
robot-shaped images beyond the walls, which read as reflections in the 1.41 m glass; real footage
through that glass carries them too, so they stay.

Smoke renders, jobs 51 and 52: 18 frames per venue over three GPUs, every instance damaged
(`-- --images-per-scene 2 --damage all`).

- All 18 runs exited 0 and each merge came out at 6 frames per view. The first merge refused a
  `_debug_frame0.jpg` the pipeline writes into `images/`; the merge now takes numbered frames only.
- `validate_yolo_integrity.py --strict`: NHRL 0 errors, 0 warnings. MassD 0 errors and the expected
  `house_bot` zero-instance warning.
- Damaged Meshy opponents at 3x zoom show dark cut faces and no white.
- Two opponent boxes that cropped to black (60x15 and 25x4 px, both keypoints flag 0) are robots
  cut off by the top edge of the frame, not labels on empty space.
- Hidden keypoints ran 7.8 to 10.7 percent per view at NHRL and 0 to 4.2 percent at MassD, against
  5.1 percent in the randomized pool. Six frames a view with every robot damaged is too few to read;
  the full render's gate report is the check.
- `make_domain_mix_arms.py --scale 0.0009` over both smoke renders built ten arms and verified the
  nesting.

## Status, 2026-09-12

Steps 0, 1 and 2 are done. Step 3 has not started. The 100-frame inspection renders for both
venues are queue jobs 42 and 43 on megamind, writing `training/data/_probe_nhrl_2026-09-12`
and `training/data/_probe_massd_2026-09-12`.

| Step | State |
| --- | --- |
| 0a lens | Done, and it changed the answer |
| 0b timing | Local only: 9.3 s per written frame at 128 samples on an RTX 4080 Laptop. No A6000 number yet |
| 0c schema | Done. `nc: 4` lowercase, straight out of the renderer |
| 1 megamind | Assets, image and code staged. 61 GB free on `/` |
| 2 damage | Done, with three changes to the design |
| 3 render | Not started. Needs `"view"` in the manifest, `render_shards.sh` and the `CUDA_VISIBLE_DEVICES` passthrough |

### What 0a settled

The deployed ZED reads 1280x720, fx = fy = 527.528, cx 644.906, cy 369.885, zero distortion,
across 17 saved MCAPs from MassD_2026-08-29 and NHRL_2026-05-02. That is in
`config/cameras/zed2i_720p.toml`, and it is 101.0 degrees horizontal against the phone's 95.2:
a 10 percent difference in normalized focal length, so the two lenses are not interchangeable
and the question was worth asking.

Both venues render through `config/cameras/ecam25_h01r1_estimated.toml` anyway, because the
e-CAM25 is the camera the mounts are for and the lens they were judged through.
`zed2i_720p.toml` stands as the measured record for a ZED-matched set later.

HD720 is not a scaled HD1080. The one 1080p recording in the corpus reports fx/width 0.554
against 0.412 here, so the sensor crops where HD720 bins. Do not rescale that file.

### Mount ranges

`[cages.mount]` for `nhrl_cage` now comes from 12 poses flown in `pose_camera_server.py` and
covers the near-glass eight of them:

```toml
walls = ["near", "far", "left", "right"]
along_m = [-0.55, 0.55]
height_m = [0.60, 1.25]
inset_m = [-0.20, -0.08]
aim = "centre"
tilt_offset_deg = [-22.0, -10.0]
yaw_deg = [-23.0, 23.0]
roll_deg = [-13.0, 13.0]
```

Every marked pose sits outside the polycarbonate, which is why inset is negative where the
phone-fitted block was positive. That is the cheaper side to shoot from: a pane the camera
looks through from outside is hidden for that frame and costs no labels.

Two things went wrong on the way here, both worth not repeating.

- Bug: the marks only existed in the running server's memory. The render directory under
  `--out` is not evidence they were saved.
- Fix: `curl -s -X POST -d '{"name":"..."}' http://127.0.0.1:8770/save` writes
  `mount_ranges.toml`, and `GET /state` reads the live marks out of a server still up.

- Bug: the first ranges took the envelope of all 12 marks and sampled each axis independently.
  That rendered a camera too far back and aimed too high, filling 24.9 percent of the frame
  with mat against the marks' 31.8. Uniform draws over `inset [-0.75, -0.01]` land past
  -0.17 m in 78 percent of frames where the marks are there in 33, and independent axes paired
  heights and setbacks nothing was flown at into a derived tilt of up to 67 degrees.
- Fix: the ranges above, which cover the near-glass eight. They fill 35.1 percent of the frame
  with mat, median tilt 40.4 degrees, 99.3 percent of the mat in frame. The far-back four
  framed worst of all twelve at 12 to 33 percent.

- Bug: the first near-glass block ran `inset_m` to -0.01. A camera that close sees the panes
  either side of the hidden one edge-on, and rough refraction through a grazing slab never
  converges, so the sampler runs to its cap. A scene at -0.03 took 402 s per render pass at 32
  samples, where a healthy scene takes 4 to 23 s at 128. Runs sampling -0.028 and -0.055 were
  slow on every pass, and one sampling -0.072 to -0.172 over 11 scenes had no slow pass.
- Fix: the near end is -0.08, margin past the cliff between -0.055 and -0.072. MassD holds
  `[-0.25, -0.10]` for the same reason.

A negative `tilt_offset_deg` is steeper, aimed short of the field centre, because tilt is
measured off straight down. Every mark is aimed short of the centre by 5.1 to 18.8 degrees.

### Damage, as built

`synthgen/damage.py` holds the draws and stays out of Blender, so the purity guard covers it
and 33 unit tests do. `synthgen/damage_scene.py` applies a draw and reverts it. Three changes
from the design above:

- **Nothing is deleted.** `load_robots` and the distractor pool load each model once and reuse
  it for every later scene, so a deleted part would stay deleted for the rest of the run. Part
  removal hides parts from the render, which hands both passes the same silhouette; chunk
  removal attaches a boolean difference modifier and takes a cutter from a pool built at
  startup, before `_enable_segmentation` arms the meshes that exist. A `DamageSession` reverts
  both in a `finally` after the scene's frames are written.
- **`separate loose parts` is not used.** Mesh count picks the mechanism: more than one mesh
  goes to part removal, one fused mesh goes to the cutter. Splitting a Meshy mesh is
  destructive, which the reuse rule forbids.
- **The scene split is tracked, not flipped.** Damage has to be drawn per scene, since one
  render call covers every camera pose in the scene. Rolling only per instance at 0.35 left 0
  of 30 frames fully clean on the first probe, not the 40 to 50 percent this plan assumed: a
  frame carries 4.1 robot-like instances, so `0.65^4` is 15 percent before the binomial spread
  of ten scenes. `[damage].scene_probability` now sizes the clean pool directly, and
  `DamageBudget` tracks it the way `choose_cage` tracks the scene mix, so the ratio holds to
  within one scene at any run length.

The two mechanisms measure geometry in different frames, on purpose. The cutter is an
unparented world object, so chunk removal works in world space: a distractor's parent carries
a per-scene scale, and a radius measured in the parent frame comes out wrong by that factor.
Part removal works in the parent frame, because that is where the parts and keypoints were
modelled, so protection does not depend on which way the robot faces. Measured over three
poses, one robot's protected set held at 9 and 36 parts in its own frame while world-space
boxes wobbled between 6 and 71.

At `scene_probability = 0.5` the first 100-frame pair came out 60 percent clean (NHRL) and 50
percent (MassD). NHRL runs high because a scheduled damaged scene can still roll every
instance clean. A 20k render therefore yields 10k to 12k clean frames, so a damage-off arm
cannot match a 20k damage-on arm at the same count. Drop `scene_probability` or accept the
mismatch.

Grading the mechanism: part removal on the CAD robots reads as missing armour with internals
showing. The cutter takes a plausible bite out of a Meshy shell, but the cut exposes unshaded
backfaces that render bright white, which no real robot looks like. A dark interior material
on cut faces would fix it. At 34 to 70 px it may not matter.

### Findings that affect later steps

- **The domain frames put our robots at roughly half the pixels the randomized pool does.**
  Longest bbox edge at 1280x720, median: `mr_stabs_mk2` 39 px (NHRL) and 34 px (MassD) against
  58 px randomized; `mrs_buff_mk3` 70 and 52 against 96. That is faithful to what a cage mount
  sees, but step 4 pins `imgsz 640` for every arm, and 34 to 39 px is the size range that
  already starved the cage-high detector. A domain arm can lose on resolution and read as
  losing on domain. Pre-register imgsz as a factor or record the confound.
- **Keypoint visibility did not move.** 3.9 percent flag-0 on the NHRL domain probe against
  5.2 percent in the randomized pool, so the outside-the-glass mounts and the one-way panes
  are not eating keypoints.
- **`validate_yolo_integrity.py --strict` cannot pass on a MassD render.** It reports zero
  errors and one warning, `house_bot` with zero instances, which is correct: that spec has
  `[house_bot_box] enabled = false`. The step 3 gate needs this written in as an exception.
- **Each written frame costs two render passes.** The clean distractor-free pass doubles every
  render. `[output].ignore_obstructions = true` halves the render cost and drops the occlusion
  gate with it, which is the biggest single lever on a multi-day render.
- **Output is smaller than first budgeted.** 130 to 140 KB per frame, so 40,000 frames is
  about 5.4 GB rather than 8.

### Two config mechanisms added

`extends` and `only_cage`, both in `synthgen/configuration.py`. A per-venue config is three
keys different from the shared one, and copying 1000 lines twice would mean editing every
`[[robots]]` change three times. `config_cage_nhrl.toml` is six lines:

```toml
extends = "config.toml"
only_cage = "nhrl_cage"

[output]
num_images = 20000
```

Tables merge key by key; arrays, including `[[robots]]`, replace wholesale. `extends` must name
a file in the same directory, because relative paths inside the inherited config resolve
against the loaded file. `only_cage` puts the named cage at probability 1.0 and disables the
rest, so no scene lands in the HDRI arena or the other venue.

### Camera views: a third each, 2026-09-13

The rectification step may move later in the C++ pipeline, which would put raw sensor frames in
front of the detector. So each venue's 20,000 frames split three ways:

| View | Frames per venue | What it is |
| --- | --- | --- |
| `pinhole` | 6,667 NHRL, 6,666 MassD | Rendered at the rectified matrix. What every render before 2026-09-13 was |
| `rectified` | 6,667 | The sensor frame through the C++ `Rectifier`'s maps at alpha 1.0, black border included (36 percent of the frame for the e-CAM25) |
| `distorted` | 6,666 NHRL, 6,667 MassD | The raw sensor frame, through the calibration's OpenCV distortion model |

`synthgen/lens.py` does all three, and `pose_camera_server.py` shows its views through the same
module. A run picks one with `--view` or `[[cages]].view`. Labels follow the view: segmentation,
instance and depth maps warp nearest-neighbour, and keypoints are mapped through the same model
with visibility 0 for any the view cannot see.

Cost, measured on 2026-09-13 on two A6000s at 64 samples, over 6 NHRL frames and 10 MassD frames
per view, so treat it as a first read and not the 0b probe:

| Venue | pinhole | distorted | rectified | A third each |
| --- | --- | --- | --- | --- |
| NHRL | 1.85 s/frame | 3.79 s (2.05x) | 3.77 s (2.03x) | 1.69x pinhole |
| MassD | 1.94 s/frame | 4.31 s (2.22x) | 4.31 s (2.22x) | 1.81x pinhole |

A warped view renders 2560x1442 to write 1280x720, four times the pixels, but the frame costs about
twice a pinhole one because the distractor-free clean pass grows much less than the colour pass.

Two gaps this opens:

- **The manifest does not record the view.** `_append_manifest_row` writes venue, scene, mount and
  instances. The three views are separate runs merged into one dataset, so the view arms in step
  4 cannot be filtered until each row carries `"view"`.
- **The eval set has no distorted frames.** `nhrl_keypoints_eval_test` is ZED footage, rectified
  by the ZED with zero distortion at 101 degrees, and the e-CAM25 has not recorded a fight yet.
  See step 5.

The HDRI randomized pool `R` has no lens model and stays pinhole, so any arm mixing `R` with `D`
is more than a third pinhole.

### Left to do

1. The A6000 timing probe, at 128 and 64 samples. It needs a per-cage `render_samples`
   override, not `--render-samples`. Run it per view: the table above is a first read.
2. Decide on the white interior faces the cutter exposes.
3. Decide imgsz for step 4, given the pixel-size gap above.
4. Add `"view"` to each `manifest.jsonl` row.
5. `render_shards.sh` with the per-view split and the `CUDA_VISIBLE_DEVICES` passthrough, then
   step 3.

## Questions

1. **Amount.** How many domain-synthetic frames before the eval curve flattens? Is 20k per
   venue overkill or not enough?
2. **Domain vs randomized.** At a matched frame count, does cage-domain synthetic beat
   randomized HDRI scenes?
3. **Do I need randomized at all?** Once domain data is in, does deleting all 17,995
   randomized frames cost anything?
4. **Damage.** Does randomized part loss on our robot and on opponent meshes improve recall
   on real damaged robots?
5. **View.** At a matched frame count, does a detector trained on distorted frames do as well
   on distorted input as one trained on pinhole or rectified frames does on rectified input?
   And does a mix of all three hold up on both?

Questions 4 and 5 ride along at no extra render cost beyond the views themselves: damage is
sampled per instance and the view is fixed per run, both recorded per frame, so their arms are
filters over the same render, not separate renders. The view is not in the manifest yet; that
change has to land before step 3 or the merged render cannot be split back by view.

## What already exists

| Piece | State |
| --- | --- |
| `training/data/all_robot_keypoints` | 18,447 train (17,995 randomized synthetic + 452 real), 2,049 val. `nc: 3` `[mr_stabs_mk2, mrs_buff_mk3, nhrl_robot]`, `kpt_shape [2, 3]` |
| `training/data/synthetic` | the flat 20,001-frame randomized pool the above was split from |
| `training/synthetic/render_scenes.py` | generic pipeline, `[cage]` section renders a tracked fraction of scenes inside a cage spec at 1280x720. Has `--num-images`, `--start-index`, `--seed`, `--out`, `--render-samples` |
| `training/synthetic/cage/cage2_overhead_high.toml` | NHRL 3 lb cage, graded in `cage_scene_render_match_2026-09-11.md` |
| `training/synthetic/cage/massd_resurgence6.toml` | MassD arena, graded in `massd_arena_scene_2026-09-11.md`. Generic-pipeline integration in flight by another agent |
| `synthgen/cage_mount.py` | samples wall mounts (1.00 to 1.45 m up, 26 to 42 deg tilt, 0.02 to 0.25 m inside the glass), bracketing where our camera goes |
| `training/data/nhrl_keypoints_eval_test` | 688 `pass` frames over 8 recordings: 590 NHRL May, 98 MassD Aug. `nc: 4` `[mr_stabs_mk2, mrs_buff_mk3, opponent, house_bot]` |
| `training/yolo/make_scaling_splits.py` | writes arms as image-list `.txt` files, so arms cost kilobytes and share one disk cache |
| `training/model_eval/{score.py, edit_labels.py, make_eval_dataset.py}` | scoring against TensorRT engines, the label editor, the empty-label dataset builder |

`render_cage_samples.py` is the other cage renderer. It uses the fitted broadcast-camera poses
and belongs to the scene-grading loop, not to this experiment. Everything here goes through
`render_scenes.py` so the mount varies.

## Step 0: settle three things before rendering 40,000 frames

These are cheap and each one can invalidate the render.

### 0a. Point the cage camera at our own intrinsics

`[cage].camera_calibration` is `config/cameras/brettzone_cage_high.toml`, NHRL's phone at
95.2 deg horizontal. Our deployed camera is the ZED. If the two fields of view differ, every
domain frame is rendered through the wrong lens and the whole premise of question 2 is
weakened.

```bash
# The saved MCAPs predate the Foxglove migration, so convert one first.
venv/bin/python scripts/convert_ros1_mcap.py \
  data/saved_recordings/MassD_2026-08-29/auto_battlebot_mrs_buff_mk3_massd_ns_jetson_2026-08-29_13-08-16__2026-08-29T13-20-08.mcap \
  /tmp/massd_converted.mcap
# Then read /camera/camera_info (auto_battlebot.recording.mcap_io.decode_camera_info).
```

Write the result to `config/cameras/zed2i_1080p.toml` in the same schema
(`calibration_id`, `width`, `height`, `fx`, `fy`, `cx`, `cy`, `k1..k3`, `p1`, `p2`) and set
`[cage].camera_calibration` to it. If the ZED numbers land within a few percent of the phone,
record that and move on.

**Done 2026-09-12, with two corrections.** The MassD MCAPs are already in the current format,
so `convert_ros1_mcap.py` skips them with `profile '' is not ros1`; read them straight through
`auto_battlebot.recording.mcap_io.iter_messages`, which yields `(topic, log_time, payload)`
and tags the payload so `decode_camera_info` accepts it. And the camera runs HD720, not 1080p,
so the file is `config/cameras/zed2i_720p.toml`. See the status section for what it says and
why the renders use the e-CAM25 regardless.

### 0b. Timing probe

Nothing in the repo records seconds per frame for a cage scene, and the render is the schedule
driver. Run 200 frames per spec and measure.

The probe runs on megamind, on one A6000, after step 1 has staged the assets there.

```bash
ssh megamind
cd /home/ben/auto-battlebot
CUDA_VISIBLE_DEVICES=0 bash training/synthetic/docker/run_synthetic.sh --require-gpu \
  auto-battlebot-synthetic blenderproc run render_scenes.py -- config_cage_nhrl.toml \
  --num-images 200 --out ../data/_probe_nhrl --render-samples 128 --seed 0
```

Record wall clock, peak VRAM, and the drop rate (`scenes_attempted` vs `images_written`).
Repeat at `--render-samples 64`. If 64 grades the same on a spot check, take it: the cage
config asks for 128 because the glass is noisy, and halving samples halves a multi-day render.

**Two corrections, 2026-09-12.** `--render-samples` never reaches a cage scene: each
`[[cages]]` entry sets its own `render_samples = 128`, which overrides the run value. Testing
64 means a variant config that overrides the per-cage key. And `--require-gpu` used to set only
a flag, leaving the GPU arguments empty, so the command above took the CPU path silently
without even reaching the GPU probe. It now implies `--gpu`.

From seconds per frame, compute the full render cost three ways and pick the shard count:
40,000 frames on one A6000, on two, on all three. That number decides how long the queue is
blocked, which is the real cost of rendering on the training box.

### 0c. One class schema for every arm

Three schemas are live right now and they do not agree:

- `all_robot_keypoints`: `nc: 3`, lowercase, no `house_bot`
- cage renders: `nc: 4`, uppercase `MR_STABS_MK2`, includes `house_bot`
- eval set: `nc: 4`, `opponent` where the training sets say `nhrl_robot`

Pick `nc: 4` `[mr_stabs_mk2, mrs_buff_mk3, nhrl_robot, house_bot]` for training, lowercase
throughout. `all_robot_keypoints` needs a `data.yml` bump only, since its class ids 0 to 2
already match and no frame carries a house bot. The score call is then one string for every
arm:

```
--labels "mr_stabs_mk2,mrs_buff_mk3,opponent,house_bot"
```

The MassD arena has no house bot, so `house_bot` rows come only from the NHRL half. Expect its
AP to move with the NHRL fraction and read it separately.

## Step 1: stage the render on megamind

Rendering moves to megamind, where the training is. That buys three A6000s and drops the
dataset transfer entirely, since the render output is already on the training box. It costs
queue time: the render has to own the GPUs while it runs, so it goes through `gpu_queue.py`
like any arm.

### What megamind has and does not have

Checked 2026-09-12:

| Path | megamind | Action |
| --- | --- | --- |
| `training/data/models` | present, 82 MB | verify the two robot files match, do not re-send |
| `training/data/distractor_models/robots` | **missing**, 3.9 GB local (146 Meshy GLBs) | upload |
| `training/data/distractor_models/distractor_gpu_audit.csv` | **missing**, 224 KB | upload |
| `training/data/environments` | **missing**, 25 MB | upload |
| `training/data/cc_textures` | **missing**, 7.8 GB local across 524 sets | upload **13 sets only**, 151 MB |
| `training/data/hdris` | **missing**, 4.5 GB local | skip |
| `training/data/distractor_models/objaverse` | **missing**, 9.9 GB local | skip |
| `training/data/distractor_models/robots_backup` | **missing**, 2.8 GB local | skip |
| `auto-battlebot-synthetic` docker image | **missing** (playback image is there) | build on megamind |
| NVIDIA container runtime | working, `--gpus all` sees all three A6000s | nothing |
| `/media/storage/auto-battlebots-archive` | present, 1.3 TB free | render output goes here when done |

Three of those skips need justifying, because each one is a multi-GB transfer avoided:

- **HDRIs.** `[cage].probability = 1.0` means no scene ever takes the HDRI arena path.
  `load_environment_assets` globs the HDRI dir only `if hdri_dir.exists()` and returns an
  empty list otherwise, so an absent dir degrades instead of raising. Confirm on the 200-frame
  probe that the log says `0 HDRIs available` and every frame still renders.
- **objaverse.** Its `[[distractors.sources]]` block is commented out in `config.toml`. The
  only live source is `../data/distractor_models/robots`.
- **Most of cc_textures.** Robot and cage materials load through
  `bproc.loader.load_ccmaterials(dir, used_assets=[...])`, so only named sets are read. The
  named sets across `config.toml`, both cage specs, and the `cage_spec.py` dataclass defaults
  are: `Concrete035`, `Foil002`, `Foil003`, `Metal012`, `Metal030`, `Paper001`, `Plastic007`,
  `Plastic007_blue`, `Plastic007_yellow`, `Rubber001`, `Wood027`, plus the two sticker dirs
  `mrs_buff_mk3_top_sticker` and `mrs_buff_mk3_bottom_sticker`. That is 151 MB, not 7.8 GB.

The one path that loads the **whole** texture dir is `load_environment_assets`, which calls
`load_ccmaterials` with no `used_assets` to build the ground-plane material list. In a
cage-only run those ground materials are never applied, so a pruned dir just yields a shorter
list. Watch the `N CC textures available for ground` line on the probe and confirm it says 13
rather than failing.

### Upload

Measured link: 100 MB in 24 s, about 4.2 MB/s. The 4.1 GB payload is roughly 16 minutes.
No `-z`, since GLBs and PNGs are already compressed.

```bash
# 1. Meshy opponent pool and the VRAM audit it is gated by.
rsync -a --info=progress2 \
  training/data/distractor_models/robots \
  training/data/distractor_models/distractor_gpu_audit.csv \
  megamind:/home/ben/auto-battlebot/training/data/distractor_models/

# 2. Cage and arena environments: mat albedos, house bot textures, camera metadata.
rsync -a --info=progress2 training/data/environments/ \
  megamind:/home/ben/auto-battlebot/training/data/environments/

# 3. Only the referenced texture sets. The `./` marks where the preserved path starts, so it
#    belongs at the repo root, not inside cc_textures: with the marker after cc_textures these
#    landed as /home/ben/auto-battlebot/Concrete035 and had to be moved by hand.
rsync -a --info=progress2 --relative \
  ./training/data/cc_textures/{Concrete035,Foil002,Foil003,Metal012,Metal030,Paper001,Plastic007,Plastic007_blue,Plastic007_yellow,Rubber001,Wood027,mrs_buff_mk3_top_sticker,mrs_buff_mk3_bottom_sticker} \
  megamind:/home/ben/auto-battlebot/

# 4. Confirm the robot models already there are the ones the config names.
ssh megamind 'cd /home/ben/auto-battlebot && ls -l "training/data/models/MR STABS MK2.gltf" "training/data/models/MRS BUFF MK3.glb"'
```

Everything else the render needs is tracked in git: `synthgen/`, the per-venue configs, the
cage specs, the Dockerfile, `run_synthetic.sh`. Push the branch and pull it on megamind.

### Build the image on megamind

```bash
ssh megamind 'cd /home/ben/auto-battlebot && \
  docker build -f training/synthetic/Dockerfile -t auto-battlebot-synthetic training/synthetic'
```

The local image is 11.3 GB and bakes Blender 4.2.1 plus both pip trees, so expect a similar
size and a 20 to 40 minute build. megamind's docker root is `/var/lib/docker` on `/`, which
has 76 GB free, so the build fits and leaves about 60 GB.

Build rather than transfer. `docker save | ssh megamind docker load` moves 11.3 GB at
4.2 MB/s, which is 45 minutes, and the Dockerfile pins its pip versions, so a rebuild is
reproducible. Fall back to save-and-load only if the build resolves different apt packages.

### Assets must live inside the repo tree

`run_synthetic.sh` mounts exactly one host path, `-v "${repo_root}:/workspace"`. A symlink
from `training/data/...` out to `/media/storage` dangles inside the container, because the
link target is not mounted. So on megamind the render assets and the render output are real
directories under `/home/ben/auto-battlebot`, not symlinks into `/media/storage`.

`/` on megamind is at 92 percent with 76 GB free. The budget: 4.1 GB of assets, about 11 GB
for the image, and about 5.4 GB of render output. That fits, with roughly 55 GB to spare.

When a venue's render finishes and passes its gates, move it to the archive and point the
training `data.yml` `path:` at the new location. Training reads through the venv, not through
docker, so it does not care where the dataset lives:

```bash
ssh megamind 'mv /home/ben/auto-battlebot/training/data/synth_cage_nhrl_<date> \
  /media/storage/auto-battlebots-archive/'
```

### Check before uploading anything

```bash
timeout 60 venv/bin/python training/gpu_queue.py status
```

An upload is not a GPU job and does not need the queue, but a 4 GB rsync during a training run
evicts its page cache and spikes epoch time about 30x until it recovers. Upload while the
queue is empty, or accept that you just slowed someone else's arm down.

## Step 2: damage as a random variable

New module `training/synthetic/synthgen/damage.py`, new `[damage]` block in `config.toml`,
applied per robot instance after load and before the segmentation pass, so bboxes and keypoint
visibility are computed on the damaged silhouette with no annotation changes.

### Two mechanisms, chosen by mesh structure

**Part removal (CAD robots).** `import_gltf_as_robot` returns the GLB's mesh objects as a
list, so our robots arrive already split into parts. Delete a random subset.

- `severity ~ U(0.05, 0.30)` as a fraction of removable parts.
- Protect the chassis and any part that anchors a keypoint. `[robots.keypoints]` front and
  back are model-frame offsets, so deleting the part under one leaves the keypoint floating in
  air and the label becomes a lie. Protected set is a name-pattern list per robot plus a
  geometric fallback: any part whose bounding box contains a keypoint.
- Protect parts above a volume fraction so a single delete cannot remove most of the robot.

**Chunk removal (Meshy opponents).** Those GLBs are usually one fused textured mesh, so part
removal does nothing.

- First try `separate loose parts`. If it yields more than one island, fall back to part
  removal above.
- Otherwise apply a boolean difference with a randomly placed cutter (cube or icosphere)
  seeded on the mesh surface, scaled to remove `U(0.03, 0.20)` of the bounding volume.
- Reject and resample if the cut leaves the mesh non-manifold in a way that breaks the
  segmentation pass, or removes a keypoint anchor region.

Cosmetic-only damage (scorch marks, roughness patches, darkened albedo) is a third mechanism
and is out of scope for the first pass. Note it as a follow-up.

### Sampling and bookkeeping

- Per instance: `p_damage = 0.35`. Roughly a third of robots in a frame are damaged, which is
  about what a late-round fight looks like.
- Write `manifest.jsonl` beside `images/` and `labels/`, one row per frame:
  `{"image": "000123.jpg", "venue": "nhrl", "mount": {...}, "instances": [{"class": "mrs_buff_mk3", "damage": 0.18, "mechanism": "parts"}]}`.

The manifest is what makes question 4 free. Damage-off arms filter to frames where every
instance has `damage == 0`, damage-on arms take everything. At `p_damage = 0.35` and one to
three robots per frame, roughly 40 to 50 percent of frames are fully undamaged, so a 20k
render yields an 8k to 10k clean pool. Confirm that split on the probe and raise the render
count if the clean pool comes out too thin to match the damage-on arm.

Sanity gate before the full render: render 200 damaged frames and page through
`sheet.png`. Reject the mechanism if robots come out unrecognizable rather than chewed.

**Correction, 2026-09-12.** `render_scenes.py` writes no `sheet.png`; that comes from
`render_cage_samples.py`. Page the output with `training/yolo/validate_yolo_dataset.py
<dataset>`, which draws boxes and keypoints on a grid, and crop in on the largest damaged
instance from `manifest.jsonl` before grading. At the cage mount our robots run 34 to 70 px,
where damage is not visible at all.

## Step 3: render

Two datasets, flat, no split. Splits are image lists later. Each holds a third of its frames in
each view, recorded per frame in `manifest.jsonl`.

`_append_manifest_row` does not write the view yet: a row carries `image`, `venue`, `scene`,
`mount` and `instances`. Add `"view"` before submitting. The per-view runs are hardlinked into one
flat directory, and after the merge nothing else in the output says which view a frame came from.

```
training/data/synth_cage_nhrl_<date>/{images,labels,manifest.jsonl,data.yml}
training/data/synth_cage_massd_<date>/{images,labels,manifest.jsonl,data.yml}
```

`[cage].probability = 1.0` and `[cage].spec` go in per-venue copies of `config.toml`
(`config_cage_nhrl.toml`, `config_cage_massd.toml`) rather than being passed on the command
line, so the render is reproducible from a file.

### What gets rendered

A scene is one arrangement of robots, opponents, lights and damage on the venue's floor, shot from
10 sampled wall mounts (`images_per_scene = 10`). Frames dropped by `min_robot_visibility = 0.10`
cost scenes, not frames: the run keeps drawing scenes until it has written its count.

| Variable | Values | Drawn | Recorded in |
| --- | --- | --- | --- |
| Venue | `nhrl_cage` or `massd_arena` | per run, by config | manifest `venue` |
| View | `pinhole`, `rectified`, `distorted` | per run, by `--view` | manifest `view`, once added |
| Our robots | 1 or 2 of `mrs_buff_mk3` (weight 2.0) and `mr_stabs_mk2` (weight 0.5) | per scene | labels |
| Our robot pose | 30 percent airborne with a random tumble, otherwise flat, upright or inverted, random yaw | per scene | |
| Opponents | 1 to 5 Meshy models, 0.5x to 2x a 0.25 m beetleweight, 10 percent airborne, labelled `nhrl_robot`. Pool re-rolled every 100 images | per scene | labels |
| House bot | in every NHRL scene, none at MassD (`[house_bot_box] enabled = false`) | per venue | labels |
| Camera mount | wall, along, height, inset, tilt offset, yaw, roll from `[cages.mount]`. NHRL 0.60 to 1.25 m up and 8 to 20 cm outside the glass; MassD 0.45 to 1.10 m up and 10 to 25 cm outside | per frame | manifest `mount` |
| Damaged scene | damaged or clean, tracked to 0.5 by `DamageBudget` | per scene | manifest `instances` |
| Damaged instance | 0.35 per robot-like instance in a damaged scene. Mrs Buff loses 1 to 3 named assemblies, Mr Stabs 5 to 30 percent of its parts, a Meshy opponent a chunk of 3 to 20 percent of its bounding volume | per scene | manifest `instances` |
| Lighting | LED tube strength +-25 percent | per scene | |
| Materials | roughness jitter 0.5, hue +-10 degrees | per scene | |
| Motion blur | 30 percent of frames, 5 to 15 px kernel | per frame | |
| Fixed | e-CAM25 calibration, 1280x720 output, 64 samples, rectify alpha 1.0 | | |

Frames per shard and view. Each cell is one `render_scenes.py` run. Each venue is short one frame
of a third; NHRL takes it on `distorted` and MassD on `pinhole`, so every view reaches the 13,333
frames its step 4 arm draws.

| Shard | NHRL pinhole / rectified / distorted | MassD pinhole / rectified / distorted | Frames per shard |
| --- | --- | --- | --- |
| 0, GPU 0 | 2,223 / 2,222 / 2,222 | 2,222 / 2,223 / 2,222 | 6,667 |
| 1, GPU 1 | 2,222 / 2,223 / 2,222 | 2,222 / 2,222 / 2,223 | 6,667 |
| 2, GPU 2 | 2,222 / 2,222 / 2,222 | 2,222 / 2,222 / 2,222 | 6,666 |

Frames by category:

| Category | NHRL | MassD | Both venues |
| --- | --- | --- | --- |
| All frames | 20,000 | 20,000 | 40,000 |
| `pinhole` | 6,667 | 6,666 | 13,333 |
| `rectified` | 6,667 | 6,667 | 13,334 |
| `distorted` | 6,666 | 6,667 | 13,333 |
| Scenes written, at 10 frames each | 2,000 | 2,000 | 4,000 |
| Both our robots in scene | ~10,000 | ~10,000 | ~20,000 |
| Mrs Buff only | ~8,000 | ~8,000 | ~16,000 |
| Mr Stabs only | ~2,000 | ~2,000 | ~4,000 |
| House bot in scene | 20,000 | 0 | 20,000 |
| Fully clean, the damage-off pool | ~11,800 | ~11,800 | ~23,500 |
| At least one damaged instance | ~8,200 | ~8,200 | ~16,500 |

The view rows are exact, set by the run counts. The rest are expectations from the draws above:

- Robots: half of scenes take both, the other half take one, Mrs Buff 80 percent of the time.
  Counts are by scene, before the visibility drop.
- Clean pool: 0.5 + 0.5 x E[0.65^n], with n the robot-like instances in a damaged scene. One or two
  robots plus one to five opponents from a five-model pool gives E[0.65^n] = 0.18, so 58.8 percent
  clean. A pool the VRAM budget holds under five models raises that. The 100-frame probes read 60
  percent (NHRL) and 50 percent (MassD) over ten scenes each. Per view that is about 3,900 clean
  and 2,700 damaged frames.

### Shard across the three A6000s

The queue is strictly serial: one job at a time, whatever `-d` says. So three shards submitted
as three jobs would run one after another. To use all three GPUs the render is **one** queue
job that launches three containers and waits.

Two pieces make that work, as built on 2026-09-13:

1. `run_synthetic.sh` translates `CUDA_VISIBLE_DEVICES` into `--gpus device=N`, so a shard pinned
   to one GPU sees only that GPU.
2. `training/synthetic/render_shards.py`, with the allocation and merge in `synthgen/shards.py`,
   starts one worker per GPU. Each worker renders its shard one container per view, with disjoint
   `--start-index` and distinct `--seed`, into `<out>_parts/shard<i>_<view>/`. Once every run has
   its full count, it hardlinks the runs into one flat `<out>` with `os.link`, rewrites
   `data.yml`'s `path` to the host directory (each run records its container path), and writes
   `render_shards.json` with seconds per frame per run. Rerunning the same command resumes:
   finished runs are skipped, and a run cut short continues one past its last frame under a fresh
   seed.

   Each shard renders its share as three sequential runs, one per `--view`, each a third of the
   shard's frames with its own `--start-index` and `--seed`. That is nine runs for three shards,
   about 2,222 frames each. Splitting by view inside every shard, rather than giving each GPU one
   view, keeps the shards the same length: a warped frame costs about twice a pinhole frame, so a
   pinhole-only GPU would sit idle for half the render.

Separate shard directories rather than one shared `--out`: `--start-index` keeps image
filenames disjoint, but `data.yml` and `manifest.jsonl` are written per run and would race.

```bash
ssh megamind
cd /home/ben/auto-battlebot
venv/bin/python training/gpu_queue.py submit --name render_cage_nhrl_20k --by <agent> -d 0 1 2 -- \
  venv/bin/python training/synthetic/render_shards.py config_cage_nhrl.toml \
    --out ../data/synth_cage_nhrl_2026-09-13 --total 20000 --gpus 0 1 2 --seed-base 0

venv/bin/python training/gpu_queue.py status
venv/bin/python training/gpu_queue.py logs -f
```

MassD is the same command against `config_cage_massd.toml` with
`--views rectified distorted pinhole --seed-base 200`, so it runs short on `pinhole` where NHRL runs
short on `distorted`. The MassD arena is already in `config.toml`.

Check `status` before submitting. A render that owns all three GPUs for many hours pushes every
queued training arm back by that much, so submit it with a name that says what it is and tell
whoever else is queued.

Budget: 130 to 140 KB per 1280x720 JPEG on the 2026-09-12 probes, so 40,000 frames is about
5.4 GB. megamind's `/` has 76 GB free. Fine, and step 1 covers moving the finished datasets to `/media/storage`.

Gates after each render:

```bash
venv/bin/python training/yolo/validate_yolo_integrity.py training/data/synth_cage_nhrl_<date> --strict
venv/bin/python training/synthetic/domain_render_report.py training/data/synth_cage_nhrl_<date> \
  --baseline training/data/all_robot_keypoints/train
```

The report prints a per-view table, writes `gate_report.json` beside the manifest, and exits 1 on a
failed gate: a view off its third, a view dropping more than 25 percent of frames, rows with no view,
or a view with no Mrs Buff.

- Zero errors, zero warnings, except MassD's `house_bot` zero-instance warning, which is correct
  for a spec with `[house_bot_box] enabled = false`.
- Per-class counts printed and recorded. Our robots must not be rare.
- Keypoint visibility distribution: how many rows carry vis-0 keypoints. A jump against the
  randomized pool means the mount or the glass is eating keypoints.
- Drop rate from `min_robot_visibility`. If more than 25 percent of scenes are discarded, the
  mat margin or the distractor count needs a look before burning the rest of the budget.
- Per-view frame counts from `manifest.jsonl`, matching the category table above. Drop
  rate and vis-0 keypoints per view as well, since the distorted and rectified views cut objects
  at the frame edge and at the border where pinhole does not.
- Page 50 random frames per view with `training/yolo/validate_yolo_dataset.py <dataset>`, which
  draws boxes and keypoints on a grid. `render_scenes.py` writes no `sheet.png`.

## Step 4: arms

Every arm is a `.txt` image list, built by `training/yolo/make_domain_mix_arms.py`, which draws
from the corpus and both renders with per-source counts. `make_scaling_splits.py` splits one
dataset by scene and did not extend to several sources cleanly. Frames are drawn by a single fixed shuffle
per source so arms nest: the 10k domain arm is a prefix of the 20k one, and a drop in accuracy
cannot be blamed on which frames got picked.

Constants across arms: `yolo26s-pose`, imgsz 640, batch and epochs fixed, seed 0, 3x A6000 DDP
through the queue, `--save-period 25`, `--cache ram`. The default disk cache writes one
full-resolution `.npy` per frame, about 157 GB for `d40000` against the 74 GB free on megamind.

`R` = randomized frames from `training/data/synthetic`. `D` = domain frames, split evenly
between the two venues unless noted. The 452 real frames are in every arm.

| Arm | R | D | Answers |
| --- | --- | --- | --- |
| `base` | 17,995 | 0 | baseline, the corpus today |
| `d2500` | 17,995 | 2,500 | Q1 |
| `d5000` | 17,995 | 5,000 | Q1 |
| `d10000` | 17,995 | 10,000 | Q1 |
| `d20000` | 17,995 | 20,000 | Q1 |
| `d40000` | 17,995 | 40,000 | Q1, the whole render |
| `swap_half` | 10,000 | 10,000 | Q2, total synthetic held at 20,000 |
| `swap_all` | 0 | 20,000 | Q2 and Q3 |
| `nhrl_only` | 0 | 20,000 NHRL | Q2, venue transfer |
| `massd_only` | 0 | 20,000 MassD | Q2, venue transfer |
| `nodamage` | best mix | same count, damage-free frames only | Q4 |
| `view_pinhole` | 0 | 13,333 pinhole, both venues | Q5 |
| `view_rectified` | 0 | 13,333 rectified, both venues | Q5 |
| `view_distorted` | 0 | 13,333 distorted, both venues | Q5 |
| `view_mixed` | 0 | 13,333, a third of each view | Q5, the render's own mix |

`nodamage` depends on which mix wins Q1 to Q3, so the builder writes both candidates,
`nodamage_d20000` and `nodamage_swap_all`, and the one matching the better of `d20000` and
`swap_all` gets trained.

`D` in the Q1 to Q4 arms draws a third of each view, the way the render lands. The four view arms
hold `R` at zero, because the randomized pool is all pinhole and would tilt every one of them
toward it.

`base`, `swap_all` at 20k and `d20000` share three points on the amount curve, so the grid is
fifteen arms, not twenty.

Every arm in the grid trains `yolo26s-pose`, decided 2026-09-13 to keep the experiment fast. The
earlier plan to confirm three or four arms on `yolo26x-pose` is dropped. `base` on the corpus took
4 h 14 min for 200 epochs on `yolo26s-pose`, so an arm runs about 2 h (base) to 6.5 h (`d40000`)
at 100 epochs.

### The final arm: `yolo26x-pose` on the winning dataset

Once every `yolo26s-pose` arm is scored, one more arm trains `yolo26x-pose` with the same constants
on the winning dataset:

1. The default winner is the arm with the highest agnostic opponent recall on the pooled eval, the
   metric the adoption criterion below uses.
2. The single-venue arms are then checked against `swap_all`, which has the same 20,000 domain
   frames split across both venues: `nhrl_only` on `venue_nhrl`, and `massd_only` on
   `venue_massd`. If a single-venue arm scores higher than `swap_all` on its own venue's eval
   frames, the `yolo26x-pose` arm trains on that single venue's data instead.
3. If both single-venue arms win on their own venues, ask which venue before submitting.

**Resolved 2026-09-16:** neither single-venue arm wins on its own venue (`nhrl_only` -0.079,
`massd_only` -0.439 against `swap_all`), so rule 2 does not fire and the `yolo26x-pose` arm trains
on the winning mixed dataset. `swap_half` leads every arm scored so far.

Submit it the same way as the grid, with `yolo26x-pose` as the model key.

### The step-count confound

`d40000` sees 3.2 times the frames of `base`, so at fixed epochs it also gets 3.2 times the
gradient steps and part of any win is just more training. Handle it the way
`synthetic_arms_2026-07-31` did: keep epochs fixed at 100 for the headline table, and use the
`--save-period 25` checkpoints to read every arm again at matched frame-presentations. Report
both. If the win survives at matched steps it is the data.

At 100 epochs Ultralytics writes `epoch25.pt`, `epoch50.pt` and `epoch75.pt` but no `epoch100.pt`;
`last.pt` is the epoch-100 checkpoint. `run_domain_mix_arm.sh` copies all four.

### The val set is not a decision surface

`all_robot_keypoints/val` is 2,004 synthetic and 45 real. Arms trained on more synthetic will
look better on it for reasons that have nothing to do with the field. Use it for training
bookkeeping and early-stopping only. Every claim in the writeup comes from `score.py` on
`nhrl_keypoints_eval_test`.

## Step 5: score

```bash
venv/bin/python training/yolo/convert_to_onnx.py data/models/yolo26s-pose_<arm>_<date>.pt
venv/bin/python training/yolo/convert_to_tensorrt.py data/models/yolo26s-pose_<arm>_<date>.onnx --workspace 4

venv/bin/python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
  --candidate base=data/models/yolo26s-pose_base_<date>_x86_64_sm89.engine \
  --candidate d20000=data/models/yolo26s-pose_d20000_<date>_x86_64_sm89.engine \
  --labels "mr_stabs_mk2,mrs_buff_mk3,opponent,house_bot" \
  --taxonomy training/model_eval/taxonomy.yaml --conf 0.5 --baseline base \
  --output training/data/nhrl_keypoints_eval_test/scores_domain_mix
```

Build engines on pathfinder (sm89), not megamind (sm86). Check the printed
`num_keypoints=2 num_classes=4` line on every run: a wrong `--labels` length misparses the
tensor and returns near-zero recall that looks like a broken engine.

Score three ways:

1. **Pooled**, all 688 frames, paired bootstrap against `base`.
2. **Per venue.** NHRL May (590 frames) against MassD Aug (98). This is the direct read on
   question 2: NHRL-cage synthetic should move the May recordings and MassD synthetic should
   move the August one. If `nhrl_only` lifts MassD as much as `massd_only` does, then the win
   is generic cage-ness, not venue match, and there is no reason to build a scene per venue.
3. **Per recording.** Each recording is one opponent, so per-recording recall is per-opponent
   grade. Pooled AP understates the good cases.

Keypoint metrics go through `taxonomy_keypoint_ours.yaml`, which excludes opponents so heading
error reflects our robot.

### Scoring the view arms

`nhrl_keypoints_eval_test` cannot answer question 5 on its own. It is ZED footage: rectified by
the camera, zero distortion, 101 degrees, no black border. That is closest to `pinhole`, so on it
`view_pinhole` has a home-field advantage and `view_distorted` is scored on input it was never
meant for.

- Score all four view arms on it anyway and report it as a check that no view broke the
  detector, not as the answer.
- The answer needs e-CAM25 footage: raw frames for the distorted arm, and the same frames through
  `rectify_maps` for the rectified arm, labelled once in the distorted frame and mapped with
  `undistort_points`. Record and label that before scoring question 5 for real.

### Pre-registered criteria

Write these down before the first score run and do not move them afterwards.

- **Adopt** a domain mix if agnostic opponent recall on the pooled eval rises by at least 0.03
  with a 95 percent CI excluding zero, and our-robot heading error does not get worse by more
  than 1 degree. Opponent recall is read with `training/model_eval/taxonomy_opponent.yaml`,
  which excludes our robots, the house bot and `object` from GT and predictions; `taxonomy.yaml`
  scores all four classes together and is not this metric. Heading error is read with
  `taxonomy_keypoint_ours.yaml`.
- **Drop randomized** if `swap_all` is within 0.01 recall of `d20000` on both venues.
- **Damage helps** if `damage-on` beats `nodamage` on opponent recall with a CI excluding zero.
- Anything else that moves is an unregistered finding and needs a confirmatory run before it
  drives a deployment decision. `synthetic_arms_2026-07-31` pre-registered recall, got a
  precision win, and had to label it unregistered. Same discipline here.

## Step 6: grow the eval set with pre-labels

The 98 MassD frames are the weak point. Any per-venue claim about MassD rests on them, and 98
frames gives a wide CI no amount of bootstrap resampling fixes. The fix is more labeled
frames, and the pre-label loop is what makes that affordable.

**Assumption to confirm:** `nhrl_keypoints_eval_test` is already fully labeled (688 `pass`
frames, no empty label files), so "hand label the NHRL eval test set" means growing it, mostly
on the MassD side, plus finishing `nhrl_cage_high_eval` (650 frames, 201 still empty). Both
use the identical loop below. Say which one comes first and I will order the steps.

### The loop

1. **Sample frames.** `make_eval_dataset.py` at a higher `--per-video` over the MCAPs, writing
   empty labels.

```bash
venv/bin/python training/model_eval/make_eval_dataset.py \
  'data/saved_recordings/MassD_2026-08-29/*.mcap' \
  --output-dir training/data/nhrl_keypoints_eval_test --per-video 250 \
  --extra-classes opponent house_bot
```

2. **Pre-label.** New script `training/model_eval/prelabel_dataset.py`: runs a `.pt` over each
   subdataset's `images/` and writes YOLO pose rows into `labels/`. `export_labels.py` cannot
   do this, since it reads detection topics out of a `label_playback` MCAP and these frames
   have none.

   Run it at `--conf 0.15`, not 0.5. Deleting a spurious box in `edit_labels.py` is one
   keypress; drawing a missing box plus two keypoints is a dozen actions. Bias the pre-labeler
   toward over-detection.

3. **Correct** in `edit_labels.py`, `space` to mark reviewed and jump.

4. **Merge** the review state with `merge_validation_state.py`.

5. **Second round.** The first pre-labeler is whichever arm is best today. Once `d20000` or
   `swap_all` exists, re-pre-label the frames not yet reviewed with it. A model trained on
   MassD-domain synthetic should pre-label MassD frames better than anything trained without
   it, which is the practical payoff of this experiment independent of the deployment result.

### Two guards, because pre-labeled GT can poison the eval

- **Miss bias.** A frame where the pre-labeler sees nothing arrives empty, and an empty frame
  looks reviewed at a glance. Every frame gets opened at fixed zoom, and no frame is marked
  reviewed from the thumbnail.
- **Blind audit.** Hold out 10 percent of frames, label them from empty with no pre-labels,
  and compare box counts against the pre-labeled population. That number is the measured bias
  the eval carries, and it goes in the writeup. If the pre-labeled set has systematically
  fewer boxes, the eval flatters every model, including the one that did the pre-labeling.
- **Never pre-label with an arm and then score that arm as the headline** without the audit
  number beside it.

Also measure the speedup: time 50 frames pre-labeled and 50 from empty. If pre-labeling does
not actually save time, say so and drop it.

## Risks

- **Render throughput is unmeasured.** Everything downstream is scheduled off step 0b. At 10
  seconds per frame, 40,000 frames is 111 GPU-hours, which is 37 wall-clock hours sharded
  three ways. That is 37 hours of no training for anyone. If the probe lands near that, cut
  the render to 10,000 per venue and spend the saved time on the amount curve instead of its
  tail.
- **The view split raises that cost by 1.7x to 1.8x.** A warped frame costs about twice a pinhole
  one, so a third each lands at 1.69x (NHRL) and 1.81x (MassD) the all-pinhole cost on the first
  measurement. 37 wall-clock hours becomes about 65. The same cut applies: at 10,000 per venue the
  split is 3,333 per view per venue, still 6,666 per view across both.
- **Question 5 has no fair eval yet.** The eval set is ZED footage, which matches none of the
  three views exactly and favours pinhole. Without labelled e-CAM25 frames, any view result is a
  sanity check, not a decision.
- **The render blocks the queue.** Rendering on the training box is the whole point of step 1,
  and the cost is that other agents' arms wait. Announce the submission, and do not start the
  MassD render until the NHRL dataset has passed its gates, so a bad spec does not cost two
  slots.
- **megamind `/` is at 92 percent.** 4.1 GB of assets, 11 GB of docker image and 5.4 GB of
  render output fit in the 76 GB free, but nothing else large does. Move each finished dataset
  to `/media/storage` before starting the next render.
- **Docker mounts only the repo root.** Assets symlinked out to `/media/storage` dangle inside
  the container. Keep render inputs and outputs as real directories under the repo on
  megamind.
- **Pruned assets are a bet on two code paths.** Skipping the HDRIs and 511 of 524 texture sets
  rests on `[cage].probability = 1.0` never taking the arena path and on `load_ccmaterials`
  honoring `used_assets`. The 200-frame probe is what confirms it. If the probe logs missing
  textures or falls back to a default world, upload the rest before committing to 40,000
  frames.
- **Intrinsics mismatch.** Covered by 0a, and it is the single item most likely to make the
  domain arms underperform for a reason unrelated to the hypothesis.
- **MassD integration is in flight.** The MassD render cannot start until the other agent's
  generic-pipeline work lands. Start the NHRL render first.
- **Damage can make labels wrong.** A keypoint on a deleted part is a false label that trains
  the heading head toward noise. The protected-part rule and the 200-frame visual gate are the
  defense; if either is shaky, ship damage as a separate small dataset instead of mixing it
  into the main render.
- **`yolo26s-pose` may not rank arms the way `yolo26x-pose` would.** The grid runs on `s` for
  speed and only the winner is retrained on `x`, so an arm that `x` would have preferred can lose
  on `s`. The final arm's score against the `s` winner is the only check on that.
- **Segmentation stops at glass.** Any mount rendering through polycarbonate loses its labels.
  This is no longer handled by keeping the camera inside: every marked mount sits outside, and
  `apply_one_way_glass` hides the pane a camera looks through from outside, per frame. That is
  why `inset_m` stays strictly negative, so the hiding always applies. The 2026-09-12 probe
  found keypoint visibility unchanged against the randomized pool, 3.9 percent flag-0 against
  5.2, so the mechanism holds.

## Next steps

As of 2026-09-17. Everything the plan set out to run has run: both renders and their gates, the
fifteen-arm `yolo26s-pose` grid, the three follow-up arms, and the final `yolo26x-pose` arm, all
scored. Questions 1 to 5 are answered above. What is left is a decision and two datasets:

1. **Decide what ships, because the registered rule no longer picks one model.** Rule 1 takes the
   highest pooled opponent recall, which is `nodamage_swap_half` at 0.600, with `swap_half` at
   0.562. `d40000_x50` recalls 0.419 and wins everything else: precision 0.967, the best F1 in the
   experiment at conf 0.25 (0.726), keypoint error 5.81 px against the best `s` arm's 7.65, and the
   cleanest class composition on cage-high footage. Its recall deficit is not a threshold artifact,
   so this is a real trade of missed opponents against false locks, and the criterion was
   pre-registered before any arm separated the two this far. Pick the deployment confidence on this
   eval at the same time: the gap between conf 0.5 and 0.25 is worth more than the gap between most
   arms, and 0.5 was inherited rather than measured.
2. **The cage-high set is a second eval only after its opponents are labelled.** Its 549 labelled
   frames carry Mrs Buff alone, 541 of them a single box, so opponent recall would be scored against
   almost nothing. `training/data/cage_high_x50_conf044/` holds `d40000_x50`'s labels at its
   count-matched threshold as a correction seed, and `training/data/cage_high_arm_labels_2026-09-16/`
   holds all fifteen `s` arms at conf 0.15. Nothing records review state in that set, so it starts
   fresh whichever seed is chosen.
3. Step 6. MassD frames are sampled into `training/data/nhrl_keypoints_eval_grow_massd_2026-09-13`,
   not into the eval set, so the set this experiment scores on does not change under it. The
   MassD MCAPs live on pathfinder; the 11 clipped `__` segments (3.8 GB) were copied to megamind.
   The staging set holds 2,400 frames: 250 per recording from the 10 clips not already in the eval
   set, and 150 from the one clip that runs short. None of the ten has a `svo_start_frame` in the
   playback config, so sampling starts at frame 0 and includes pre-match frames. Pre-labels come
   from `yolo26x-pose_all_robot_keypoints_2026-09-05_last.pt` at conf 0.15 with
   `--map nhrl_robot=opponent mr_stabs_mk2=opponent`, since only Mrs Buff fought at MassD, and a
   seeded 10 percent of frames per recording stays empty for the blind audit
   (`prelabel_holdout.json`). Correcting in `edit_labels.py` and labelling the hold-out are hand
   work. The pre-label pass wrote 2,160 frames with 1,731 boxes and held out 240. Boxes per
   recording run from 1 (`08-48-31`) and 4 (`09-50-45__09-52-23`) to 376 (`09-20-17__09-27-24`),
   so a few clips are mostly empty arena or pre-match footage and review fast.
4. Record and label e-CAM25 footage for question 5, raw and rectified. Needs the robot.
