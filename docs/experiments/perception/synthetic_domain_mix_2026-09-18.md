# Synthetic domain mix: results from the eighteen-arm grid

What 40,000 rendered cage frames bought, measured over eighteen `yolo26s-pose` and
`yolo26x-pose` arms on two eval sets. The plan, its pre-registered criteria and the running
status log are in
[synthetic_domain_mix_plan_2026-09-12.md](synthetic_domain_mix_plan_2026-09-12.md); this file
holds every result.

Unless a section says otherwise, numbers are on `nhrl_keypoints_eval_test`: 688 hand-labelled
frames from the robot's own ZED, 590 NHRL May and 98 MassD August, scored at the pre-registered
`--conf 0.5` with a 1000-sample paired bootstrap against `base`. Opponent recall reads through
`taxonomy_opponent.yaml`, heading through `taxonomy_keypoint_ours.yaml`.

## What ran

Two renders of 20,000 frames each, `nhrl_cage` and `massd_arena`, a third of each in the
`pinhole`, `rectified` and `distorted` camera views. The arms are image lists over those frames
plus the existing corpus, so they cost kilobytes and share one disk cache.

| Arm | Frames | What it holds |
| --- | --- | --- |
| `base` | 18,447 | 17,995 randomized + 452 real. No domain frames |
| `d2500` | 20,947 | base + 2,500 domain |
| `d5000` | 23,447 | base + 5,000 domain |
| `d10000` | 28,447 | base + 10,000 domain |
| `d20000` | 38,447 | base + 20,000 domain |
| `d40000` | 58,447 | base + 40,000 domain |
| `d40000_real3x` | 59,351 | `d40000` with the 452 real frames written three times |
| `swap_half` | 20,452 | Half the randomized pool swapped for 10,000 domain |
| `swap_all` | 20,452 | All randomized swapped for 20,000 domain |
| `nodamage_swap_half` | 20,452 | `swap_half` drawing domain frames from the clean pool alone |
| `nhrl_only` / `massd_only` | 20,452 | 20,000 domain frames from one venue |
| `view_pinhole` / `view_rectified` / `view_distorted` / `view_mixed` | 13,785 | One camera view each, no randomized pool |

Every arm in the table trains `yolo26s-pose` for 100 epochs at `imgsz 640`. Two confirmatory
arms changed the schedule or the model instead of the data: `d40000` retrained at 50 epochs
(`d40000_ep50`, written `d40000_s50` in the tables below) and `yolo26x-pose` on `d40000` at 50
epochs (`x_d40000_ep50`, written `d40000_x50`).

The pre-registered criteria, restated from the plan: adopt a mix if agnostic opponent recall
rises by at least 0.03 with a CI excluding zero and heading error does not worsen by more than
1 degree; drop randomized if `swap_all` is within 0.01 recall of `d20000` on both venues; damage
helps if damage-on beats `nodamage` on opponent recall with a CI excluding zero. Anything else is
unregistered and needs a confirmatory run before it drives a deployment decision.

## Question 1: how much domain data

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

### The same eight arms at `--conf 0.25`

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

### Why recall falls as the mix grows

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

### Matched presentations, set up 2026-09-16

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

### What the trajectory says, scored 2026-09-16

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

## Question 2: domain against randomized, and whether venue match matters

### Venue transfer, scored 2026-09-16

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

## Question 3: does randomized still earn its place

The registered test was whether `swap_all` lands within 0.01 recall of `d20000` on both venues.
It passes in the stronger direction: `swap_all` recalls 0.524 against `d20000`'s 0.358 pooled,
+0.149 on NHRL and +0.296 on MassD, with half the frames and no randomized pool at all. Deleting
all 17,995 randomized frames costs nothing measurable on opponent detection.

It does cost keypoints. `swap_all` is the one swap arm whose keypoint error goes backwards, 5.69
px against `swap_half`'s 3.69 on the cage-high set and 7.11 degrees of heading error against 6.30
on ZED, the only swap arm whose heading gain against `base` fails to exclude zero. Half and half
is the better trade than all domain.

## Question 4: damage

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

## Question 5: camera view

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

## The confirmatory arms and the `yolo26x` model

### The final arms, scored 2026-09-17

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

### The same six at conf 0.25, unregistered

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

## The second eval set: cage-mount broadcast footage

Everything above is the robot's own ZED. The cage-high set is 636 hand-corrected frames from a
cage mount, six NHRL Brettzone recordings and three MassD ones, described in
[cage_high_eval_scoring_2026-09-18.md](cage_high_eval_scoring_2026-09-18.md). It ranks the grid
differently, and the reason is geometric.

One caveat applies to every number in this section: that set's ground truth was pre-labelled by
`x_d40000_ep50` at conf 0.44 with no blind hold-out, then corrected by hand. That arm is
therefore partly scored against its own output, most severely on keypoints, and the other
seventeen arms are comparable with each other but not with it.

### Why the ranking inverts: the robots are 2.3 times the size

Opponent boxes on this footage have a median sqrt(area) of 146.8 px against the ZED set's 62.5,
and take 0.103 of the frame against 0.059. A cage mount looking down at a 3 lb arena shows a
robot about 5.5 times the area the robot's own camera does across the mat. Finding the robot is
close to solved at that size, which is why `base` recalls 0.856 here and 0.339 there, and why
the arms separate on false positives instead.

### Opponent detection, conf 0.5

`taxonomy_opponent.yaml` over all 636 frames: every GT opponent box, matched by any remaining
prediction. This is the metric the plan's adoption rule reads.

| Arm | Recall | vs `base` | Precision | F1 | mAP50 | Opp kp err | Opp heading |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `base` | 0.856 | | 0.399 | 0.544 | 0.658 | 21.9 px | 15.8 deg |
| `d2500` | 0.916 | +0.061 better | 0.791 | 0.849 | 0.859 | 19.5 | 15.1 |
| `d5000` | 0.901 | +0.046 better | 0.819 | 0.858 | 0.847 | 16.8 | 10.9 |
| `d10000` | 0.862 | +0.007 ns | 0.865 | 0.863 | 0.824 | 13.7 | 8.9 |
| `d20000` | 0.842 | -0.013 ns | 0.868 | 0.855 | 0.813 | 13.2 | 8.1 |
| `d40000` | 0.857 | +0.002 ns | 0.883 | 0.870 | 0.835 | 14.9 | 10.6 |
| `d40000_ep50` | 0.895 | +0.039 better | 0.792 | 0.840 | 0.870 | 17.1 | 11.7 |
| `d40000_real3x` | 0.860 | +0.005 ns | **0.969** | 0.911 | 0.858 | 14.8 | 9.9 |
| `swap_half` | **0.929** | +0.074 better | 0.714 | 0.807 | 0.855 | 19.4 | 16.0 |
| `swap_all` | 0.849 | -0.007 ns | 0.666 | 0.747 | 0.775 | 18.7 | 14.4 |
| `nodamage_swap_half` | 0.911 | +0.056 better | 0.755 | 0.826 | 0.845 | 17.6 | 12.2 |
| `nhrl_only` | 0.810 | -0.046 worse | 0.497 | 0.616 | 0.662 | 16.8 | 12.7 |
| `massd_only` | 0.238 | -0.617 worse | 0.718 | 0.358 | 0.214 | 18.7 | 16.4 |
| `view_pinhole` | 0.854 | -0.002 ns | 0.476 | 0.611 | 0.653 | 21.6 | 16.0 |
| `view_rectified` | 0.811 | -0.044 worse | 0.348 | 0.487 | 0.604 | 21.6 | 19.5 |
| `view_distorted` | 0.860 | +0.005 ns | 0.595 | 0.703 | 0.774 | 23.9 | 20.1 |
| `view_mixed` | 0.819 | -0.036 worse | 0.659 | 0.731 | 0.722 | 20.3 | 15.8 |
| `x_d40000_ep50` | 0.910 | +0.054 better | 0.917 | **0.913** | **0.893** | **6.9** | **5.3** |

- **This footage is easy to find robots in and hard to be right about.** `base`, the arm with no
  domain data at all, recalls 0.856 opponents here against 0.339 on the ZED set. Fifteen of the
  eighteen arms land between 0.81 and 0.93. Recall barely separates them; precision spans 0.348 to
  0.969, a range of nearly three to one, and every arm but `view_rectified` beats `base` on it with
  a CI excluding zero.
- **The ranking inverts against the ZED eval set.** There the amount curve fell with domain count
  and `swap_half` led at 0.562 while `d40000` collapsed to 0.295. Here `d40000` matches `base` on
  recall and beats it by 0.484 precision, and on F1 the order is almost exactly the ZED order
  reversed: `d40000_real3x` 0.911 and `d40000` 0.870 against `swap_half`'s 0.807. The conservatism
  the plan measured as a cost on ZED footage is worth more than it costs here.
- **`d40000_real3x` is the surprise.** It was statistically indistinguishable from `base` on ZED
  (0.333 recall, "oversampling the real frames did not restore recall") and it is the most precise
  arm in the experiment here, 0.969 at 0.860 recall. Whatever oversampling the 452 real frames did,
  it shows up on cage-mount footage and not on the robot's own camera.
- **`massd_only` fails on both sets.** 0.238 recall, the only arm worse than `base` on F1. It holds
  20,000 MassD frames and still cannot find opponents in MassD broadcast footage, which is the same
  result the ZED set gave at 0.159. A single venue is too narrow whichever venue you test on.
- **The view arms rank as they did on ZED**, `view_pinhole` and `view_distorted` ahead of
  `view_rectified`, and they are the weakest family here: three of the four sit below 0.60 F1 on
  precision alone. `view_rectified` is the only arm that loses to `base` on precision.

### Pooled, every class together, conf 0.5

`taxonomy.yaml` scores our robot, the opponent and the house bot as one population. Agnostic
recall asks whether a box landed on the robot; instance recall asks whether it was also named
correctly.

| Arm | Agnostic recall | vs `base` | Instance recall | Naming gap | Instance precision | F1 |
| --- | --- | --- | --- | --- | --- | --- |
| `base` | 0.774 | | 0.615 | 0.158 | 0.567 | 0.742 |
| `d2500` | 0.835 | +0.061 better | 0.803 | 0.031 | 0.843 | 0.855 |
| `d5000` | 0.817 | +0.043 better | 0.787 | 0.030 | 0.891 | 0.868 |
| `d10000` | 0.783 | +0.010 ns | 0.755 | 0.029 | 0.907 | 0.855 |
| `d20000` | 0.679 | -0.094 worse | 0.659 | 0.021 | 0.917 | 0.791 |
| `d40000` | 0.725 | -0.049 worse | 0.715 | 0.010 | 0.941 | 0.824 |
| `d40000_ep50` | 0.834 | +0.061 better | 0.810 | 0.025 | 0.873 | 0.866 |
| `d40000_real3x` | 0.643 | -0.131 worse | 0.635 | 0.008 | **0.971** | 0.777 |
| `swap_half` | 0.770 | -0.004 ns | 0.711 | 0.058 | 0.812 | 0.820 |
| `swap_all` | 0.785 | +0.011 ns | 0.738 | 0.046 | 0.810 | 0.821 |
| `nodamage_swap_half` | **0.848** | +0.074 better | 0.811 | 0.037 | 0.856 | 0.871 |
| `nhrl_only` | 0.696 | -0.078 worse | 0.639 | 0.057 | 0.635 | 0.693 |
| `massd_only` | 0.331 | -0.443 worse | 0.317 | 0.014 | 0.885 | 0.487 |
| `view_pinhole` | 0.708 | -0.066 worse | 0.650 | 0.058 | 0.640 | 0.702 |
| `view_rectified` | 0.749 | -0.025 worse | 0.663 | 0.086 | 0.530 | 0.665 |
| `view_distorted` | 0.783 | +0.009 ns | 0.742 | 0.041 | 0.742 | 0.783 |
| `view_mixed` | 0.787 | +0.013 ns | 0.727 | 0.059 | 0.805 | 0.827 |
| `x_d40000_ep50` | 0.839 | +0.065 better | **0.830** | 0.009 | 0.934 | **0.888** |

- **Pooling charges the domain-heavy arms for the house bot they miss.** `d40000_real3x` leads the
  opponent table and is second-worst here at 0.643 agnostic recall, because a third of the pooled
  GT is house bots it does not find. Which table is right depends on whether the house bot matters
  to the task; for aim assist it does not, for a cage-mount labelling model it does.
- **`base` finds robots and names them wrong.** Its 0.158 naming gap is nearly twice the next arm's,
  and only `view_rectified` names boxes worse, at 0.530 instance precision against `base`'s 0.567. Every domain arm closes the gap,
  and the most domain-heavy close it almost entirely: `d40000_real3x` 0.008, `x_d40000_ep50` 0.009.
- **`x_d40000_ep50` is the only arm strong on both axes**, 0.839 agnostic and 0.830 instance recall
  at 0.934 precision, for the best pooled F1 in the grid. It is also the arm that pre-labelled this
  GT, so read it with the caveat above.

### Per class, conf 0.5

One inference pass per arm over all 636 frames, matches broken out by GT class. "Extra boxes"
counts predictions that matched no GT box at all; "wrong class" counts boxes that landed on a
robot and named it something else.

| Arm | House bot 512 | Mrs Buff 629 | Opponent 609 | Mr Stabs FP | Extra boxes | Wrong class |
| --- | --- | --- | --- | --- | --- | --- |
| `base` | **0** | 556 | 521 | 11 | 545 | 277 |
| `d2500` | 279 | 569 | 558 | 9 | 207 | 55 |
| `d5000` | 247 | 582 | 549 | 7 | 116 | 52 |
| `d10000` | 213 | 583 | 525 | 9 | 85 | 50 |
| `d20000` | 57 | 583 | 513 | 0 | 68 | 36 |
| `d40000` | 154 | 575 | 522 | 0 | 60 | 18 |
| `d40000_ep50` | 287 | 585 | 545 | 12 | 163 | 43 |
| `d40000_real3x` | 9 | 579 | 523 | 0 | **19** | 14 |
| `swap_half` | 104 | 575 | **566** | 8 | 187 | 102 |
| `swap_all` | 207 | 568 | 517 | 14 | 222 | 81 |
| `nodamage_swap_half` | 293 | 571 | 555 | 6 | 174 | 65 |
| `nhrl_only` | 81 | 547 | 491 | 0 | 545 | 99 |
| `massd_only` | 0 | 411 | 144 | 3 | 48 | 24 |
| `view_pinhole` | 92 | 526 | 520 | 13 | 540 | 101 |
| `view_rectified` | 170 | 497 | 494 | 6 | 880 | 150 |
| `view_distorted` | 220 | 555 | 524 | 12 | 380 | 71 |
| `view_mixed` | 229 | 545 | 499 | 1 | 205 | 104 |
| `x_d40000_ep50` | **300** | **600** | 553 | 1 | 87 | **15** |

- **Synthetic cage data is what teaches the house bot, and `base` never learns it.** `base` finds
  0 of 512 house bots. Every arm carrying NHRL-cage domain frames finds some, topping out at
  `x_d40000_ep50`'s 300 (0.586) and `nodamage_swap_half`'s 293. The plan's count-based read
  guessed at this ("nobody finds the house bot, 264 of 650 at best") without GT; with GT the
  answer is that the best arm finds three in five, and the arm with no domain data finds none.
- **House bot recall does not track domain count.** `d2500` finds 279 and `d20000` 57;
  `d40000_real3x`, the most precise arm on opponents, finds 9. The arms that collapse toward
  conservatism drop the house bot first, which fits it being the smallest and most static thing
  in frame.
- **Mr Stabs false positives are a low-threshold problem, not a conf 0.5 problem.** The plan
  counted 120 to 137 per `s` arm at count-matched thresholds; at 0.5 the worst arm emits 14 and
  five arms emit none. It is still true that `x_d40000_ep50` is near-clean at 1.
- **`base`'s failure mode is naming, not finding.** 545 extra boxes and 277 wrong-class calls
  against `d40000_real3x`'s 19 and 14. It puts boxes on robots and calls them the wrong thing,
  which is exactly what the agnostic-vs-instance gap in the pooled run shows.

### Per recording, opponent recall at conf 0.5

Each recording is one opponent robot. The three rightmost are MassD broadcast.

| Arm | clyde | ironwarrior | johnundercut | sphinx | stingop | wreckcreat | beeroll | stardust | sirslicey |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| *GT opponents* | *41* | *97* | *75* | *147* | *44* | *98* | *10* | *49* | *48* |
| `base` | 0.93 | 0.92 | 0.99 | 0.93 | 0.45 | 0.91 | 0.20 | 0.57 | 0.94 |
| `d2500` | 0.95 | 0.98 | 0.93 | 0.96 | 0.70 | 0.89 | 0.40 | 0.88 | 1.00 |
| `d40000` | 0.93 | 0.92 | 0.91 | 0.85 | 0.59 | 0.89 | 0.50 | 0.76 | 0.98 |
| `d40000_real3x` | 0.85 | 0.91 | 0.92 | 0.90 | 0.68 | 0.87 | 0.30 | 0.69 | 0.96 |
| `swap_half` | 0.98 | 0.98 | 0.97 | 0.96 | 0.73 | 0.94 | 0.50 | 0.84 | 0.98 |
| `nodamage_swap_half` | 0.98 | 0.95 | 0.99 | 0.95 | 0.55 | 0.93 | 0.60 | 0.84 | 1.00 |
| `massd_only` | 0.02 | 0.29 | 0.11 | 0.04 | 0.00 | 0.07 | 0.60 | 0.86 | 0.96 |
| `x_d40000_ep50` | 0.93 | 0.93 | 0.89 | 0.92 | 0.82 | 0.96 | 0.40 | 0.88 | 0.96 |

- **`massd_only` is venue-locked, not broken.** It recalls 0.86 and 0.96 on two of the three MassD
  recordings and 0.00 to 0.29 on all six NHRL ones. The ZED set read this arm as broken
  everywhere, 0.077 on its own venue's frames, but those frames are the robot's onboard camera at
  the MassD event, not broadcast footage of the MassD arena. What the arm learned transfers to the
  camera position it was rendered from and not to the robot's own view. That is a viewpoint
  result the ZED set could not separate from a venue result.
- **`stingoperation` is the hard NHRL recording** for every arm: `base` 0.45 against 0.91 or better
  on four of the other five. `x_d40000_ep50` handles it best at 0.82.
- **`beeroll` carries 10 opponent boxes over 21 frames** and no arm exceeds 0.60. Per-recording
  numbers there are noise, and its frames were also the most heavily corrected.

### Our own robot and its heading, conf 0.5

`taxonomy_keypoint_ours.yaml` drops the opponent and the house bot and scores Mrs Buff alone:
629 GT boxes, 3,440 visible keypoints. This is what the aim assist consumes.

| Arm | Recall | vs `base` | Precision | Kp err px | Heading err | Heading acc@10deg |
| --- | --- | --- | --- | --- | --- | --- |
| `base` | 0.887 | | 0.939 | 4.78 | 2.89 deg | 0.968 |
| `d2500` | 0.906 | +0.019 better | 0.936 | 3.66 | 2.08 | 0.974 |
| `d5000` | 0.925 | +0.038 better | 0.934 | 3.34 | 1.76 | 0.985 |
| `d10000` | 0.927 | +0.040 better | 0.918 | 3.27 | 1.79 | 0.983 |
| `d20000` | 0.928 | +0.041 better | 0.959 | 3.43 | 1.98 | 0.981 |
| `d40000` | 0.914 | +0.027 better | **0.988** | 3.35 | 1.68 | 0.984 |
| `d40000_ep50` | 0.932 | +0.045 better | 0.918 | 3.50 | 1.92 | 0.978 |
| `d40000_real3x` | 0.921 | +0.033 better | 0.975 | 3.05 | 1.90 | 0.984 |
| `swap_half` | 0.919 | +0.032 better | 0.907 | 3.69 | 2.48 | 0.977 |
| `swap_all` | 0.905 | +0.017 ns | 0.930 | 5.69 | 3.34 | 0.958 |
| `nodamage_swap_half` | 0.908 | +0.021 better | 0.906 | 3.63 | 2.58 | 0.968 |
| `nhrl_only` | 0.871 | -0.016 ns | 0.919 | 7.37 | 3.20 | 0.962 |
| `massd_only` | 0.660 | -0.227 worse | 0.976 | 11.66 | 3.16 | 0.964 |
| `view_pinhole` | 0.846 | -0.041 worse | 0.896 | 7.84 | 4.96 | 0.944 |
| `view_rectified` | 0.817 | -0.070 worse | 0.874 | 6.62 | 3.31 | 0.943 |
| `view_distorted` | 0.893 | +0.006 ns | 0.870 | 5.31 | 3.58 | 0.966 |
| `view_mixed` | 0.871 | -0.016 ns | 0.921 | 5.40 | 3.59 | 0.945 |
| `x_d40000_ep50` | **0.954** | +0.067 better | 0.923 | **1.05** | **0.56** | **0.997** |

**Read the keypoint columns with the seeding in mind.** Every keypoint in this GT was placed by
`x_d40000_ep50` and then adjusted by hand, so that arm is being scored against its own output
wherever the labeller left a keypoint where it landed. Its 1.05 px and 0.56 degrees are not a
measurement of its accuracy, they are a measurement of how often nobody moved its keypoints. On
the ZED set, whose GT it never touched, the same model reads 5.81 px and 5.28 degrees. The other
seventeen arms are comparable with each other here; none of them is comparable with that row.

- **Heading is close to solved on this footage for every serious arm.** Twelve arms hold heading
  error under 3.6 degrees and accuracy within 10 degrees above 0.94. On the ZED set the same arms
  ran 5.3 to 8.3 degrees. Bigger robots make the front-back vector easier, and the GT keypoints
  came from a model that is good at exactly that.
- **Every `d*` arm improves keypoint error over `base` with a CI excluding zero**, by 1.1 to 1.7 px,
  and so do `swap_half`, `nodamage_swap_half` and `d40000_real3x`. This agrees with the ZED set,
  which is the useful part: it is the one family of results the two eval sets tell the same story
  about.
- **The view arms and the single-venue arms lose keypoints.** `view_pinhole` 7.84 px, `nhrl_only`
  7.37, `massd_only` 11.66, all significantly worse than `base`'s 4.78. Narrow training
  distributions cost keypoint precision on our own robot even where they cost little detection.
- **`swap_all` is the one swap arm that goes backwards**, 5.69 px against `swap_half`'s 3.69.

### Venue split, opponent detection at conf 0.5

516 NHRL cage-high frames against 120 MassD frames. Both are cage-mount broadcast views, which
is what makes this split different from the ZED set's: there, "MassD frames" were the robot's own
camera at the MassD event.

| Arm | NHRL R | NHRL P | NHRL F1 | MassD R | MassD P | MassD F1 |
| --- | --- | --- | --- | --- | --- | --- |
| `base` | 0.888 | 0.420 | 0.571 | 0.701 | 0.307 | 0.427 |
| `d2500` | 0.922 | 0.822 | 0.869 | 0.888 | 0.669 | 0.763 |
| `d5000` | 0.918 | 0.855 | 0.886 | 0.822 | 0.672 | 0.739 |
| `d10000` | 0.871 | 0.869 | 0.870 | 0.822 | 0.846 | 0.834 |
| `d20000` | 0.859 | 0.904 | 0.880 | 0.766 | 0.719 | 0.742 |
| `d40000` | 0.863 | 0.939 | 0.899 | 0.832 | 0.685 | 0.751 |
| `d40000_ep50` | 0.906 | 0.770 | 0.833 | 0.841 | 0.928 | 0.882 |
| `d40000_real3x` | 0.878 | 0.965 | **0.920** | 0.776 | **0.988** | 0.869 |
| `swap_half` | **0.942** | 0.698 | 0.802 | 0.869 | 0.809 | 0.838 |
| `swap_all` | 0.845 | 0.636 | 0.725 | 0.869 | 0.853 | 0.861 |
| `nodamage_swap_half` | 0.916 | 0.735 | 0.816 | 0.888 | 0.872 | 0.880 |
| `nhrl_only` | 0.861 | 0.532 | 0.658 | 0.570 | 0.339 | 0.425 |
| `massd_only` | 0.100 | 0.714 | 0.175 | 0.888 | 0.720 | 0.795 |
| `view_pinhole` | 0.839 | 0.456 | 0.590 | **0.925** | 0.586 | 0.717 |
| `view_rectified` | 0.799 | 0.312 | 0.449 | 0.869 | 0.705 | 0.778 |
| `view_distorted` | 0.855 | 0.558 | 0.675 | 0.888 | 0.848 | 0.868 |
| `view_mixed` | 0.797 | 0.647 | 0.714 | **0.925** | 0.712 | 0.805 |
| `x_d40000_ep50` | 0.916 | 0.911 | 0.914 | 0.879 | 0.949 | **0.913** |

- **Venue match works on this footage, which is the opposite of what the ZED set showed.** The plan
  concluded "question 2's premise fails: neither single-venue arm beats the two-venue arm on its
  own venue". Here both single-venue arms clearly prefer their own venue: `massd_only` recalls
  0.888 on MassD against 0.100 on NHRL, and `nhrl_only` 0.861 on NHRL against 0.570 on MassD. What
  changed is the camera, not the venue. The renders put the camera on a cage mount, this eval is
  cage-mount footage, and the ZED set is the robot's own view of the same arenas. Venue-specific
  synthetic data transfers to the viewpoint it was rendered from.
- **That reframes `massd_only` rather than rescuing it.** It is still the worst arm pooled, because
  it is useless on five sixths of the frames. But "20,000 MassD frames barely detect" was a
  viewpoint failure being read as a data failure.
- **`x_d40000_ep50` is the only arm above 0.9 F1 on both venues.** `d40000_real3x` leads NHRL at
  0.920 and drops to 0.869 on MassD, where its recall falls to 0.776 at near-perfect precision.
- **MassD rests on 120 frames and 107 opponent boxes.** Differences under about 0.05 F1 there are
  not worth reading, and `beeroll` alone contributes 10 of those boxes.

### Is conf 0.5 the right threshold here

The plan's next-steps list flags that 0.5 "was inherited rather than measured". Opponent F1 at
three thresholds, with each arm's best row expanded:

| Arm | F1 @0.25 | F1 @0.5 | F1 @0.75 | Best | Recall | Precision |
| --- | --- | --- | --- | --- | --- | --- |
| `base` | 0.425 | **0.544** | 0.536 | 0.50 | 0.856 | 0.399 |
| `d2500` | 0.714 | **0.849** | 0.525 | 0.50 | 0.916 | 0.791 |
| `d5000` | 0.768 | **0.858** | 0.487 | 0.50 | 0.901 | 0.819 |
| `d10000` | 0.794 | **0.863** | 0.325 | 0.50 | 0.862 | 0.865 |
| `d20000` | 0.838 | **0.855** | 0.157 | 0.50 | 0.842 | 0.868 |
| `d40000` | 0.830 | **0.870** | 0.253 | 0.50 | 0.857 | 0.883 |
| `d40000_ep50` | 0.718 | **0.840** | 0.404 | 0.50 | 0.895 | 0.792 |
| `d40000_real3x` | 0.839 | **0.911** | 0.227 | 0.50 | 0.860 | 0.969 |
| `swap_half` | 0.579 | **0.807** | 0.605 | 0.50 | 0.929 | 0.714 |
| `swap_all` | 0.565 | **0.747** | 0.584 | 0.50 | 0.849 | 0.666 |
| `nodamage_swap_half` | 0.637 | **0.826** | 0.574 | 0.50 | 0.911 | 0.755 |
| `nhrl_only` | 0.400 | **0.616** | 0.459 | 0.50 | 0.810 | 0.497 |
| `massd_only` | **0.418** | 0.358 | 0.183 | 0.25 | 0.325 | 0.586 |
| `view_pinhole` | 0.318 | **0.611** | 0.542 | 0.50 | 0.854 | 0.476 |
| `view_rectified` | 0.294 | 0.487 | **0.544** | 0.75 | 0.414 | 0.795 |
| `view_distorted` | 0.501 | **0.703** | 0.629 | 0.50 | 0.860 | 0.595 |
| `view_mixed` | 0.546 | **0.731** | 0.611 | 0.50 | 0.819 | 0.659 |
| `x_d40000_ep50` | 0.783 | **0.913** | 0.631 | 0.50 | 0.910 | 0.917 |

- **0.5 is the best of the three for sixteen of the eighteen arms**, and the two exceptions are the
  two worst arms. On this footage the threshold was inherited correctly. The ZED set pointed the
  other way, where conf 0.25 gave every arm a better F1 than 0.5, because there recall was the
  binding constraint and here precision is.
- **Raising to 0.75 breaks the domain-heavy arms specifically.** `d20000` falls from 0.855 to
  0.157 and `d40000_real3x` from 0.911 to 0.227, while `base` barely moves and `swap_half` loses
  0.2. The arms that look conservative at 0.5 are not emitting high-confidence boxes; they emit
  fewer boxes in a band just above 0.5. That is worth knowing before anyone tunes a deployment
  threshold upward on the strength of their precision.
- **The gap between thresholds is smaller than the gap between arms here**, unlike on the ZED set.
  Best-to-worst across thresholds for a fixed arm spans about 0.1 to 0.3 F1; best-to-worst across
  arms at conf 0.5 spans 0.358 to 0.913.

### Box counts on the cage-high footage, 2026-09-17

Superseded by the scoring above, and kept because it is how the pre-label threshold was picked
before the set had ground truth.

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

## What this changes

**The two eval sets disagree about the grid, and both readings are correct.** The ZED set is the
robot's own camera: robots are small, recall is the binding constraint, and `swap_half` wins by
finding opponents the conservative arms miss. This set is a cage mount: robots are 5.5 times the
area, every arm finds them, and the ranking falls out of false-positive rate instead. `d40000`
went from 0.295 recall and last place to 0.870 F1 and third; `d40000_real3x`, which the plan
recorded as no better than `base`, is the most precise arm in the experiment here.

Nothing in this changes what should ship. The robot carries a ZED, so the deployment decision
stays on the ZED set and on the trade the plan already framed: `swap_half`'s opponent recall
against `d40000_x50`'s precision and keypoints. What this eval settles is a different question the
plan also asked, which model should label cage-high footage, and there the answer is clear.

**Three findings that are new rather than a re-ranking:**

1. **Synthetic cage data is what teaches the house bot.** `base` finds 0 of 512. Every arm with
   NHRL-cage domain frames finds some, best 300. No amount of real footage in the corpus taught
   it; the rendered cage did.
2. **Venue-specific synthetic data transfers to the viewpoint it was rendered from.** `massd_only`
   recalls 0.888 on MassD cage-mount footage and 0.100 on NHRL; `nhrl_only` 0.861 and 0.570 the
   other way. The plan's "venue diversity beats venue match" came from a set whose MassD frames
   are the robot's own camera, which is not the view the MassD arena was rendered from.
3. **Conf 0.5 is right for this footage and 0.75 is not.** Sixteen of eighteen arms peak at 0.5,
   and the domain-heavy arms fall off a cliff at 0.75, which says their extra precision comes from
   emitting fewer boxes rather than more confident ones.

## What is still open

1. **Decide what ships, because the registered rule no longer picks one model.** Rule 1 takes the
   highest pooled opponent recall, which is `nodamage_swap_half` at 0.600 with `swap_half` at
   0.562. `d40000_x50` recalls 0.419 and wins everything else: precision 0.967, the best ZED F1 at
   conf 0.25 (0.726), keypoint error 5.81 px against the best `s` arm's 7.65, and the best numbers
   on cage-mount footage. Its recall deficit is not a threshold artifact, so this is a real trade
   of missed opponents against false locks, and the criterion was pre-registered before any arm
   separated the two this far.
2. **Pick the deployment confidence on the eval rather than inheriting it.** On ZED footage every
   arm scores better at conf 0.25 than at 0.5; on cage-mount footage sixteen of eighteen peak at
   0.5. The threshold belongs to the camera, and 0.5 was inherited rather than measured.
3. **The cage-high keypoint numbers need a blind hold-out before anyone quotes them.** That set
   was pre-labelled by `x_d40000_ep50` with `--holdout 0`, so its 1.05 px there is close to
   self-measurement. Ten percent labelled from empty, as the plan's step 6 specifies, is the fix.
4. **A second eval on the deployment camera.** The cage-high set answers labelling-model questions
   and deployment questions not at all, because a cage mount is not what the robot carries.
5. **Question 5 still needs labelled e-CAM25 footage, raw and rectified.** No view ranking here
   can answer it: the ZED eval is the `pinhole` view's home ground, which the plan called in
   advance.
