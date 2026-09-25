# Pose model size ladder: results

Results of [pose_model_size_ladder_plan_2026-09-24.md](pose_model_size_ladder_plan_2026-09-24.md).
All five `yolo26` pose sizes trained on the leak-free `d50000` arm with one schedule and one seed,
scored on the cage-high eval set in one paired bootstrap per run.

`m` fails Q2, so the ZED Box stays on `s`. Opponent recall is flat from `s` to `x` (0.936 to
0.947) and only `n` falls off it. Size buys precision and heading instead: F1 peaks at `l`,
heading error falls at every step from `n` to `x`, and only `x` is better than `s` at both venues.

## Setup

| Arm | Model | Queue job | Took |
| --- | --- | --- | --- |
| `n_d50000` | `yolo26n-pose` | 36 | 1 h 54 min |
| `s_d50000` | `yolo26s-pose` | 33 | 2 h 18 min |
| `m_d50000` | `yolo26m-pose` | 34 | 3 h 24 min |
| `l_d50000` | `yolo26l-pose` | 37 | 4 h 10 min |
| `x_d50000` | `yolo26x-pose` | 35 | 5 h 56 min |

Every arm: 68,447 frames, 30 epochs (2.05 M presentations), `imgsz 640`, batch 96 over three
A6000s, `--seed 0`, pretrained start. Weights are
`data/models/yolo26{n,s,m,l,x}-pose_d50000_2026-09-24_{epoch10,epoch20,last}.pt`, and the scored
engines are the `last_rect384x640_x86_64_sm86` builds from the same jobs.

The eval set is `training/data/cage_high_x50_conf044/d40000`: 636 hand-corrected broadcast frames,
516 from six NHRL Brettzone recordings and 120 from three MassD recordings. It replaced
`nhrl_keypoints_eval_test` on 2026-09-25, before any cage-high number was read, because the
domain renders were modelled on cage-high footage. No ZED 2i frame is scored here. `d50000` shares
none of the 636 frames; `d50000_cagehigh` holds all of them.

Every run printed `num_keypoints=2, num_classes=4` (133 engine loads) and the expected frame count:
636 pooled, 516 and 120 by venue, 320 and 316 on the halves. Conf 0.5, 1000-sample paired
bootstrap, baseline `s`. Scores are in `training/data/cage_high_x50_conf044/scores_size_ladder*/`.

Two caveats from the plan apply to every table. Recall saturates on this footage, so most of the
spread is in precision. The ground truth was seeded by `x_d40000_ep50` and corrected by hand, so
keypoint and heading numbers lean toward `x`-shaped output.

## The size curve, conf 0.5

Opponent rows use `taxonomy_opponent.yaml`; heading and keypoint error use
`taxonomy_keypoint_ours.yaml` (Mrs Buff's own keypoints). Deltas are against `s` with the 95% CI.

| Size | Precision | Recall | vs `s` | F1 | vs `s` | Heading | vs `s` | Kp err |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `n` | 0.778 | 0.877 | -0.066 [-0.091, -0.043] | 0.825 | +0.035 | 3.35 deg | +1.18 [+0.62, +1.78] | 5.15 px |
| `s` | 0.680 | 0.943 | | 0.790 | | 2.16 | | 3.59 |
| `m` | 0.753 | 0.936 | -0.007 [-0.026, +0.013] | 0.835 | +0.044 | 1.89 | -0.28 [-0.92, +0.60] | 3.02 |
| `l` | 0.825 | 0.936 | -0.007 [-0.026, +0.012] | **0.877** | +0.087 | 1.54 | -0.62 [-1.08, -0.28] | 2.77 |
| `x` | 0.789 | **0.947** | +0.005 [-0.015, +0.024] | 0.861 | +0.071 | **1.36** | -0.80 [-1.45, -0.01] | **2.63** |

By venue, opponent F1 against `s`:

| Size | NHRL (516) | vs `s` | MassD (120) | vs `s` |
| --- | --- | --- | --- | --- |
| `n` | 0.831 | +0.057 better | 0.796 | -0.086 worse |
| `s` | 0.773 | | 0.883 | |
| `m` | 0.842 | +0.069 better | 0.800 | -0.083 worse |
| `l` | **0.911** | +0.137 better | 0.741 | -0.142 worse |
| `x` | 0.848 | +0.075 better | **0.933** | +0.050 better |

- **On MassD, `m` and `l` get worse as they get bigger.** They gain precision at NHRL and lose it
  at MassD: `l` is at 0.879 precision on NHRL and 0.632 on MassD. The MassD loss comes from two
  recordings: `r2_stardust` (`m` F1 -0.160, `l` -0.211) and `r3_sirslicey` (`l` -0.135). `x` is
  the only size better than `s` at both venues.
- **`n` loses one recording outright.** On `stingoperation` it recalls 0.205 against `s`'s 0.886,
  which is most of its pooled recall deficit.
- **`s` has the lowest precision of the five,** 0.680. Its false positives are concentrated on
  `sphinx` (0.479) and `clyde` (0.412), the two recordings where `l` gains the most F1 (+0.257
  and +0.224). `m` gains on `sphinx` but not on `clyde`.

## The questions

The noise floor from the plan is 0.05 opponent recall, one seed per arm. No recall delta between
`s`, `m`, `l` and `x` reaches it.

**Q1, the size curve.** Recall does not rise with size. It is 0.936 to 0.947 from `s` to `x`, all
within 0.011, and `n` sits 0.066 below. Heading error falls at every step, 3.35, 2.16, 1.89, 1.54,
1.36 degrees, and keypoint error with it. F1 peaks at `l` (0.877), then `x` (0.861). This matches
the bbox detector's flat `s` to `l` recall in `model_size_2026-09-04.md`, with the difference
moving into precision and keypoints.

**Q2, `m` for the 60 Hz plan. Fail.** `m`'s recall is 0.936 against `s`'s 0.943, -0.007 with a
CI spanning zero. Its heading error is no worse (1.89 against 2.16, ns), but the rule needs both.
The ZED Box stays on `s`. `m` does gain 0.044 F1 over `s` with a CI excluding zero, but that is
unregistered, and it comes with a MassD F1 loss of 0.083.

**Q3, `n` as the cheap option. Fail on both counts.** Recall is 0.066 below `s` against the 0.03
allowance, and heading error is 1.18 degrees worse against the 1-degree allowance. `n` is not a
fallback.

**Q4, does `x`'s keypoint lead survive? It passes the registered test, and the plan's own caveat
discounts it.** `x_d50000`'s heading error is 1.36 degrees against `s`'s 2.16, -0.80 with a CI of
[-1.45, -0.01], which excludes zero. The plan, amended before scoring, says a lead under about
1 degree is not evidence on this set, because the ground truth started as `x_d40000_ep50` output.
0.80 degrees is under that bar. `l` holds a smaller lead with a tighter CI (-0.62, [-1.08, -0.28]),
and `l` shares no model with the seed, which makes the size effect on heading more believable
than the `x` row alone.

## Unregistered reads

### Threshold, picked on one half and reported on the other

Each recording was split in two with a fixed seed (half `a` 320 frames, half `b` 316). The conf
with the best opponent F1 on `a` was then scored on `b`.

| Size | F1 on `a` at 0.25 / 0.35 / 0.5 / 0.65 | Pick | `b` at pick, P / R / F1 | `b` at 0.5, F1 |
| --- | --- | --- | --- | --- |
| `n` | 0.644 / 0.718 / 0.822 / 0.782 | 0.5 | 0.784 / 0.876 / 0.827 | 0.827 |
| `s` | 0.596 / 0.697 / 0.784 / 0.787 | 0.65 | 0.839 / 0.781 / 0.809 | 0.796 |
| `m` | 0.738 / 0.780 / 0.839 / 0.840 | 0.65 | 0.834 / 0.837 / 0.835 | 0.831 |
| `l` | 0.785 / 0.827 / 0.871 / 0.862 | 0.5 | 0.828 / 0.944 / 0.882 | 0.882 |
| `x` | 0.714 / 0.794 / 0.867 / 0.889 | 0.65 | 0.964 / 0.876 / 0.918 | 0.856 |

- **On this footage the threshold wants to go up, the opposite of the grid's ZED finding.**
  Conf 0.25 costs every size F1 (full set: `s` 0.595, `x` 0.704), with recall 0.95 to 0.98 and
  precision 0.43 to 0.64.
- `s`, `m` and `x` picked 0.65, the top of the swept range, so their best threshold may be
  higher. `x` gains the most from moving: 0.918 held-out F1 at 0.65 against 0.856 at 0.5.
- For `s` and `m` the move to 0.65 trades 0.17 and 0.10 recall for precision and changes held-out
  F1 by 0.013 and 0.004. That is within noise.

### Grid anchors, square engines

`score_size_ladder.sh` with `SHAPE=square`, baseline `s_d50000`, the grid's three square engines
beside it. The anchors reproduce the grid's published cage-high numbers exactly.

| Candidate | Precision | Recall | vs `s` | F1 | Heading |
| --- | --- | --- | --- | --- | --- |
| `s_d50000` | 0.686 | 0.934 | | 0.791 | 2.27 deg |
| `x_d50000` | 0.793 | 0.949 | +0.015 ns | 0.864 | 1.43 |
| `swap_half` | 0.714 | 0.929 | -0.005 ns | 0.807 | 2.48 |
| `d40000_s50` | 0.792 | 0.895 | -0.039 worse | 0.840 | 1.92 |
| `x_d40000_ep50` | 0.917 | 0.910 | -0.025 worse | 0.913 | 0.56 |

- `s_d50000` and the grid's `swap_half` are indistinguishable on recall and F1 (F1 +0.016, CI
  [-0.004, +0.035]). Adding the 10,000 basement frames and 30 epochs neither helped nor hurt on
  this footage.
- `x_d40000_ep50` leads on precision and heading, but it seeded the labels. Its 0.56-degree
  heading and 1.05 px keypoint error are close to scoring its own output.

### Trained-on-test ceiling

The `_cagehigh` arms trained on all 636 eval frames.

| Candidate | Precision | Recall | F1 | Heading |
| --- | --- | --- | --- | --- |
| `s_d50000` | 0.680 | 0.943 | 0.790 | 2.16 deg |
| `s_d50000_cagehigh` | 0.993 | 0.949 | 0.971 | 2.23 |
| `x_d50000` | 0.789 | 0.947 | 0.861 | 1.36 |
| `x_d50000_cagehigh` | 0.992 | 0.987 | 0.989 | 1.13 |

Training on the eval frames almost removes false positives (precision 0.993) and leaves `s`'s
recall and heading where they were. On this set, any `_cagehigh` number is a memorization
measurement.

## What this hands the 60 Hz plan

- Its accuracy half fails. `m_d50000` does not beat `s_d50000` on opponent recall, so the ZED Box
  stays on `s`, and the live 59 Hz check on the trained `m` engine is not needed for the decision.
  It could not have run anyway: the ZED Box (192.168.50.183) was offline throughout.
- If the criterion moves to F1 or precision, `m` gains 0.044 F1 pooled and loses 0.083 on MassD.
  That decision needs a new registration and should read both venues.
- `l` has the best pooled F1 and the steadiest heading lead, but gate L already failed it on
  latency (54.7 Hz). `x` is the only size that improves on `s` at both venues, and it runs at
  33.9 Hz on the box.

## Next steps

1. Decide whether a precision-led criterion replaces recall for the ZED Box on this footage. If it
   does, register it, then run the live 59 Hz check on `m` once the box is back on the network.
2. Extend the threshold sweep above 0.65 for `s`, `m` and `x` on half `a`.
3. Find out why `m` and `l` lose MassD precision. The loss sits in `r2_stardust` and
   `r3_sirslicey`.
