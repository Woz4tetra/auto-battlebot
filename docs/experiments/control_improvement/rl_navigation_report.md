# RL navigation: toy test, environment, C++ navigation and training

Plan: `docs/plans/rl_policy_navigation_plan.md`. This report covers plan steps 1 to 8, everything
that needs no new recordings: the toy viability test, the observation spec and its parity gate,
the firmware kernels, the Warp training environment, `RlPolicyNavigation` in C++, curriculum
training and the sim gates G3, G4 and G5. Recordings (step 0), gates G1, G2 and G6 and the robot
runs wait on the test box and a second robot.

## Status

| Plan step | State | Gate |
| --- | --- | --- |
| 1 toy viability | done: yes to all four questions, PPO goes on | |
| 2 observation spec | done: `auto_battlebot/rl/obs_spec.py`, C++ `ObservationBuilderV1` | G3 builder: 24 golden cases equal to 1e-5 |
| 3 firmware on the GPU | done | per-motor RMS 9e-6% (gate 0.5%); 2 ms physics allowed (0.3 mm change, gate 2 mm) |
| 4 environment | done: `auto_battlebot/rl/envs/warp_nav.py` | 40,000 to 91,000 policy steps/s with sensing (2,048 to 8,192 worlds) |
| 5 C++ skeleton | done: `rl_policy/`, `RlPolicyNavigation` | constant bundle drives in the lockstep sim |
| 6, 7 training | done: 1.0B steps through stage D, `data/models/nav_policies/rl_nav_d_fix` | G5 split: 23x the hits per hit taken, higher danger share |
| 8 export and parity | done | G3 passes; G4 within 10% on success, strikes and danger |

## Step 1: toy viability test

`auto_battlebot/rl/envs/toy.py`: the grey-box plant from the Mr Stabs Mk2 profile in torch, a 2.5 m
box, one static opponent, discs for robots, 4,096 worlds on the laptop RTX 4080. Configs in
`playground/rl_toy/configs/`, runs in `runs/rl_toy/`, summary from
`playground/rl_toy/summarize_bakeoff.py`. Scripted baseline: `PursuitNavigation`'s control law with
the Mr Stabs profile gains (`max_linear_command` 0.25), and the same law at 1.0.

| Question | Answer | Numbers (2,048 held-out episodes, median over seeds) |
| --- | --- | --- |
| 1 PPO strikes from behind or the side in over 90% of episodes within ~30 min | yes | 99.4% by the first evaluation (31M steps, 2 min); 99.9% final (3 seeds, 200M steps, 15 min) |
| 2 beats pursuit on hits per hit taken and time to first strike | yes | 1,790 against 0.055 hits landed per hit taken; 0.67 s against 20 s (no strike) median to first strike |
| 3 still beats pursuit with the sensing model and the opponent filter mirror | yes | 100% success, 262 hits per hit taken, 0.82 s, against pursuit's 53%, 0.33 and 14.1 s |
| 4 FastTD3 against PPO | PPO | FastTD3 never struck in 3 seeds (0% at every budget); PPO 99.4% at 50M steps and 99.8% at 5 min |

Notes:

- The toy policy farms strikes: it backs off past the 0.5 s engagement gap and re-strikes, 26 to 36
  strikes per 20 s episode. The plan's "first contact per engagement" rule allows it.
- FastTD3 is our own implementation (`auto_battlebot/rl/learners/fasttd3.py`): distributional
  clipped double Q, n-step 3, per-world noise in [0.001, 0.4], no LayerNorm (RTNeural cannot run
  it). The laptop arm used batch 8,192, not the paper's 32,768, which costs 100 ms per update here.
  It converges on a smooth two-state toy (`tests/python/test_rl_core.py`), so the failure on the
  navigation toy is exploration: a deterministic actor with small noise never finds the sparse
  strike reward that PPO's stochastic policy finds in two minutes. Seed 0 ran before a uniform
  random warm-up was added; seeds 1 and 2 had it. All three stayed at zero.
- The C++ agent's builds and my smoke tests shared the box with the bake-off, so wall-clock
  numbers are pessimistic for every arm. The step-matched comparison carries the conclusion.

## Sensing model: two defects found and worked around

1. **Dropout gaps too long.** `simulation/sensing_models/provisional.toml` fits gaps between
   keypoint heading updates over whole match recordings, robots off the field included (spans hit
   its 20 s cap). Our robot's gaps came out at p90 2.15 s, p99 11 s, with 31% of the time in gaps
   over 0.5 s. The six NHRL May fights measured any-detection gaps at p50 56 ms, p90 338 ms, max
   5.8 s (`perception_reliability_may_fights.md`). The RL file
   `simulation/sensing_models/provisional_rl.toml` (written by
   `playground/rl/fit_provisional_ekf_error.py`) takes the measured gap quantiles for every cell of
   both roles. Measured runs stay as fitted, so robots read measured 83% (ours) and 70% (opponent)
   of the time against the fights' pooled 58% and 48%.
2. **No our-robot EKF error table.** `OurEkfErrorModel` needs `[robots.ours.ekf_error]`, which the
   sensing plan fits in step 5 from sessions not yet recorded. The same script fits it from the
   real EKF on the 2026-10-15 sysid sessions (AprilTag-driven) against tag truth: heading sigma
   0.023 to 0.24 rad and yaw-rate sigma 0.16 to 3.2 rad/s by speed bin, correlation times 0.4 to
   0.7 s. Position sigmas copy the measurement noise (assumed). It is a floor: live YOLO is noisier.

One bug of mine on top: both environments passed `OurEkfErrorModel` the measurement's capture
stamp, which is always a latency old, so the model never left its gap state and froze our rendered
pose. Opponent position error in the observation was 0.83 m median; with the delivery time it is
0.11 m. Every run made with the bug is in `runs/*/invalid_ekf_stamp_bug/` and none is reported
here. `test_toy_sensing_ekf_leaves_gaps` guards it.

## Steps 3 and 4: firmware kernels and the environment

- **Firmware gate:** the Warp port of the current firmware (yaw loop, mixer, sample-and-hold at
  100 Hz) against the scalar `FirmwareMixer` on 60,063 logged samples of the 2026-10-15 sessions:
  per-motor RMS difference 9e-6% of full command, max 3.6e-5% (`playground/rl/check_warp_firmware.py`).
- **Physics step:** the 2026-10-15 brake fit validated on its holdout at 1 ms and 2 ms. Median
  position error at 0.4 s is 13.3 mm and 13.0 mm, so the environment runs 2 ms. Heading error at
  0.4 s rises from 0.024 to 0.035 rad.
- **Environment:** `WarpNavEnv`, the fitted MJCF plus mocap walls (1.5 to 2.5 m arenas), up to three
  wall blocks, a house bot and a box opponent per world. Per substep, in one captured graph:
  transport delay, firmware, drivetrain delay, command shaping, opponent motion, `mjw.step`,
  contact classification (front arc, wedge at body x above 0.06 m and |y| under 0.081 m, read off
  the collision hulls). Holes end the episode on the chassis centre; flips after 0.5 s tipped.
  Sensing, the observation, rewards, curriculum and randomization run in torch on the same arrays.
  Opponents: static, straight, circle, random walk, pursuit (the fastest, shortest-delay corner held
  out), corner, and replay of eight human tracks from the NHRL May and MassD replays (two
  recordings held out).
- **Throughput** on the idle laptop RTX 4080 (`playground/rl/bench_warp_env.py`, random actions):

  | Worlds | Stage A (no sensing, no hazards) | Stage D (sensing, full scene) |
  | --- | --- | --- |
  | 2,048 | 77,000 policy steps/s | 40,000 |
  | 4,096 | 101,000 | 66,000 |
  | 8,192 | 131,000 | 91,000 |

  Training ran 4,096 worlds while sharing the GPU with evaluations, at about 110,000 steps/s in
  stage A and 41,000 in stage D. Resets and the sensing model cost as much as the physics.

## Step 5: RlPolicyNavigation in C++

Built in a worktree by a subagent and applied here. 723 C++ tests: 722 pass, 1 skipped (a
TensorRT engine test). RTNeural is pinned to 95c3c0f9, ELU and an empty activation parse natively,
and the 138-256-256-128-2 actor runs in 9.3 us median on the x86 dev box (not yet measured on the
Orin NX). Beyond the plan:

- `RobotDescription::measurement_age_s` and `FieldHazard::kind` (`HazardKind`), so the builder has
  ages and hole/wall-block kinds.
- Startup checks run in `make_navigation()`, so a bad bundle fails app start instead of the control
  thread; `opponent_mode` reaches the navigation through a config hook and anything but `KALMAN` is
  refused.
- `ConstantPolicy` is selected by a bundle with `policy_kind = "CONSTANT"`
  (`tests/rl_policy/bundles/constant_forward/`). It drives a full episode in the lockstep sim.

## Sim gates: baselines on the held-out scenarios

`playground/rl/lockstep_eval.py export` writes 128 held-out evaluation episodes from the Warp
environment (stage `EVAL`: stage D's mix on the nominal plant, held-out pursuit corner and replay
tracks); `run` plays each in the lockstep sim (`simulation/rl_eval_server.py`, the same arena,
opponents, command chain and scoring on CPU MuJoCo) against the real release binary.

| Lockstep, 109 attack episodes | Hits landed per hit taken | Strikes per episode | Hits taken per episode | Success | Danger share | Flips per episode |
| --- | --- | --- | --- | --- | --- | --- |
| `MotionProfileNavigation` | 0.146 | 0.35 | 2.39 | 28% | 0.140 | 0.94 |
| `PursuitNavigation` (desktop profile) | 0.089 | 1.02 | 11.4 | 44% | 0.106 | 0.28 |

Danger share is danger seconds over seconds alive. Both baselines land nothing on the held-out
pursuit opponents.
`MotionProfileNavigation` flips the robot in most episodes: it commands full speed into the
opponent, and the lockstep sim, like the robot since 2026-10-17, has no slew limit.

## Training

`training/rl/train_nav_policy.py`, PPO, 4,096 worlds, one seed, on the laptop RTX 4080. The
curriculum advances at 70% success over the current stage's episodes, or after 250M steps in a
stage (the fallback).

| Run | Steps | What changed | End of run, rolling |
| --- | --- | --- | --- |
| `rl_nav_ppo` | 0 to 12.7M | stage A to B at 5.7M, B to C at 12.7M | |
| `rl_nav_ppo_c3` | 12.7M to 263M | stage C plateaued at 41 to 44% success | fallback advance to D at 0.431 |
| `rl_nav_ppo_cd` | 263M to 421M | stage D | 49 to 52% success, 0.4 flips per episode |
| `rl_nav_ppo_d_ekf` | 421M to 513M | the lockstep-fitted EKF error table | 52% success |
| `rl_nav_ppo_d_fix` | 513M to 1.00B | the broadphase-bounds fix (solid blocks, corners and opponent edges) | 62% success, 0.13 to 0.23 flips, policy std 0.029 |

Every run before `d_fix` trained with the ghost walls described below. With solid walls the first
iterations of `d_fix` flipped 0.76 times per episode and succeeded 28% of the time; it recovered
within 30 minutes. Policy std fell from 0.08 to 0.029 across the two fine-tunes, so the policy has
almost stopped exploring: further training from this checkpoint needs its log-std reset or an
entropy bonus.

## Final policy: G3, G4 and G5

`rl_nav_d_fix` at 1.00B steps, exported to `data/models/nav_policies/rl_nav_d_fix`. Same 128
held-out scenarios; Warp evaluated three times because one run is not reproducible (see below).

| Attack episodes (109) | Hits per hit taken | Strikes | Hits taken | Success | Danger share | Danger s | Flips |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Warp, 3 runs | 3.3 to 5.3 | 4.56 to 5.30 | 0.91 to 1.39 | 61.5 to 62.4% | 0.214 to 0.234 | 3.6 to 4.1 | 0.15 to 0.17 |
| Lockstep C++ | 3.3 | 4.53 | 1.37 | 61.5% | 0.223 | 4.09 | 0.13 |
| `MotionProfileNavigation` | 0.146 | 0.35 | 2.39 | 28% | 0.140 | 0.57 | 0.94 |
| `PursuitNavigation` | 0.089 | 1.02 | 11.4 | 44% | 0.106 | 1.77 | 0.28 |

- **G3 passes:** the self-test's worst error is 6.3e-7 (tolerance 1e-4).
- **G4 passes on what it was built to catch.** Against the Warp mean, lockstep success is -0.5%,
  danger share +1%, danger seconds +6% and strikes -8%. Hits taken (+20%) and hits per hit taken
  (-25%) are outside 10% but inside the three Warp runs' own range, and flips (-19%) favour the
  C++ side. Run-away episodes agree too: 0.26 flips per episode in lockstep against 0.21 to 0.37.
- **G5 is split.** Hits per hit taken beats `MotionProfileNavigation` 23 times over (3.3 against
  0.146), with 62% success against 28% and a seventh of the flips. Danger fails: a 0.223 share of
  time alive against 0.140, and 4.1 s per episode against 0.57 s. Part of the per-episode gap is
  that `MotionProfileNavigation` flips in 94% of episodes and lives 4.1 s of 20; the share removes
  that and still fails, because the policy engages instead of keeping away. Against the held-out
  pursuit opponents nobody lands a hit; the policy takes 0.21 hits per episode there against
  `MotionProfileNavigation`'s 4.6.
- Fine-tuning on solid walls traded aggression for safety against the 616M-step checkpoint:
  flips 0.24 to 0.13, danger share 0.164 to 0.223, strikes about the same.

## How G4 was closed (interim policy)

Policy: the stage D checkpoint at 310M steps (`rl_nav_ppo_cd`, 2026-10-17 06:20), exported with
`playground/rl/export_checkpoint.py` to `data/models/nav_policies/rl_nav_interim`. The numbers in
this section are for that policy, before the fixes below.

- **G3 passed here too.** The binary's startup self-test runs the 16 exported observation/action
  pairs: worst error 9.0e-7 (tolerance 1e-4). The builder's 24 golden cases pass in the C++ tests.
- **The Warp evaluation is not reproducible run to run.** The same checkpoint on the same 128
  seeded scenarios gave 3.0, 5.2, 4.2 and 3.8 hits landed per hit taken, and 2.8 to 4.3 strikes
  per episode. GPU contact solving is not bit-deterministic and the episodes are chaotic, so one
  Warp run against one lockstep run cannot resolve a 10% band. Three Warp runs give the spread.

| 128 held-out scenarios, attack episodes | Hits per hit taken | Strikes | Hits taken | Success | Danger share | Flips |
| --- | --- | --- | --- | --- | --- | --- |
| Warp, 3 runs | 3.8 to 5.2 | 2.8 to 4.3 | 0.74 to 0.93 | 47 to 51% | 0.056 to 0.075 | 0.33 to 0.34 |
| Lockstep C++ | 5.9 | 3.3 | 0.57 | 45% | 0.129 | 0.45 |

Danger share is danger seconds over seconds alive. Strikes, success and hits taken land inside or
near the Warp spread. Danger share doubles and flips rise by a third; in run-away episodes the C++
policy flips in 17 of 19 against 5 to 8 in Warp, after 8 to 17 wall contacts each. G4 failed.

The cause is our robot's state estimate. Warp renders it with `OurEkfErrorModel`, whose table
(`playground/rl/fit_provisional_ekf_error.py`) compared the sysid EKF's corrected state at each
measurement stamp with tag truth. That leaves out the error of propagating to control time across
60 to 80 ms of latency, which is most of it when the robot spins at 4 rad/s. In lockstep the real
filter runs on simulated keypoints. Scored against the sim truth on the same ticks, its output is
several times worse than what Warp trained on:

| Our-robot estimate, interim policy | Heading error | Position error |
| --- | --- | --- |
| Warp (`OurEkfErrorModel`) | 4.7 deg median, p90 20 | 8 mm median |
| Lockstep C++ filter, one run-away episode | 14 to 24 deg median, p90 94 | 115 mm median |

In that episode the policy's wall rays were off by 0.28 m median and its opponent's body-frame
position by 0.54 m. Heading flips (over 135 deg) were only 3.6% of ticks, so the error is wide,
not bimodal. No lag in either direction aligns the two tracks better, and the error has no fixed
offset in the body frame (median -3 cm along, IQR 13 cm).

**Fix: refit the error table from lockstep.** `RlPolicyNavigation` now logs the state its
observation builder read (`our_x`, `our_y`, `our_yaw`, `our_speed`, `our_yaw_rate`), and
`lockstep_eval.py run --record <dir>` keeps an MCAP per episode.
`fit_provisional_ekf_error.py --lockstep <dir>` scores that state against the sim truth of the
same tick, gap ticks left out, and writes the table into `provisional_rl.toml`. Fit on 64 episodes
of a separate scenario set (seed 7), 53 with enough fresh ticks:

| Sigma by speed bin (0 to 1.5 m/s) | sysid table | Lockstep fit |
| --- | --- | --- |
| along, cross (m) | 0.005 to 0.012 (assumed) | 0.022 to 0.049 |
| heading (rad) | 0.023 to 0.24 | 0.22 to 0.31 |
| forward speed (m/s) | 0.057 to 0.79 | 0.20 to 0.58 |
| yaw rate (rad/s) | 0.16 to 3.2 | 2.4 to 5.3 |

Correlation times for heading and yaw rate fell from 0.6 and 0.7 s to 37 and 26 ms: the C++
filter's heading and yaw rate jump from frame to frame instead of drifting. Binned by yaw rate
rather than speed, the heading sigma runs 0.14 to 0.40 rad, so spinning costs the policy its
heading; under this policy speed and spin rise together and the speed bins carry most of it.
With the new table the Warp environment renders our heading at 12.0 deg median error and position
at 44 mm, against the lockstep filter's 10.3 deg and 47 mm.

Scored in Warp under the new table, the interim policy barely changed (3.6 hits per hit taken,
danger share 0.072, flips 0.30), so the estimate error alone does not explain the G4 gap. Two
more differences turned up:

1. **Opponent size.** The sim keypoint model sends front and back keypoints and no box, so the C++
   filter sized the opponent from the keypoint spacing: the policy read a 0.07 m half-diagonal
   against Warp's 0.17 m (the opponent's footprint, which a detector box also reports).
   `lockstep_eval.py` now passes the scenario's opponent size as the `OPPONENT` entry of
   `robot_size_meters_per_label`. Danger share fell from 0.129 to 0.102; flips rose from 0.45 to
   0.56. `MotionProfileNavigation` and pursuit score identically with and without it, so they do
   not read the opponent's size.
2. **Ghost walls in the Warp environment (my bug).** On the same 48 exported worlds, with the same
   saturated constant command for 1.5 s, the Warp environment tipped the robot far less often
   than the lockstep world (CPU MuJoCo). Tracing a world where they split, the CPU robot hit a wall
   block at 0.48 s and the Warp robot drove through the same spot with no contact. The environment
   resizes arena walls, wall blocks and the opponent per world through a batched `geom_size`, but
   MuJoCo Warp culls contact pairs with `geom_rbound` and `geom_aabb`, which stayed at the MJCF's
   compile-time placeholders: block radius 0.1 m against 0.11 to 0.29 m in use, wall half-length
   1.0 m against up to 1.3 m, and the nominal opponent box against sizes up to 1.2x. The outer ring
   of every larger block, the corners of larger arenas and the edges of larger opponents were
   passable. Every policy so far trained in that world. The two bound fields are now batched and
   recomputed from the sizes after each reset (`_sync_bounds`), with a test that compares them with
   what MuJoCo compiles.

   | Tipped within 1.5 s, command (linear, angular) | Warp before | Warp fixed | CPU, 1 ms | CPU, 2 ms |
   | --- | --- | --- | --- | --- |
   | -0.8, 0 | 0.52 | 0.81 | 0.77 | 0.73 |
   | -1.0, 0 | 0.58 | 0.90 | 0.90 | 0.81 |
   | +1.0, +0.6 | 0.02 | 0.02 | 0.15 | 0.06 |
   | +0.5, +0.3 | 0.02 | 0.02 | 0.25 | 0.23 |

   Reversing into walls now matches. Forward turns into a wall still tip less in Warp. The worlds
   rest identically (same contacts, same pitch to 1e-7 rad), pure linear and pure angular commands
   track tick for tick, ten times the solver iterations change nothing, and CPU at 2 ms agrees with
   CPU at 1 ms. In the one forward-turn world I traced, the two robots met the wall at slightly
   different angles after 0.3 s of driving: Warp with the hull, which pressed it nose-down, and CPU
   with a wheel, which climbed the wall and tipped. That residue is open.

Whether the real robot's filter is this noisy is open: the lockstep sim feeds it the provisional
sensing model's keypoints, not live YOLO. A yaw-rate sigma of 5 rad/s from a filter fed a heading
measurement every 20 ms tick looks high, and is worth checking against a real recording when the
test-box sessions exist.

## Crowded arenas stall the policy

Setting up `simulation/sim_mr_stabs_mk2.toml` as a training-like world turned up a weakness. In a
2.4 m arena with the opponent random-walking at 0.7 m/s, the stage D fine-tune at 616M steps
(lockstep, 4 to 8 episodes per layout) struck:

| Layout | Strikes per 20 s episode |
| --- | --- |
| No hazards | 13 to 17 |
| One wall block and one hole | 1 to 6 |
| One hole and the house bot | 0 to 5 |
| Two wall blocks, a hole and the house bot | 0 in every episode |
| One wall block, a hole and the house bot (house parked far from our start) | 0 to 2 |

Every hazard costs aggression, and the house bot costs the most. All of these layouts are inside
stage D's distribution (0 to 3 discs, a house bot in half the worlds), so this is the policy, not
a sim mismatch: these numbers come from the lockstep evaluation world (`lockstep_eval.py` on
hand-built scenarios), which reproduces the training environment, and the interactive sim stalled
the same way. The held-out suite averages it out (60% success). Score the policy per hazard count once the scorer
exists (plan section 13); if the dense cells stay near zero, weight dense worlds up in stage D.

## Standoff in the opponent's front arc

In the interactive sim, Mr Stabs stopped when a far-away opponent faced it and only moved once the
opponent turned. Lockstep, `rl_nav_d_fix`, a static opponent, 3 episodes per case:

| Static opponent at 0.6 to 2.0 m | Moved in the first 3 s | Strikes in 20 s |
| --- | --- | --- |
| Facing Mr Stabs | 0.01 to 0.07 m | 0 in all 12 episodes |
| Facing away | 1.6 to 3.6 m | 7 to 25, first at 0.5 to 4.4 s |

At 1.5 m, turning the opponent 5, 15 or 30 degrees away, or offsetting it sideways by 0.1 or
0.3 m, changed nothing; at 60 degrees, the edge of its 60 degree front arc, the policy struck
within 1.9 s. So the policy parks whenever it is inside the opponent's front arc and outside the
danger zone (0.37 m). Closer in, the danger term (-1 per s) moves it, which is why it backs off and
goes around at short range.

Training made parking free and usually right. Nothing charged for time, the danger zone ends at
0.37 m, and six of the seven opponent behaviours move, so waiting for one to turn its side
usually paid; a static opponent facing us is about 5% of episodes. Circling a facing opponent at
1 to 2 m takes about 2 s, the discount's horizon (0.99 per 20 ms), so the strike at the end was
mostly discounted away. With the policy's action std at 0.029 it no longer explores the detour.

A standing-still penalty (-0.5 per s under 0.2 m/s, out of contact) is now in both the Warp
environment and `LockstepWorld`, not yet trained with. On the cases above, parking now scores
19.8 to 20.0 s still; striking runs score 1.7 to 4.1 s. The logged episode reward of a parked
robot is still positive (+12.4 at 2.0 m): with gamma under 1 the approach shaping pays
`(1 - gamma) * distance` per step for standing still far away. That cancels in the discounted
return (Ng et al. 1999), so learning is unaffected, but it makes the summed episode reward a poor
metric. Acceptance after retraining: a head-on static opponent at 1 to 2 m gets struck within 5 s.

## Pursuit: a chattering opponent and a free standoff

Two defects showed in the interactive sim's trained pursuit opponent. Both were in training too.

- **The pursuer's yaw chattered.** `opponents.py` turned it at `clamp(err / dt, -1, 1) *
  turn_rate`, which saturates past 1 degree of heading error at a 20 ms step, so it turned at a
  full 4 to 10 rad/s every step and reversed direction on 90% of steps. It now turns at
  `clamp(err / dt, -turn_rate, turn_rate)`: on the 19 held-out attacking pursuit episodes its
  median yaw rate fell from 5.1 to 0.3 rad/s.
- **Freezing in front of a pursuer cost almost nothing.** `rl_nav_d_fix` against those 19
  episodes: the pursuer arrives within 0.1 to 0.9 s, then sits at 0.31 m for the rest of the
  episode; Mr Stabs stands still for 19.3 s of 20, lands nothing and takes 4 hits in total. A hit
  only counted on first arc contact, and moving away let the pursuer touch again at -10 each.

Changes, not yet trained with: continued arc contact charges another hit every 0.5 s
(`arc_rehit_s`), and the standing-still penalty now exempts only wedge pushing from outside the
opponent's arc. Tests cover both.

The re-hit did not fire against pursuit at first: the no-push rule stopped an opponent's closing
motion at 0.33 m between centres (both footprint half-diagonals plus 6 cm), short of the 0.18 m
at which the two touch nose to nose, so the pursuer hovered 15 cm off our nose. The rule now
stops it at contact: the two footprint boxes' extents along the line between centres, with our
heading added to what the opponent tracks. Its closing speed is capped to reach that distance and
no further.

| `rl_nav_d_fix` against the 19 held-out pursuit episodes (lockstep) | Before | After |
| --- | --- | --- |
| Hits taken, total | 4 | 546 |
| Danger s per episode, median | 19.4 | 1.5 |
| Seconds standing still, median | 19.3 | 14.5 |
| Flips, total | 0 | 5 |

The frozen policy now pays for being pinned. The cost is flips: an idle robot against pursuit
(stage C, 2,000 steps x 1,024 worlds) flips 0.06 times per episode and takes 7 hits. The boxes'
corners meet before their centre-line extents do, so an off-axis pursuer still overlaps our
footprint and pries it; stopping 5 mm short instead (0.069 flips) or pressing 5 mm in (0.093)
barely changes that. Fixing it needs the true box-to-box distance (separating axes) instead of
the centre-line extents.

## Flip risk without the slew limit

From a standstill in open floor, the fitted sim robot wheelies over backwards on a held forward
step of 0.8. A step of 0.6 lifts the nose 72 degrees with the fit's armature (1.10) and 10 degrees
with the other restart's (0.51), which fit the data equally well (loss 54.4 against 54.8), so the
lift threshold sits inside what the sysid could not identify. Reverse steps up to 0.7 move the
pitch by about 1 degree in open floor; reversing into a wall flips the robot over its nose. The
removed limit was there for this ("caps linear accel at 8.5 m/s^2, slip/flip limit"), and the
plant check found the real robot lifts its nose more often than the fit does. With the limit gone
from the firmware and every profile, a navigation that steps its command to 0.8 or more can flip
Mr Stabs. A step-response test on the robot (0.5, 0.6, 0.7 forward from rest, on a soft surface)
would pin the armature and the threshold.

In training, flips were the largest loss: 0.4 to 0.5 per stage C episode, split between forward
wheelies and backing into walls at speed after a strike. Two of the causes were the environment's,
not the policy's, and are fixed: a mocap opponent driving into the wedge pried an idle robot over
(66 flips in 80 early-ending episodes with zero actions; now none), and a command-change weight too
small to stop the policy chattering.
