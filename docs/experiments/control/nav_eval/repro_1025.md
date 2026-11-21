# 10/25 reverse freezes in lockstep

Nav scoring plan section 0.1 (`docs/plans/nav_scoring_and_rl_v2_plan.md`). Run 2026-11-14.

At the plan's 0.48 guard scale the freeze does not reproduce: the arm with both stages delivers
-25% median wheel command in its reverse windows against the field's -12.5%. At 0.30, the guard
scale the field actually ran inside its reverse windows, it does: both stages give -14.4% (95% CI
-15.4 to -13.5), inside the field's CI (-22.2 to -8.8), with 28 windows against the field's 28.
The guard stage carries most of the drop. The slew limit adds little to the wheel command but,
on top of a tight guard, quadruples how often the policy ends up in sustained reverse (28 windows
against 7). Attribution: the motor guard is the main cause, the slew limit a secondary one. Both
come off in phase 1 (1.1, 1.2), so the session should not need another look first.

## Statistic

A reverse window is a run of policy ticks where the trailing 0.5 s mean of the policy's linear
command is below -0.5, lasting at least 0.5 s, with autonomy on. The number is the median wheel
command (mean of the firmware's `left_cmd` and `right_cmd`, percent, positive forward) over every
sample inside those windows, pooled. The 95% CI resamples windows (2000 draws).

The plan's description ("below -0.5 for over 0.5 s") left the window rule open. This rule is the
one that matches the plan's figures: it gives 28 windows and 20.1 s (plan: 27 and 18.8 s), -12.5%
duty (plan: -12%), and -0.62 commanded against -0.38 sent (plan: -0.60 and -0.33). A per-tick
threshold instead gives 5 windows.

## Arms

`rl_nav_p4` in lockstep on 128 held-out evaluation scenarios (`lockstep_eval.py export --count
128 --seed 2026`, stage EVAL, arenas 1.5-2.5 m), the same scenarios in every arm. The slew limit
is the host's `EspNowTransmitter` limiter at the 10/25 profile's 2000 RPM/s over 1500 RPM, 1.33
per second. The guard is a fixed scale on both wheel commands after the firmware mixer. Both stages
live in `auto_battlebot/rl/lockstep_world.py` behind the scenario keys `slew_per_s` and
`guard_scale`.

| Arm | Slew (/s) | Guard scale | Episodes | Windows | Window s | Median wheel % (95% CI) | Mean commanded | Mean sent | Chatter |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| field 10/25 | 1.33 | varies | 3 recordings | 28 | 20.1 | -12.5 (-22.2 to -8.8) | -0.62 | -0.38 | 0.245 |
| both | 1.33 | 0.48 | 128 | 8 | 7.6 | -25.1 (-27.2 to -22.3) | -0.57 | -0.59 | 0.143 |
| slew | 1.33 | 1.0 | 128 | 10 | 13.0 | -52.1 (-57.0 to -48.2) | -0.60 | -0.60 | 0.179 |
| guard | 0.00 | 0.48 | 128 | 7 | 7.8 | -26.3 (-27.1 to -24.2) | -0.62 | -0.62 | 0.142 |
| neither | 0.00 | 1.0 | 128 | 10 | 8.3 | -51.1 (-56.4 to -45.9) | -0.55 | -0.55 | 0.114 |

Chatter is the commanded linear's mean change per policy tick inside the windows.

At the field's in-window guard scale (same scenarios, both and guard arms only):

| Arm | Slew (/s) | Guard scale | Episodes | Windows | Window s | Median wheel % (95% CI) | Mean commanded | Mean sent | Chatter |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| both | 1.33 | 0.30 | 128 | 28 | 29.0 | -14.4 (-15.4 to -13.5) | -0.59 | -0.57 | 0.147 |
| guard | 0.00 | 0.30 | 128 | 7 | 4.4 | -16.6 (-17.1 to -16.2) | -0.54 | -0.54 | 0.108 |

## Findings

- **Pass condition met at the field's guard scale.** The arm with neither stage (-51%) is far from
  the field. The arm with both lands inside the field CI at 0.30 (-14.4%) and outside it at the
  plan's 0.48 (-25%). The guard-only arm at 0.30 (-16.6%) also sits inside the field CI, near its
  edge, so the guard alone gets most of the way.
- **The guard is the stage that matters in the sim.** A 0.48 scale halves the wheel command, with
  or without the slew limit.
- **The slew limit binds in the sim but not inside its reverse windows.** Sent differs from
  commanded on 76% of all ticks in the slew arms (largest step 0.0267 per tick, 1.33/s x 20 ms),
  yet the sim's reverse windows are held reverses that the limiter has caught up with. The field's
  windows chatter (0.245 per tick against 0.11-0.18), so the field limiter never caught up.
- **The field guard was tighter than 0.48 inside the windows.** Median guard scale inside the
  windows was 1.0 on 22-09 (6 s of windows), 0.26 on 00-28 and 0.32 on 00-49. The plan reports a
  median of 0.48 inside its own reverse windows; on this window rule the two sessions that limited
  sat near 0.3, and 0.48 is too loose to reproduce the field number.
- **Slew plus guard makes the policy reverse more.** At guard 0.30, adding the slew limit moves the
  windows from 7 to 28 and from 4.4 s to 29.0 s. The policy keeps asking for reverse while the
  robot does not move, the same loop the field showed.
- **The sim policy rarely reverses hard.** 7-10 windows in 2560 s of episodes against 28 in about
  367 s of field autonomy. The field's 1.5 m box with the 2x4 edge gap (plan, "Edge gap") puts the
  robot against walls far more than the sim's 1.5-2.5 m flat-walled arenas.
- **Reverse flips the fitted robot.** Held -0.9 reverse on open floor tips the sim robot over its
  nose within about 0.6 s (`test_lockstep_slew_and_guard_stages`), which is part of why the sim
  sees so few long reverse windows: 22 of 128 episodes in the slew and neither arms end FLIPPED.

## Reproduce

```bash
source scripts/activate_python.sh
D=/tmp/repro_1025
python -m auto_battlebot.nav.repro_1025 field --out-dir $D
python -m auto_battlebot.rl.lockstep_eval export --count 128 --seed 2026 --out $D/scenarios.json
python -m auto_battlebot.nav.repro_1025 sim $D/scenarios.json --out-dir $D
python -m auto_battlebot.nav.repro_1025 report --out-dir $D
```

Each arm takes about 200 s on the laptop with 6 workers.

## Next steps

- Check why the field policy chatters in reverse: replay the 10/25 observations through the bundle
  offline and compare the action sequence with the sim's on similar wall-near states.
- Rerun the arms in a fixed 1.5 m arena once the edge gap (plan 5.5) exists, since the field's
  reverse windows are wall-driven.
- Phase 1 as planned (slew off, guard loosened, thermal trip) removes both stages. The field's
  extra chatter and the wall-driven reverse are still unexplained; the edge gap (5.5) covers the
  second.
- `--guard-scale 0.30 --arms both guard` reproduces the second table.
