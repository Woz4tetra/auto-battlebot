# RL environment opponents and kernels

Code: `auto_battlebot/rl/envs/opponents.py`, `policy_opponents.py`, `trained_opponent.py`, `toy.py`, `warp_kernels.py`.

## Scripted opponents

- Behaviours: static, straight, circle, random walk, pursuit, corner, replay. All are kinematic mocap bodies driven in torch, batched over worlds.
- `OpponentParams` columns are `[W]` each. `behaviour`, `delay_steps` and `corner` are long tensors, `corner` is 0..3, `direction` is +1 or -1. The constructor rejects anything else.
- `grip` is the pursuit traction in m/s^2. `None` means unlimited, which is what scenarios saved before grip existed load as. The key stays `grip` in stored scenario JSON.
- `history` holds our pose per delay slot, columns `HIST_X, HIST_Y, HIST_VX, HIST_VY, HIST_YAW`.
- `pursuit_fixed` is `[W, 3]` with columns `PURSUIT_COL_TOP_SPEED, PURSUIT_COL_TURN_RATE, PURSUIT_COL_DELAY`. It pins the pursuit draw per world.
- Grip is drawn last in `reset`, so every other draw matches what a seed gave before grip existed.

## Pursuit curriculum and holdout

- `pursuit_level` 0 draws the easy end `PURSUIT_EASY`, level 1 the hard end `PURSUIT_HARD`.
- The delay shrinks as the pursuer gets harder, so its draw uses `1 - u`. At level 1 the three draws equal what a seed made before the curriculum existed.
- The evaluation grid is `PURSUIT_GRID`: 5 speeds x 4 turn rates x 4 delays, 80 cells.
- The held-out corner is top speed above `PURSUIT_HOLDOUT_SPEED` with delay below `PURSUIT_HOLDOUT_DELAY`. `reset(holdout=True)` draws only that corner. Training draws redraw the delay when they land in it, so evaluation never scores starts the policy trained on.

## Wall and contact rules

`Opponents.command` returns world-frame `(vx, vy, yaw rate)` per world.

- `footprint_half_diagonal` keeps the opponent off the walls.
- `opp_half_length_width` is `[W, 2]`: half length along its heading, half width across. It stops the opponent at contact with us. Without it the opponent is a square with that half-diagonal.
- `external_drive` is `(mask [W], velocity [W, 3])`. Worlds in the mask take that velocity instead of the scripted one (policy opponents from `policy_opponents.py`), under the same wall and contact rules.
- No pushing through us: the opponent is a mocap body with infinite mass. Driven into our wedge it cannot ride up it like a real robot, so it pries our chassis over. Its closing speed along the line between centres is capped so it arrives at contact (`CONTACT_PRESS_M`) and goes no further. It still turns and slides along us.
- Contact is the extent of the two footprint boxes along the line between centres (`box_support`), so a pursuer reaches our nose and its arc touches.
- Measurement behind the rule: `docs/experiments/control/2026-10-17_opponent_no_push_through.md`.

## Policy opponents

`PolicyOpponents.command` takes the observation input from the opponent's side: its pose as `our_*`, ours as `opp_*`. It fills the `hist_*` fields itself. `warp_nav._policy_opponent_command` builds that input, and `test_policy_opponent_input_is_perspective_swapped` pins it.

## Trained opponent

`TrainedOpponent` takes `opp_half_length_width`, the opponent footprint as half length along its heading and half width. It keeps the opponent off the walls and stops it at contact with us.

## Toy environment sensing

- Our EKF error model starts a measurement gap from the delivery stamp: capture stamp plus latency. The capture stamp alone is always one latency old and never leaves the gap.
- A world with no delivered frame this tick leaves the opponent filter's `output_stale` untouched, as the C++ filter does.

## Warp kernels

- The firmware path is `auto_battlebot/mujoco_sim/firmware.py` (`legacy=False`, auto steer on) ported line for line, so its constants are the C++ `yaw_control::Config` and mixer constants. `firmware_test` runs it alone for comparison with the scalar `FirmwareMixer`.
- `direct` = 1 skips the firmware: left = linear - angular, right = linear + angular, in percent, the radio's mix. Positive angular turns counterclockwise, as the firmware's does.
- CRSF percents: the firmware negates channel 0 into `a_percent` and the transmitter reverses the angular channel (`closed_loop.py`). `test_warp_command_chain_signs_match_the_app` drives the env and checks forward and counterclockwise.
- `shaping` columns are the `SHAPE_*` constants: deadzone left and right, curve, gain left and right, pack volts, reverse gain left and right, coast flag.
- `weapon_sweep_hit` samples the weapon's sweep cylinder (axis along body y) at the rim and half radius on both faces and the mid-plane, plus the axis. A point within `WEAPON_SWEEP_MARGIN_M` of the opponent's box counts as the weapon touching it. The sweep vec4 is `SWEEP_CENTRE_X, SWEEP_CENTRE_Z, SWEEP_RADIUS, SWEEP_HALF_WIDTH` in the body frame.
- `classify_contacts` with `WEAPON_WEDGE` counts a contact point ahead of `strike_x` and within `strike_half_width` of the centre line as on our wedge. `WEAPON_DISK` counts any opponent contact while the sweep touches its box.
- Geom roles are `GEOM_ROLE_*`: other, ours, opponent, wall, wall block, house bot.
- Flag columns are `FLAG_*`: any opponent contact, contact inside the opponent's arc, wedge contact outside the arc, largest `|phi|` of such a contact, wall, wall block, house bot.

## Torch plant

`auto_battlebot/rl/torch_plant.py` ports `auto_battlebot/control/plant.py` (the reference) to torch. It steps one substep for every world at once with per-world parameters, so domain randomization is a tensor draw. `test_torch_plant_matches_numpy` pins it to the reference. The command path in front of it mirrors `DifferentialDriveProcessor::process` without the channel reversal: angular keeps priority inside `|linear| + |angular| <= limit`, then each wheel loses its zero deadzone (`test_saturate_gives_angular_priority_and_wheel_deadzone`).

## Scripted pursuit policy

`auto_battlebot/rl/policies/scripted.py` registers `pursuit`, the toy's scripted baseline. It reads the opponent's body-frame position from the observation and steers at it with the same PD heading loop, angle threshold, distance ramp, turn scale and steer brake as `src/navigation/pursuit_navigation.cpp`. `PursuitConfig` defaults are the gains the Mr Stabs Mk2 bench profile ran before it moved to `MotionProfileNavigation`, except `max_linear_command`, which the evaluation sets per arm because 0.25 is a bench setting, not a fighting one.

## Environment internals

- **Substep order and graph capture.** One physics substep runs `command_path` (transport delay, firmware mixer yaw loop, drivetrain delay, command shaping into wheel volts), `move_mocap` (the kinematic opponent and house bot), `mjw.step`, `classify_contacts`, `advance`. The environment captures the substeps into one CUDA graph.
- **Mixer paths by robot.** The firmware path ports `auto_battlebot/mujoco_sim/firmware.py` (`legacy=False`, auto steer on) line by line, and `firmware_test` runs it alone to compare against the scalar `FirmwareMixer`. That is Mr Stabs Mk2's chain. Mrs Buff Mk3 has no onboard controller: the radio mixes linear and angular straight into her ESCs (`direct`), so she runs open loop with no eRPM feedback.
- **Strike geometry.** Mr Stabs Mk2's strikes land on her wedge, modelled as a box at the nose. Mrs Buff Mk3's land on her weapon disk's sweep, a cylinder, because the disk fills it as it spins. `weapon_sweep_hit` tests it against the opponent's box sample points over that cylinder.
