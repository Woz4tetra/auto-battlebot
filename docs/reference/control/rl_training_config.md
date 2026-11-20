# RL training config fields

`auto_battlebot/rl/envs/warp_nav.py`, `policy_opponents.py`, `learners/fasttd3.py` and `sidestep.py` define the dataclasses below. `training/rl/train_nav_policy.py` builds them from the run TOML. Curriculum and self-play are in `docs/reference/control/rl_training.md`.

## `PolicyDriver`

| Field | Meaning |
| --- | --- |
| `robot` | Footprint key into `warp_nav.ROBOT_SIZE_M`. |
| `plant_file` | TOML with a `[plant]` table, repo-relative. |
| `checkpoint` | Training checkpoint that drives the opponent, repo-relative. `""` means self-play. |

## `Stage`

| Field | Meaning |
| --- | --- |
| `hazards` | Most holes and wall blocks per arena. |
| `random` | Domain randomization. 0 is the nominal plant, 1 the full ranges. |
| `arena` | Range of the arena side length, m. |
| `run_away` | Fraction of episodes that are `RUN_AWAY`. |
| `house` | Probability of a house bot. |
| `policy` | Share of opponents driven by a policy, for robots that have drivers. Default 0.0. |

## `WarpNavConfig`

| Field | Meaning |
| --- | --- |
| `robot` | A key of `ROBOTS`. Default `mr_stabs_mk2`. |
| `fit_file` | Plant fit file. Empty means the robot's own (`RobotProfile.fit_file`). |
| `sensing_model` | Sensing model file. Empty means the robot's own (`RobotProfile.sensing_model`). |
| `hole_margin_m` | `reward["hole_near_per_s"]` ramps from 0 at this distance outside a hole's edge to full weight at the edge. The policy learns to keep clear before the terminal hole penalty. |
| `still_speed_mps` | Under this body speed, the robot counts as standing still and pays `reward["still_per_s"]`. Spinning in place still counts, because a robot turning on the spot is as easy to hit as a parked one. Exempt: our wedge on the opponent from outside its arc. Pushing or pinning it from there is slow on purpose. Any other contact is not exempt, because pinned in its arc is the worst place to sit. |
| `arc_rehit_s` | Continued contact inside the opponent's arc is charged as another hit taken every this many seconds. A weapon keeps hitting a robot pinned against it. |
| `holdout` | Evaluation: draw only the held-out pursuit corner and replay tracks. |

### Reward keys

| Key | Meaning |
| --- | --- |
| `hole_near_per_s` | Shaping ahead of the terminal hole penalty, scaled by `hole_margin_m`. Off by default (0.0). |
| `still_per_s` | Cost per second standing still, as defined by `still_speed_mps`. A moving target beats a parked one even when moving does not lead to a strike. 20 s parked costs as much as one hit taken. Applies in both modes. |
| `command_change` | Penalty on the change of the linear command per step, to stop chatter. Chatter spins the rotors up and down, and their reaction lifts the chassis, which no slew limit stops. |
| `flank` | Weight of the flank term in the attack potential (`_potential`): being beside or behind the opponent at close range. A pursuer turns to keep us in its arc, so the point-behind distance alone barely changes while we circle it. |

Why these values: `docs/experiments/control/2026-10-17_rl_reward_weights.md`.

## `FastTd3Config`

| Field | Meaning |
| --- | --- |
| `buffer_steps` | Replay buffer length per world. |
| `batch_size` | Update batch. The paper uses 32768. The laptop arm runs 8192, see `docs/experiments/control/2026-10-17_fasttd3_batch_size.md`. |
| `learning_starts` | Buffer steps of uniform-random actions before updates. |

## `SidestepConfig`

`auto_battlebot/rl/sidestep.py` is the scripted controller that beats a pursuer by out-turning it. Tuning measurements: `docs/experiments/control/2026-10-17_sidestep_tuning.md`.

| Field | Meaning |
| --- | --- |
| `orbit_m` | Radius we circle the opponent at while working round to its rear. |
| `orbit_lead_deg` | Aim this far further round the circle. 90 is the tangent. |
| `attack_deg` | Ram once our bearing from its heading is past this. |
| `gain` | Angular action per radian of heading error. |
| `max_angular` | Angular action cap. 0.5 is about 19 rad/s. Full angular spins the chassis over. |
| `lead_s` | Heading error is taken this far ahead at the current yaw rate. |
| `max_linear` | Linear action cap. |
| `max_lateral_mps2` | Linear action is capped so that speed times yaw rate stays under this, in m/s^2. Harder turns roll the robot over. |
| `slow_m`, `strike_linear` | Inside `slow_m` of touching, linear action is capped at `strike_linear`. Ramming the immovable opponent at 2.6 m/s stopped it in one step and pitched it over its nose. |
| `contact_m` | Gap at which we count as touching and back off. |
| `back_off_s` | Reverse this long after touching. Off by default (0.0). Against a pursuer, backing straight out stays in its arc and it pins us. |
| `linear_slew`, `angular_slew` | Most each action may change per 20 ms step. 0.1 for both. |
| `back_off_linear` | Linear action while backing off after contact. |
| `switch_deg` | Change orbit direction only once this far over the centre line. |
