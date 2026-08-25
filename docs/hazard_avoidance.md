# Hazard avoidance

Two hazards, one representation. The floor hole is static in the field frame and known before the
match; the house bot is tracked live and moves. Both reach the control stack as inflated keep-out
discs on `FieldDescription.hazards`, and every consumer reduces them to a distance-to-centre test.

Implements `docs/hazard_avoidance_plan.md`. Measurements are in
`docs/experiments/control_improvement/hazard_avoidance_report.md`; the run-away overshoot
investigation that preceded it is in `run_away_overshoot_report.md`.

## Representation

```cpp
struct FieldHazard {
    Pose2D center;
    double radius = 0.0;             // already inflated
    Velocity2D velocity;             // zero for the hole, filtered track velocity for the house bot
    HazardSource source;             // STATIC | TRACKED
};
```

**Inflation happens once, at assembly, never at a consumer.** The stored radius is the raw hazard
radius plus our robot's half-diagonal plus the source's margin, so every consumer treats `radius`
as "centre distance below this is a loss" and no one re-derives clearance.

`FieldDescription` already flows through the filter, target selection, and navigation, so one
member reaches every consumer with no signature churn.

## Where hazards come from

`HazardAssembler` runs once per cycle in `ControlLoop::run_cycle`, before target selection, and
stamps the list onto the field. Two sources, deliberately different in how they decay:

| source | origin | decay |
|---|---|---|
| `STATIC` | `[field_filter] hazards_file` | never; a detector outage must not delete the hole |
| `TRACKED` | any track at `Group::NEUTRAL` | held `hazard_tracked_hold_s` past going stale, then dropped |

Nothing else has to be configured for the house bot: `YoloSegRobotBlobModel` already emits
`Label::HOUSE_BOT`, `label_group_utils.hpp` maps it to `Group::NEUTRAL`, and
`[robot_filter.label_mapping]` assigns it `NEUTRAL_ROBOT_1`. The tracking already worked; only the
consumer was missing.

## Static geometry lives in one file

```toml
# config/hazards/<name>.toml -- field-frame metres, raw geometry before inflation
[[hazards]]
kind = "hole"           # hole | wall_block
center = [-0.35, 0.2]
radius = 0.22
```

The C++ field filter reads it through `[field_filter] hazards_file` and the kinematic sim reads
the same path through `[sim] obstacles_file`. One file, so the simulated floor and the
controller's model of it cannot drift apart. A missing file throws on both sides rather than
yielding an empty list: a path typo must not silently remove the hole from one of them.

The sweep harness generates one such file per run and points both sides at it, which makes drift
structurally impossible rather than something a startup assert has to catch.

There is no equivalent file for tracked hazards, because both sides derive the radius from the
robot itself. In sim that means `[[opponents]] hazard_radius` must equal the half-diagonal of the
footprint `GroundTruthRobotFilter` reports (0.15 x 0.15 m, so 0.106); setting it larger models a
house bot the controller cannot see the true size of, and the sweep reports the scrapes.

## The four layers

Weakest to strongest. Options 1 and 2 are the floor and ceiling; 3 and 4 do the work.

**1. Keep the goal legal.** `EmptyCircleSolver` takes hazards alongside opponents as weighted
sites: a hazard contributes `distance_to_centre - radius` instead of `distance_to_centre`. The
constraint-triple enumeration is unchanged in structure -- three walls, two walls and a site, one
wall and two sites, three sites -- but every family now solves through one routine that carries
each site's radius, because two sites being equally clear is a linear equation in (cx, cy, r) once
you subtract their squared forms. With no hazards the answers and the family names are identical
to before. `clamp_to_field` in both navigations then pushes the goal out of any hazard after the
rectangle clamp. Without this, the layers below spend the whole match fighting a goal that sits
inside the hole.

**2. Last-ditch reverse.** Within `hazard_reverse_distance` of a hazard edge with the heading
pointed at it, `linear_x` is overridden to reverse. This is a stopping response, and the plant
says it will fire too late to save a full-speed approach: from `k_fwd = 4.88` m/s the stop takes
0.86 m against a 0.62 m turn-out radius. It is a backstop for what the steering layer cannot
solve, mainly a hazard that appears close because the house bot drove into our path or its track
initialised late. **If a sweep shows it firing often, that is evidence options 3 and 4 are
mistuned, not evidence this is working.**

**3. Speed cap for steering authority.** Not "slow down so you can stop", which this plant cannot
do, but "slow down only as much as the turn requires". An arc starting tangent to the current
heading deviates laterally by about `L^2 / (2r)` at along-track distance `L`, and the achievable
radius is `v / max_yaw_rate`, so clearing a hazard of effective radius `R` at range `L` needs:

```
v_cap = max_yaw_rate * L^2 / (2 R)
```

With `max_yaw_rate = 7.93` rad/s and `R = 0.4` m: `L = 1.0` m gives 9.9 m/s (above `k_fwd`, so no
cap at all), `L = 0.5` m gives 2.48 m/s, `L = 0.3` m gives 0.89 m/s. The cap stays out of the way
until a hazard is genuinely close and near the heading. `R` is scaled by how far off the heading
ray the centre sits, so a hazard well off to one side does not throttle a clean run past it, and
the cap never goes below `hazard_speed_cap_floor` -- a cap that reaches zero recreates the
stop-in-time behaviour the plant cannot deliver and strands the robot with the hazard in front of
it.

MotionProfileNavigation only. The cap is stated in m/s against a measured yaw rate, and
PursuitNavigation closes its heading loop in normalized command against an unfitted plant, so
there is no honest number to cap with there.

**4. Tangent waypoint (the primary response).** When the run from `our_pose` to the goal enters an
inflated hazard, the goal is replaced by a tangent point on the hazard's rim, choosing the side
that costs less heading change. Navigation needs no change at all: both controllers only ever
chase a `Pose2D`.

Three details that are load-bearing:

- **The tangent is taken against a circle slightly wider than the keep-out**
  (`hazard_waypoint_clearance_m`). Aiming at the keep-out rim exactly means arriving with zero
  clearance and grinding along the edge, which is what the first version did. With the margin, the
  run from the waypoint to the true goal is unblocked against the keep-out itself, so the goal
  comes back on its own instead of the robot orbiting.
- **The side latches.** Same pattern as `committed_turn_sign_`: without it the robot re-picks a
  side every tick and cuts back across the hazard it was rounding. It releases once the direct run
  is clear by `hazard_side_release_m`.
- **Tracked hazards are tested where they will be.** `hazard_prediction_horizon_s` advances a
  `TRACKED` hazard along its filtered velocity before the intersection test. The house bot moving
  across our path is the case this exists for.

Chained hazards re-test the substituted waypoint against the remaining ones, capped at
`hazard_tangent_max_iterations`. A hazard already routed around is excluded from later passes: the
new segment runs tangent to it by construction, so re-testing it only produces a substitution
loop. When it does not converge, the speed cap and the reverse backstop cover it.

## Diagnostics

- `/diagnostics` `hazards` channel, every cycle: count, per-hazard centre, radius and source, and
  the our-robot half-diagonal the inflation used. Lets a replay tell "the controller did not know"
  from "the controller knew and drove in anyway".
- Both navigations log `hazard_count`, `hazard_waypoint`, `hazard_side`, `hazard_reverse` and the
  steered target; MotionProfileNavigation adds `hazard_speed_capped`.
- `safest_point_target/solver` logs `n_hazards` alongside `n_opponents`, and the winning family
  name says whether a hazard pinned the answer (`*_hazard`, `wall_site_pair`, `site_triple`).
- `/hazard_markers` publishes one ring per hazard to the UI and the MCAP: amber for `STATIC`,
  magenta for `TRACKED`. Published even when empty, so a hazard that has aged out does not linger
  on the overlay as something the controller is still avoiding.
- The UI debug overlay draws the same rings on the camera image.

## Simulator

`[[hazards]]` geometry becomes obstacles in the kinematic sim. A `wall_block` clamps the chassis
to its boundary and kills forward speed, exactly like an arena wall; a `hole` ends the run. The
server prints one machine-readable line per episode:

```
EPISODE outcome=FELL_IN tick=14 sim_time=0.467 fell_in=1 wall_hits=0 block_hits=0 min_clearance=-0.0039
```

`sim_sweep.py` parses it into the `fell_in`, `min_clearance_m` and `block_hits` columns. A fall-in
cannot be recovered from the MCAP, because the recording simply stops there.

Opponents keep out of hazards too, for spawns and for `random_walk` targets, or a run starts with
one standing in the hole and the solver spends the match routing around a target that cannot
exist.

### Viewer

`./scripts/run_simulation.sh` opens an OpenCV window and runs until Ctrl-C. Both are config
defaults (`[viewer] enable = true`, `[sim] max_ticks = 0`), because running the sim by hand means
watching it. `sim_sweep.py` overrides both for the runs it drives, so batch sweeps stay headless
and end after `SWEEP_MAX_TICKS` ticks. When a cap does expire the server says so on stdout and
names the setting, since the C++ side only reports it as `failed to receive header`.

Sprites are named after the robot (`mrs_buff_mk3`, `mr_stabs_mk2`, `mrs_buff_mk2`, `house_bot`),
not after a role, because which one is ours is what the sim config decides. `[our_robot] sprite`
and each `[[opponents]] sprite` name a manifest entry; an opponent standing in for the house bot
sets `sprite = "house_bot"` alongside its `hazard_radius`.

The window shows: arena texture, hazard discs with
the inflated keep-out ring drawn next to the real geometry, robot sprites rotated by yaw, heading
and command vectors, clearance, and a fell-in banner. Press inside an opponent to drag it; release
resumes its configured behaviour from the drop point. Our own robot is deliberately not draggable
-- its pose is the plant's integrated state, and moving it out from under the plant desynchronises
the filter in ways that read as control bugs.

Rendering costs wall-clock time but cannot corrupt a run: the sim owns logical time and ships
`sim_time` in the response header, which the C++ side adopts through `ManualClock`. A slow render
makes the sim slower in real time and changes nothing the controller sees. Keep that property.

Sprites are rendered offline and committed; Blender is not a runtime dependency:

```bash
cd training/synthetic
blenderproc run render_sprites.py -- config_meshy_grade.toml
```

The renderer reads the same config the synthetic training data does, so the OnShape part-colour to
PBR-material mapping, the sticker textures, the upright pitch, and the per-model ground roll are
defined in exactly one place. The front direction comes from each robot's `[robots.keypoints]`
front/back pair, so the sprite and the keypoint detector agree on which end is the front by
construction. Models, textures, and HDRIs live under `training/data`, which is gitignored:
re-rendering needs a machine with the training data, but the committed PNGs are all the viewer
reads.

Two conventions live in `sprites.json` beside the PNGs -- pixels-per-metre, and the robot front
pointing +X -- so runtime rotation is a plain rotation by yaw with no per-asset offset. Get either
wrong and every sprite sits slightly off or the robots drive sideways.

## Config reference

`[field_filter]`, shared by every field filter type:

| key | default | meaning |
|---|---|---|
| `hazards_file` | `""` | shared geometry file, relative to `config/`; empty = no static hazards |
| `hazard_static_margin_m` | 0.10 | clearance added to a static hazard beyond our half-diagonal |
| `hazard_tracked_margin_m` | 0.05 | same for a hazard from a live neutral track |
| `hazard_tracked_hold_s` | 0.75 | how long a stale neutral track keeps producing a hazard |

`[navigation]`, both controllers unless noted:

| key | default | meaning |
|---|---|---|
| `hazard_tangent_enable` | true | option 4 |
| `hazard_tangent_max_iterations` | 3 | chained-hazard passes |
| `hazard_waypoint_clearance_m` | 0.06 | clearance the tangent waypoint leaves beyond the keep-out |
| `hazard_side_release_m` | 0.05 | release margin for the latched pass side |
| `hazard_speed_cap_enable` | true | option 3, MotionProfileNavigation only |
| `hazard_speed_cap_floor` | 0.35 | m/s floor under the cap, same nav only |
| `hazard_prediction_horizon_s` | 0.25 | how far ahead a TRACKED hazard is advanced |
| `hazard_reverse_distance` | 0.12 | option 2; 0 disables |
| `hazard_heading_threshold` | 1.047 | ~60 deg; heading must point at the hazard |
| `hazard_reverse_min_speed` | 0.35 | reverse floor while the backstop is active |

The tracked margin is smaller than the static one on purpose: the house bot is a hazard we must
not hit, but treating it as a large keep-out disc makes big parts of the field unreachable when it
parks near the middle.

## Open questions

- **Hole geometry.** A disc is assumed throughout. If the real hole is a long rectangle, one
  inflated disc either wastes a lot of field or fails to cover the corners. Covering it with two
  or three overlapping discs keeps every consumer unchanged and is probably enough. Decide once
  the arena dimensions are known -- which is also why no shipping config sets `hazards_file` yet.
- **House bot inflation radius.** The margin above is a starting point, not a measurement. It may
  want to be mode-dependent.
- **Whether attack should treat hazards differently from run-away.** Declining to chase an
  opponent standing next to the hole is correct; declining to chase one merely near the house bot
  may hand away the match. Today both modes get the same treatment, and the goal is pushed to the
  hazard boundary rather than abandoned, so attack closes to the nearest legal point instead of
  giving up.
