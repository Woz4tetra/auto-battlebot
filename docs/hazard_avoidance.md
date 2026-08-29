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

Write the path repo-relative, extension included:

```toml
hazards_file = "config/hazards/hole_and_block.toml"
```

The two configs that name it share no directory, so neither can own the base. Each side resolves
against the project root itself. Unlike `extends`, this is a plain file path with no implied
directory and no implied extension.

The sweep harness generates one such file per run and points both sides at it, which makes drift
structurally impossible rather than something a startup assert has to catch.

There is no equivalent file for tracked hazards, because both sides derive the radius from the
robot itself. In sim that means `[[opponents]] hazard_radius` must equal the half-diagonal of the
footprint `GroundTruthRobotFilter` reports (0.15 x 0.15 m, so 0.106); setting it larger models a
house bot the controller cannot see the true size of, and the sweep reports the scrapes.

## The four layers

Two aim layers and one speed rule. The aim layers (1 and 4) plan against the fat
`inflated_radius`; the velocity barrier (3) enforces speed against the thin `hard_radius`. The
fixed reverse (2) survives only on PursuitNavigation, which has no plant fit for the barrier.

This is the second design. The first shipped four layers -- goal legality, tangent, a steering
speed cap, a stop-at-rim brake, plus the fixed reverse -- and the 2026-08-28 AER recording
(auto window 21:03-21:04 EDT) showed the speed side losing anyway: 24.6% of the window inside a
keep-out across 32 entries. Three mechanisms did the damage: a blocked push spun the wheels to
~4.5 m/s while the chassis crawled, so the stored speed arrived all at once when the opponent
gave way; the angle gate zeroes `linear_x` when the target falls behind, so a v_ref-side brake
never reached the plant during exactly the coast it existed for; and the fixed reverses (hazard
and wall) backed the robot into the other hole with no rear check. The barrier replaces the
cap, the brake, and the motion profile's reverse with one rule that closes all three holes.

**1. Keep the goal legal.** `EmptyCircleSolver` takes hazards alongside opponents as weighted
sites: a hazard contributes `distance_to_centre - radius` instead of `distance_to_centre`. The
constraint-triple enumeration is unchanged in structure -- three walls, two walls and a site, one
wall and two sites, three sites -- but every family now solves through one routine that carries
each site's radius, because two sites being equally clear is a linear equation in (cx, cy, r) once
you subtract their squared forms. With no hazards the answers and the family names are identical
to before. `clamp_to_field` in both navigations then pushes the goal out of any hazard after the
rectangle clamp. Without this, the layers below spend the whole match fighting a goal that sits
inside the hole.

**2. Last-ditch reverse (PursuitNavigation only).** Within `hazard_reverse_distance` of a hazard
edge with the heading pointed at it, `linear_x` is overridden to a fixed reverse. Pursuit has no
plant fit, so this stays its only speed response. On MotionProfileNavigation the barrier
supersedes it: a negative barrier bound is the same reverse, scaled by depth and limited by
whatever hazard lies behind.

**3. The velocity barrier (`HazardAvoidance::limit_command`).** One rule: per hazard, the
velocity component toward it may not exceed the stopping envelope

```
bound = (gap_to_hard_radius - max(v_approach, 0) * delay_s) / (tau_lin_d + delay_s)
```

which is the same first-order envelope the goal brake schedule inverts, aimed at the hazard.
It constrains the final command (`u * k * toward <= bound`, with `toward` the cosine between
heading and hazard bearing) and the measured speed (excess over the bound commands a
proportional reverse with full authority). Applied after the angle gate and the wall reverse,
so nothing upstream can bypass it. Three behaviours fall out:

- approaching, thrust tapers with the gap, so a blocked push can never store wheel speed the
  gap cannot absorb;
- coasting past the goal, the measured excess brakes through a zeroed channel, whichever way
  the robot faces. The recovery command's sign follows the hazard's bearing, not the robot's
  forward axis: a hazard ahead gets reverse, a hazard behind gets forward drive, so sliding
  rear-first at a hole is answered with forward, never with backing further in;
- inside the hard radius the bound is negative, which is the reverse backstop restated: an
  exit command scaled by depth, directed away from the centre, and limited by whatever other
  hazard lies on the exit path -- the rear-clearance check falls out of the rule rather than
  being a special case.

Per-hazard constraints compose as an intersection of command intervals. When hazards ahead and
behind squeeze the interval empty, the barrier splits the violation, which stops the robot
rather than diving into either. Passing abeam has zero approach component and costs nothing;
this projection is continuous where the old cap and brake had an on/off cross-track gate that
chattered exactly on the tangent-following heading.

MotionProfileNavigation only, because the envelope is stated in m/s against the plant fit.
`hazard_barrier_enable = false` exists for baseline sweeps that measure the failure mode.

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
loop. When it does not converge, the velocity barrier covers it.

## Diagnostics

- `/diagnostics` `hazards` channel, every cycle: count, per-hazard centre, inflated and hard
  radii, source, and the our-robot half-diagonal the inflation used. Lets a replay tell "the
  controller did not know" from "the controller knew and drove in anyway".
- Both navigations log `hazard_count`, `hazard_waypoint`, `hazard_side` and the steered target.
  MotionProfileNavigation adds `hazard_barrier_engaged` (the barrier changed `linear_x`),
  `hazard_barrier_braking` (measured speed exceeded the envelope, so the change was an active
  brake, not a clip), and `hazard_barrier_bound` (tightest bound in m/s; negative means inside a
  hard radius). PursuitNavigation keeps `hazard_reverse`.
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

### What a hazard carries

`FieldHazard` holds three radii, inflated once at assembly. `inflated_radius` is the steering
keep-out: geometry plus our half-diagonal plus the source margin (0.10 static, 0.05 tracked).
The aim layers and the solver test against it. `hard_radius` is the loss boundary: geometry
plus our half-diagonal plus `hazard_hard_margin_m` (0.02), where a wheel meets the hole lip.
The velocity barrier enforces against it. The split is deliberate: on the measured AER arena
(~1.48 x 1.41 m, two 0.2 m holes at x = +-0.65) the 0.47 m steering discs cover most of the
floor and the opponent stood inside one for 48% of the recorded match, so a speed rule keyed to
the fat radius would forbid the fight itself. `object_radius` is the hazard alone, and is what
the overlays draw -- a ring at the inflated radius reads as the hazard being twice its real
size, since the inflation has no visible cause on screen.

### Why one speed rule

The envelope `gap / (tau_lin_d + delay_s)` is the plant's stopping schedule, the same one the
goal-side brake inverts, so a refit moves both from the `[plant]` table with nothing hand-copied.
The barrier exists because the motion profile's own schedule is keyed to the goal, and in ATTACK
`attack_terminal_speed_fraction = 1.0` pins the reference at full speed at any distance: nothing
mission-side ever slows the robot for a hazard, so the hazard side must own the final command.
Applying it to `v_ref` instead was the first design's mistake -- the angle gate zeroes the linear
channel when the target falls behind, and the recorded robot coasted into the hole at 1-2 m/s
with the brake dutifully logged as active.

## Config reference

`[field_filter]`, shared by every field filter type:

| key | default | meaning |
|---|---|---|
| `hazards_file` | `""` | shared geometry file, repo-relative with extension; empty = no static hazards |
| `hazard_static_margin_m` | 0.10 | steering clearance added to a static hazard beyond our half-diagonal |
| `hazard_tracked_margin_m` | 0.05 | same for a hazard from a live neutral track |
| `hazard_tracked_hold_s` | 0.75 | how long a stale neutral track keeps producing a hazard |
| `hazard_hard_margin_m` | 0.02 | slack on the loss boundary (`hard_radius`) beyond our half-diagonal |

`[navigation]`, both controllers unless noted:

| key | default | meaning |
|---|---|---|
| `hazard_tangent_enable` | true | option 4 |
| `hazard_tangent_max_iterations` | 3 | chained-hazard passes |
| `hazard_waypoint_clearance_m` | 0.06 | clearance the tangent waypoint leaves beyond the keep-out |
| `hazard_side_release_m` | 0.05 | release margin for the latched pass side |
| `hazard_barrier_enable` | true | the velocity barrier, MotionProfileNavigation only; off only for baseline sweeps |
| `hazard_prediction_horizon_s` | 0.25 | how far ahead a TRACKED hazard is advanced |
| `hazard_reverse_distance` | 0.12 | option 2, PursuitNavigation only; 0 disables |
| `hazard_heading_threshold` | 1.047 | ~60 deg; heading must point at the hazard (pursuit) |
| `hazard_reverse_min_speed` | 0.35 | reverse floor while the backstop is active (pursuit) |

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
