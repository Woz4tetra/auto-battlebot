# Plan: hazard avoidance for the floor hole and the house bot

## Goal

Give the control stack a way to say "this region of the field is bad" and steer around it, for
two hazards that behave differently:

- **The floor hole.** Static in the field frame, known before the match, never moves.
- **The house bot.** Tracked live at `Group::NEUTRAL`, moves, and is currently invisible to
  target selection and navigation.

Detection is assumed solved. This plan covers representation, routing, and the control
response. It also folds in an investigation of the run-away overshoot, because overshoot past
a safe point is the same failure that puts a wheel in the hole.

## What the stack can express today

The entire geometric world model is a rectangle. `FieldDescription`
(`include/data_structures/field.hpp:33`) carries a `Size` and a transform, nothing else. Both
navigation implementations reduce avoidance to `distance_to_nearest_wall` and `clamp_to_field`
(`src/navigation/pursuit_navigation.cpp:257`, `:280`). There is no way to name a bad region that
is not the boundary.

The house bot gap is narrower than it looks. `YoloSegRobotBlobModel` emits `Label::HOUSE_BOT`
(`src/robot_blob_model/yolo_seg_robot_blob_model.cpp:541`), `label_group_utils.hpp:28` maps it to
`Group::NEUTRAL`, and `robot_filter.label_mapping` in `config/_common.toml:103` assigns it
`NEUTRAL_ROBOT_1`. It arrives in `RobotDescriptionsStamped` with a filtered pose and velocity, and
then both selectors skip it because they test `group == Group::THEIRS`. The tracking already
works; only the consumer is missing.

## Why steer instead of brake

The fitted plant says braking cannot be the primary response. From `config/_common.toml`:
`k_fwd = 4.88` m/s, `tau_lin_d = 0.1235` s, `delay_s = 0.0522` s. The brake schedule in
`compute_reference_speed` (`src/navigation/motion_profile_navigation.cpp:269`) uses exactly these:

```
stopping distance from full speed = v * (tau_decel + latency) = 4.88 * 0.176 = 0.86 m
```

The turn is cheaper. With `max_yaw_rate = 7.93` rad/s (`include/navigation/config.hpp:223`):

```
minimum turn radius at full speed = v / max_yaw_rate = 4.88 / 7.93 = 0.62 m
```

So at full speed the robot needs 0.86 m to stop and 0.62 m of radius to turn out. On a field
where the hole is a meter or two across, stopping is the response that fails.

Two caveats on the 0.62 m. `velocity_saturation_limit = 1.0` (`config/_common.toml:122`) caps
`|linear| + |angular|` at the transmitter, so full linear and full yaw cannot be commanded at
once. The steer-brake coupling (`c_sb = 2.70`) bleeds forward speed when the turn command is
large. Both make the real achievable radius worse than 0.62 m and both should be measured in sim
before the numbers below get tuned. The ordering does not change: turning out beats stopping.

## Prerequisite: hazard representation and routing

Add to `include/data_structures/field.hpp`:

```cpp
/** A region to keep out of, in the field frame. Discs regardless of the real shape:
 *  every consumer below reduces to a distance-to-center test, and EmptyCircleSolver
 *  is already built on point sites plus a radius. */
struct FieldHazard {
    Pose2D center;
    double radius = 0.0;      // already inflated; see below
    Velocity2D velocity;      // zero for the hole, filtered track velocity for the house bot
    HazardSource source = HazardSource::STATIC;  // STATIC | TRACKED, for diagnostics and decay
};

struct FieldDescription {
    ...
    std::vector<FieldHazard> hazards;
};
```

`FieldDescription` already flows through filter, target selection, and navigation
(`ControlLoop::run_cycle`, `TargetSelectorInterface::get_target`,
`NavigationInterface::update`), so one member reaches every consumer with no signature churn.

**Inflation happens once, at assembly, never at the consumer.** The stored radius is the raw
hazard radius plus our robot's half-diagonal plus a config margin. Every consumer then treats
`radius` as "center distance below this is a loss" and no one re-derives clearance. Get this
wrong in two places and the margins compound silently.

**Two sources, assembled in one place.** Add a `HazardAssembler` called at the top of
`ControlLoop::run_cycle`, right after the filter correct and before `resolve_target`:

1. **Static hazards from config.** A new `[field.hazards]` array of tables parsed alongside the
   field filter config, in field-frame meters. The hole does not move during a match, so config is
   the source of truth and any detector overwrites entries rather than being the only supplier. A
   detector outage must not delete the hole.
2. **Tracked hazards from robot descriptions.** Any description with `group == Group::NEUTRAL`
   becomes a `FieldHazard` with `radius = robot.size` half-diagonal + our half-diagonal + margin,
   carrying `robot.velocity` through. Use `robot.is_stale` to decide whether to keep it: a stale
   house bot track should persist for a configured hold time rather than vanish, since forgetting
   a hazard is worse than holding a slightly wrong one.

Diagnostics: log the assembled hazard list on its own channel each cycle (count, centers, radii,
source) so a replay can tell "the controller did not know" from "the controller knew and drove in
anyway." Publish the same set to the UI overlay and the MCAP recording.

## Option 1: keep the goal out of hazards

The cheapest correctness fix, and a prerequisite for everything after it. Without it, the steering
layers below spend the whole match fighting a goal that sits inside the hole.

- **`EmptyCircleSolver`** (`src/target_selector/empty_circle_solver.cpp`). Extend
  `empty_circle_radius` and `solve_exact` to take hazards alongside opponents. A hazard is a site
  with a nonzero radius, so its contribution to the empty-circle radius is
  `distance_to_center - hazard_radius` rather than `distance_to_center`. The constraint-triple
  enumeration is unchanged in structure: hazard sites join the 4 walls and N opponent points, and
  the wall-point and point-triple families need the offset radius folded into their closed-form
  solves. Cost grows as the cube of the constraint count, so with 4 walls, 1-3 opponents, and 1-2
  hazards this stays inside the ~15 evaluations per tick the solver report measured.
- **`NearestTarget`** (`src/target_selector/nearest_target.cpp`). Reject an intercept point that
  lands inside a hazard, or project it to the hazard boundary. The opponent standing on the hole
  edge is a real case and this is where attack mode declines to follow it in.
- **`clamp_to_field`** in both navigations. Currently a rectangle clamp; extend to push a goal out
  to the nearest hazard boundary after the rectangle clamp.

This makes the goal legal. It does nothing about the path taken to reach it, which is what
options 3 and 4 handle.

## Option 2: last-ditch reverse

Mirror `apply_wall_reverse` (`src/navigation/pursuit_navigation.cpp:240` and the motion-profile
equivalent) with `apply_hazard_reverse`: within `hazard_reverse_distance` of a hazard edge, with
heading error toward it under `hazard_heading_threshold`, override `linear_x` to reverse at
`hazard_reverse_min_speed`.

This is a stopping response and the numbers above say it will fire too late to save a full-speed
approach. Build it anyway, and be honest about its role: it is the backstop for the cases the
steering layer cannot solve, mainly a hazard that appears close because the house bot drove into
our path or because a track initialized late. It is nearly free, since the code shape already
exists and is tuned.

Do not let it become the primary defense. If the sim sweep shows option 2 firing often, that is
evidence options 3 and 4 are mistuned, not that option 2 is working.

## Option 3: speed cap for steering authority

The reframe of the original brake-schedule idea. Not "slow down so you can stop," which the plant
cannot do, but "slow down only as much as the turn requires," because achievable turn radius is
`v / max_yaw_rate` and the only way to tighten it is to reduce `v`.

An arc starting tangent to the current heading deviates laterally by about `L^2 / (2r)` at
along-track distance `L`. To clear a hazard of inflated radius `R` centered on the heading ray at
range `L`, the arc needs `L^2 / (2r) >= R`, so `r <= L^2 / (2R)`. Substituting `r = v / w_max`:

```
v_cap = max_yaw_rate * L^2 / (2 * R)
```

In `compute_reference_speed`, cast the heading ray forward, find the nearest hazard it enters, and
take the min of the existing distance-to-go cap and `v_cap`. With `w_max = 7.93` and `R = 0.4` m:
`L = 1.0` m gives 9.9 m/s, above `k_fwd`, so no cap at all. `L = 0.5` m gives 2.48 m/s. `L = 0.3` m
gives 0.89 m/s. The cap stays out of the way until the hazard is genuinely close and near the
heading, which is the behavior we want.

Apply the cap only when the swept path actually enters the hazard, and scale `R` by how far off
the ray the center sits, so a hazard well off to the side does not throttle a clean run past it.
For a `TRACKED` hazard, evaluate against the hazard's position predicted over the approach time,
not its current position.

Floor the cap at a configurable minimum. A cap that reaches zero recreates the stop-in-time
behavior the plant cannot deliver and strands the robot with a hazard in front of it.

## Option 4: tangent waypoint (the primary response)

Steering, expressed where it costs the least: in target selection, so navigation needs no change
at all. Both navigations only ever chase a `Pose2D`.

When the segment from `our_pose` to `target.pose` intersects an inflated hazard, replace the goal
with the tangent point on that hazard circle, choosing the side that costs less heading change.
The robot drives to the tangent point, and once past the hazard the segment no longer intersects
so the true goal returns. This is a one-hazard-at-a-time greedy step, not a planner, which is the
right altitude here: the field holds at most a handful of hazards and the control loop runs at
250 Hz.

Three details that decide whether this works:

- **Side hysteresis.** The tangent point flipping sides tick to tick is the obvious livelock.
  Latch the chosen side once committed and release only when the segment clears the hazard, the
  same pattern as `apply_hysteresis` and `committed_turn_sign_`
  (`src/navigation/pursuit_navigation.cpp:153`).
- **Moving hazards.** For a `TRACKED` hazard, compute the intersection against the hazard's
  predicted position over the time to reach it, not its current one. The house bot moving across
  our path is the case this exists for.
- **Chained hazards.** After substituting the tangent point, re-test the new segment against the
  remaining hazards, capped at a small iteration count. If it does not converge, fall back to the
  option 3 cap plus option 2 and log it, rather than looping.

Combined with option 3, the behavior is "cap speed enough to make the turn, then take the turn."
That is the pair that does the actual work. Options 1 and 2 are the floor and the ceiling.

## Run-away overshoot investigation

The robot still overshoots in `RUN_AWAY`. This has to be resolved as part of this work, because
overshoot past a safe point is exactly how the robot ends up somewhere it did not choose to be,
and the tangent-waypoint logic in option 4 assumes the robot actually arrives where it is aimed.

Leads, in the order they should be checked, cheapest first:

1. **`retarget_improvement_m` ships at zero.** The default is `0.0`
   (`include/target_selector/config.hpp:33`) and no config in the tree sets it, but
   `run_away_mode_plan.md:664` specified `0.15`. With zero hysteresis, `SafestPointTarget` adopts
   any candidate whose radius beats the held target by any epsilon
   (`src/target_selector/safest_point_target.cpp:48`), so as opponents move the goal steps around
   and the `distance` fed to the brake schedule never settles. A brake schedule chasing a moving
   goal will not converge, and the symptom looks exactly like overshoot. Test by setting `0.15` in
   config and re-running the recorded segments.
2. **EKF speed underread.** The `q_along = 10.0` note in `config/_common.toml` records that the
   filter's forward-speed estimate was wrong by 0.56 m/s and that "the motion profile braked on a
   number that read half the robot's real speed." That hand-set value landed 2026-08-23. Establish
   whether the overshoot observations predate or postdate it. If they postdate it, the sweep only
   fixed the segments it was tuned on and `fit_process_noise.py` needs a session with autonomous
   `RUN_AWAY` motion, which the config comment already calls for.
3. **Open-loop stretches.** `compute_linear_command` drops speed feedback when the track is stale
   (`speed_is_measured == false`), leaving the inverse-plant feedforward to carry the braking
   alone. Measure what fraction of run-away ticks run open loop. If it is high, the brake is
   running on model accuracy with no correction, and the model is the thing under suspicion.
4. **Heading-gate re-entry.** `angle_threshold = 1.7` rad (~97 deg). Below it, linear drive is
   gated off and `prev_v_ref_` and `speed_integral_` reset
   (`src/navigation/motion_profile_navigation.cpp:222`). Run-away starts by turning away from the
   opponent, so every mission crosses this gate. Check whether the `accel_limit` ramp on re-entry
   and the integrator reset combine into a launch that the brake schedule then has to catch.
5. **Solver plateau jitter.** With opponents far away, no opponent constrains the circle and the
   tie-break resolves toward `our_pose`. Confirm the goal is not oscillating in the region where
   the solution is flat.

Method: replay the 2026-08-23 20-37 `RUN_AWAY` segments with the `profile` diagnostics channel
(`distance`, `v_ref`, `v_actual`, `speed_is_measured`) and the `run_away_target` channel (`x`, `y`,
`radius`) on the same time axis. Plot distance-to-held-target against time and mark every goal
change. That single plot separates "the goal moved" from "the brake was late," which are different
bugs with different fixes, and the current evidence does not distinguish them.

Write the result to `docs/experiments/control_improvement/run_away_overshoot_report.md` with the
overshoot metric before and after, using the same metric as `stage3_sim_report.md` so the numbers
are comparable to the existing baseline.

## Simulator work: obstacles, visuals, interactivity

Two reasons this is step 2 rather than an afterthought. Nothing downstream is scoreable without
hazards in the sim, and the tangent-waypoint behavior in option 4 is the kind of thing you
diagnose by watching. A sweep row reading "minimum clearance 0.04 m" does not tell you the robot
picked the wrong side and then cut back across the hazard.

### Static obstacles

`[[obstacles]]` in the sim TOML, parsed into an `ObstacleConfig` dataclass next to
`OpponentConfig` (`simulation/config/kinematic.py:96`):

```toml
[[obstacles]]
kind = "hole"      # hole | wall_block
center = [0.3, -0.2]
radius = 0.25
```

Collision handling goes in `Plant.step` (`simulation/kinematic_sim_server.py:148`) beside the
existing wall clamp: `wall_block` clamps position to the obstacle boundary and zeroes `v`, exactly
as the wall branch does; `hole` sets a `fell_in` flag that ends the run. Return it alongside the
existing `hit` boolean so the server can end the episode and the sweep can count it.

Obstacles must also be excluded from opponent spawn positions and from `random_walk` targets
(`_random_target`, `simulation/kinematic_sim_server.py:209`), or a run starts with an opponent
standing in the hole.

The thing that must not drift: the sim's `[[obstacles]]` and the C++ side's `[field.hazards]`
describe the same world. Put the geometry in one TOML file that both read rather than two
hand-synced copies. `config/simulation/` already holds files the sim side owns and the C++ config
merge can pull the same table in. The fallback is a startup assert in the sweep harness comparing
the two, but a shared file beats a check that only runs in one code path.

### Viewer

Use OpenCV highgui. `opencv-python` is already a dependency, `cv2.setMouseCallback` gives drag
with no threading, `cv2.warpAffine` rotates sprites, and alpha compositing is a few lines of
numpy. Adding pygame buys better event handling than a feature this size needs. If highgui turns
out to be limiting, pygame is the fallback, but do not start there.

The render call goes at the end of the tick in `handle_client`
(`simulation/kinematic_sim_server.py:373`), after `_send_frame`. Rendering there costs wall-clock
time and cannot corrupt the run, because the sim owns logical time: `sim_time` ships in the
response header and the C++ side adopts it through `ManualClock`. A slow render makes the sim
slower in real time and changes nothing the controller sees. That property is what makes an
in-loop viewer safe here, and it is worth not breaking later.

```toml
[viewer]
enable = false          # sweeps stay headless and fast
window_px = 900
render_every = 1        # tick decimation
realtime = true         # pace to sim.dt for a human; false = free-run
```

`enable = false` by default keeps the existing sweeps untouched. `realtime` matters both ways:
free-running is the point for sweeps and unusable for dragging things around by hand.

Frame composition, cheapest layer first:

1. **Background.** `simulation/assets/cage/nhrl_cage_floor.png` (2048x2048), the flattened NHRL
   field texture the deleted Genesis sim used through `assets/cage/floor.obj` as a unit quad. It
   maps to the 2.4 x 2.4 m arena, matching the `ArenaConfig` defaults
   (`simulation/config/kinematic.py:28`). Scale it once at startup to the window size and keep the
   result; blitting a pre-scaled background per frame is free.
2. **Obstacles.** The hole sprite, plus a thin ring at the inflated hazard radius so the keep-out
   the controller sees is visible next to the geometry it came from. Those two circles differing
   is the most useful single thing this viewer can show.
3. **Robots.** Sprite rotated by yaw, alpha-composited at the pose.
4. **Overlays.** Heading ray, the commanded `linear_x` and `angular_z` as a vector, clearance to
   the nearest hazard, sim time, tick count, and a fell-in banner.

The nav target and the option 4 tangent waypoint are the two things most worth seeing, and the sim
cannot see either: the protocol carries a command up and poses down, nothing else. Adding a debug
channel to `simulation/protocol.py` for the controller's current target is a small change, worth
doing when option 4 lands rather than now.

### Mouse drag

`cv2.setMouseCallback` on the viewer window, hit-testing against opponent positions converted to
world coordinates:

- Press within an opponent's radius latches a drag on that opponent.
- While dragged, `Opponent.step` returns early and x/y come from the mouse, clamped to the arena
  the way `_clamp` already does. Add a `dragged` flag rather than special-casing every behavior
  branch.
- Release resumes the configured behavior from wherever it was dropped. For `circle` and `replay`
  that means restarting a parameterized path from a new position, so re-seed the behavior's phase
  from the drop point.

No protocol change is needed. `GT_POSE_FMT` carries poses only and the C++ filter derives
velocity, so a dragged opponent produces a real velocity estimate through the normal path. That is
the behavior worth having: dragging the house bot across our path makes the controller react to a
moving hazard rather than a teleporting one. Drag fast enough and the filter's innovation gate
rejects it, which is honest and worth seeing.

Leave our own robot undraggable. Its pose is the plant's integrated state, and moving it out from
under the plant desynchronizes the EKF in ways that read as control bugs.

### Blender sprites

Render offline, commit the PNGs, keep Blender out of the runtime dependencies. It is not in
`pyproject.toml` and should not be.

`simulation/scripts/render_sprites.py`, run as:

```bash
blender --background --python simulation/scripts/render_sprites.py -- --out simulation/assets/sprites
```

Per asset: import the GLB, point an orthographic camera straight down, set the film transparent,
light it flat rather than with a dramatic key so the sprite still reads at 100 px, render RGBA at
a fixed pixels-per-meter, write the PNG.

Sources already in the tree: `simulation/assets/robots/mr_stabs_mk2.glb`, `mrs_buff_mk2.glb`, and
`house_bot.glb`. There is no `mrs_buff_mk3.glb`, so the mk2 render stands in for it and the
manifest says so rather than mislabeling it. The hole gets a small modeled sprite, dark interior
with a lit rim, so it reads as depth instead of a flat circle.

Two conventions the runtime depends on, both recorded in a `sprites.json` manifest beside the
PNGs:

- **Pixels per meter**, so blitting scales correctly at any window size.
- **Robot front points +X** in the render, so runtime rotation is a plain rotation by yaw with no
  per-asset offset.

Get either wrong and every sprite sits slightly off or the robots drive sideways. The manifest
exists so those two numbers live in one place instead of scattered through the viewer.

## Order of work

1. Run-away overshoot investigation through the measurement step. It is diagnosis, it may resolve
   with a one-line config change, and every option below is tuned against a controller that
   arrives where it is aimed. Do not tune hazard avoidance on top of an unexplained overshoot.
2. Simulator work, in two passes. First the scoreable part: obstacles in the world model, the
   fell-in condition, and the sweep metrics, because nothing after this can be measured without
   them. Then the viewer, sprites, and drag, which is what makes the option 4 behavior debuggable
   by watching it instead of reading a metrics table.
3. `FieldHazard`, the assembler, config parsing, diagnostics, UI overlay, MCAP.
4. Option 1, goal legality. Solver changes get unit tests against a brute-force reference, the
   same approach `run_away_solver_report.md` used.
5. Option 2, the reverse backstop. Small, and it makes the next two safe to test aggressively.
6. Options 3 and 4 together, tuned against the sim sweep. They are one behavior and tuning either
   alone will mislead.

## Testing

Sweep metrics, added next to the existing terminal-error and overshoot columns: hazard entries
(count), minimum clearance (m), and time lost to avoidance against a no-hazard baseline run. The
last one carries the weight, because an avoidance layer that never enters a hazard by refusing to
move scores perfectly on the first two and is useless.

Cover the moving case as well as the hole: a hazard attached to a scripted opponent exercises the
house bot path, and the drag interaction covers the cases a script does not think to produce.

Unit tests: the solver against brute force with hazards present, the tangent-point geometry
including the both-sides-blocked degenerate case, and the `v_cap` formula against hand-computed
values. Replay tests on recorded fights with a synthetic hole placed in the field config, which
works today without any new recordings.

## Open questions

- Hole geometry. A disc is assumed throughout. If the real hole is a long rectangle, a single
  inflated disc either wastes a lot of field or fails to cover the corners. Covering it with two
  or three overlapping discs keeps every consumer unchanged and is probably enough. Decide once
  the arena dimensions are known.
- House bot inflation radius. It is a hazard we must not hit, but treating it as a large keep-out
  disc may make large parts of the field unreachable when it parks near the middle. The inflation
  margin for `TRACKED` hazards likely wants to be smaller than for `STATIC` ones, and possibly
  mode-dependent.
- Whether attack mode should treat hazards differently from run-away. Declining to chase an
  opponent that is standing next to the hole is correct; declining to chase one that is merely
  near the house bot may hand away the match.
