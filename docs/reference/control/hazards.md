# Hazards

## Radii

Each `FieldHazard` carries three sizes.

- Raw radius (`radius`): the hazard itself before inflation, the hole in the floor or the robot's own footprint. Overlays draw this. Drawing `inflated_radius` instead read as the hazard being twice its real size, because the inflation that makes it a configuration-space obstacle has no visible cause. The robot circles on screen give the eye enough to judge the margin without a second ring for every hazard.
- `hard_radius`: the loss boundary, the geometry plus our half-diagonal plus a small hard margin. Crossing it means a wheel is at the hole lip or the chassis is in contact, so the velocity barrier limits approach speed against it.
- `inflated_radius_m`: the fat disc steering plans around. It is deliberately larger than the hard radius: steering plans around the fat disc, the speed rule only gets strict at the thin one, so a fight in the outer band is legal but never arrives fast.

Half the diagonal of a footprint is the radius of the smallest disc containing it. Using the diagonal rather than half the length keeps the keep-out conservative for a robot approaching a hazard corner-first. The UI draws this same circle per robot so the inflation in a keep-out ring is visible: a tracked ring is this for the hazard, plus this for our own robot, plus the configured margin.

## Discs

A `FieldHazard` is a disc in the field frame regardless of the real shape. Every consumer reduces to a distance-to-centre test, and `EmptyCircleSolver` is already built on point sites plus a radius. Cover an awkward shape with two or three overlapping discs rather than teaching the consumers a second geometry.

## Layers shared by PursuitNavigation and MotionProfileNavigation

Two aim layers and one speed rule.

- The aim layers (`steer_around`, plus the goal push-out the controllers run in `clamp_to_field`) plan against the fat inflated radius.
- The speed rule (`limit_command`) is a directional velocity barrier against the thin `hard_radius`. Per hazard, the velocity component toward it may not exceed gap / (tau_decel + delay), which is the stopping envelope the plant can follow.
- The speed rule applies to the final command against measured speed, so neither the angle gate nor a wall reverse can carry the robot in.
- One rule replaced the old steering speed cap, the stop-at-rim brake, and the fixed hazard reverse, after the 2026-08-28 AER recording showed all three losing to a blocked-push spin-up and an angle-gated coast. It composes across hazards: each hazard only constrains the velocity component toward itself, so backing away from one is automatically speed-limited by the one behind.
- The only cross-tick state is the latched pass side, so the robot does not re-pick a side every tick and cut back across the hazard it was rounding.

## Tangent waypoint

`steer_around` returns a waypoint that steers around a hazard instead of at the true goal.

- The point is the tangency point on a circle of `radius` around `center`. Once the robot arrives, the run from there to the true goal is tangent to that circle and no longer blocked, so the goal comes back on its own.
- Pass a `radius` a little larger than the keep-out so the waypoint leaves real clearance. Aiming at the keep-out rim itself means arriving with zero clearance and grinding along the edge.
- It picks the side that costs less heading change from `heading`, so the robot does not cut back across the hazard it committed to rounding. `committed_side` latches that choice: pass the previous return's side back in (+1 or -1) to keep it, or 0 to choose freshly.
- Inside the circle there is no tangent, so the waypoint points straight out along the radius. That is the only useful direction there, and the reverse backstop covers the rest.

## Pursuit wiring

Only the last-ditch reverse is wired into `PursuitNavigation`. The velocity barrier is stated in m/s against the plant fit, and pursuit closes its loops in normalized command against an unfitted plant, so there is no honest envelope to enforce there. The barrier lives in `MotionProfileNavigation`. The tangent fields ride along in the shared settings struct but pursuit does not call `steer_around`.

## Reverse backstop

The wall reverse overrides `linear_x` to reverse within a distance (m) of a hazard edge when the heading also points at it. 0 disables it. It is a stopping response, and the numbers say it fires too late to save a full-speed approach. It is a backstop for what the steering layer cannot solve, mainly a hazard that appears close because the house bot drove into our path or its track initialised late. If a sweep shows it firing often, that is evidence the steering layer and the velocity barrier are mistuned, not evidence the backstop is working.

## Hazard sources and decay

The assembler is the one place inflation happens. The two hazard sources decay differently on purpose.

- Config geometry (the hole) never expires. The hole does not move during a match, so config is the source of truth for it. A path typo in the hazards file throws instead of loading nothing, because a silent miss would drop the hole from the controller's model while the sim still has one in the floor.
- A neutral track is held past going stale and then dropped. Forgetting a hazard is worse than holding a slightly wrong one, so the disc outlives the track. The hold is bounded by the configured window: an unbounded hold would fence off a chunk of the field for the rest of the match after a house bot leaves the arena.
- A file entry cannot claim kind `ROBOT`. That kind is reserved for live tracks, so a file that claims it is a typo or a misunderstanding.
- Opponents are not hazards. The solver already avoids them as point sites, and keep-out discs for them would double-count them and make the field unreachable.
- The loss boundary (`hard_radius`) carries only the small hard margin, so steering plans around the fat disc while the velocity barrier gets strict at the thin one.
- Our robot's footprint radius has a fallback. Without one, a frame with no our-robot track would briefly shrink every keep-out disc, which errs in the unsafe direction.

## Goal push-out and tangent waypoint edge cases

- A goal dead centre in a disc has no radial direction. Any push-out direction is acceptable, but it must be the same one every tick or the goal jitters.
- Push-out clamps to the field. Without the clamp, a hazard hugging a wall would push the goal out of the arena.
- Overlapping discs can bounce a point back and forth. The push-out gives up after an iteration cap instead of looping, so the guarantee is that it terminates and stays inside the field.
- A gap too narrow to thread is blocked by the other hazard whichever side is chosen. The iteration cap ends the search and the returned waypoint is finite.
- The pass side latches so the robot does not cut back across a hazard it is already rounding. The latch wins over a cheaper side on a later approach.
- Inside the circle the waypoint exits outward from the robot's current position (a robot at +0.3 exits at +1.0), not through the far side of the disc.
- The waypoint reacts to where a hazard will be over the prediction horizon, not where it is now. A house bot crossing the path is currently clear but moving in, and aiming at its current position aims at where it was.

## Velocity barrier envelope

Per hazard, the approach bound is `(gap - max(v_approach, 0) * delay) / horizon`, and the command constraint is `u * k * toward <= bound`, with `k` the plant gain for the commanded direction. Properties:

- A command near a hazard never asks for a speed the remaining gap cannot absorb, which keeps wheels from spinning up during a blocked push. Example with `k_fwd = 5`, horizon 0.2 s, delay 0.05 s: a stationary robot 0.4 m away has bound 2 m/s, so full thrust clips to 0.4.
- Excess recovery opposes the approach, not the robot's forward axis. With the rear toward the hazard and the robot sliding into it, recovery lands as full forward drive, away from it. A literal reverse would back the robot into the hole.
- The projection is on the approach component, so tangent motion has zero approach and rounding a hazard at speed is free. The old brake's on/off across-gate sat on that boundary and chattered. The projection is continuous.
- Inside a hazard ahead and one behind, the forward limit demands reverse and the rear limit demands forward. No command satisfies both, so the barrier splits the violation, which comes out as a stop.
- Pursuit builds the shared settings without a plant fit, so speeds are zero. The barrier is a pass-through there instead of dividing by zero.
- The barrier has an enable switch so a baseline sweep can measure the failure mode with it off.
