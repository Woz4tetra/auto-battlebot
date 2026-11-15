# Hazards

## Radii

Each `FieldHazard` carries three sizes.

- Raw radius (`radius`): the hazard itself before inflation, the hole in the floor or the robot's own footprint. Overlays draw this. Drawing `inflated_radius` instead read as the hazard being twice its real size, because the inflation that makes it a configuration-space obstacle has no visible cause. The robot circles on screen give the eye enough to judge the margin without a second ring for every hazard.
- `hard_radius`: the loss boundary, the geometry plus our half-diagonal plus a small hard margin. Crossing it means a wheel is at the hole lip or the chassis is in contact, so the velocity barrier limits approach speed against it.
- `inflated_radius_m`: the fat disc steering plans around. It is deliberately larger than the hard radius: steering plans around the fat disc, the speed rule only gets strict at the thin one, so a fight in the outer band is legal but never arrives fast.

Half the diagonal of a footprint is the radius of the smallest disc containing it. Using the diagonal rather than half the length keeps the keep-out conservative for a robot approaching a hazard corner-first. The UI draws this same circle per robot so the inflation in a keep-out ring is visible: a tracked ring is this for the hazard, plus this for our own robot, plus the configured margin.

## Layers shared by PursuitNavigation and MotionProfileNavigation

Two aim layers and one speed rule.

- The aim layers (`steer_around`, plus the goal push-out the controllers run in `clamp_to_field`) plan against the fat inflated radius.
- The speed rule (`limit_command`) is a directional velocity barrier against the thin `hard_radius`. Per hazard, the velocity component toward it may not exceed gap / (tau_decel + delay), which is the stopping envelope the plant can follow.
- One rule replaced the old steering speed cap, the stop-at-rim brake, and the fixed hazard reverse. It composes across hazards: each hazard only constrains the velocity component toward itself, so backing away from one is automatically speed-limited by the one behind.
- The only cross-tick state is the latched pass side, so the robot does not re-pick a side every tick and cut back across the hazard it was rounding.

## Tangent waypoint

`steer_around` returns a waypoint that steers around a hazard instead of at the true goal.

- The point is the tangency point on a circle of `radius` around `center`. Once the robot arrives, the run from there to the true goal is tangent to that circle and no longer blocked, so the goal comes back on its own.
- Pass a `radius` a little larger than the keep-out so the waypoint leaves real clearance. Aiming at the keep-out rim itself means arriving with zero clearance and grinding along the edge.
- It picks the side that costs less heading change from `heading`, so the robot does not cut back across the hazard it committed to rounding. `committed_side` latches that choice: pass the previous return's side back in (+1 or -1) to keep it, or 0 to choose freshly.
- Inside the circle there is no tangent, so the waypoint points straight out along the radius. That is the only useful direction there, and the reverse backstop covers the rest.
