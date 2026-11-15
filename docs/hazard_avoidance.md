# Hazard avoidance in navigation

Radii and the shared layers are described in `docs/hazards.md`.

## Layout

Two aim layers and one speed rule. The aim layers keep the goal legal (`clamp_to_field` pushes it out of every keep-out) and steer a tangent around whatever blocks the run to it, both against the fat inflated radius. The velocity barrier is the only speed rule: the approach-speed component toward every hazard is limited to what the plant can shed in the remaining gap to the hard radius, applied to the final command against measured speed, so neither the angle gate nor a wall reverse can carry the robot in.

The barrier replaced the steering speed cap, the stop-at-rim brake, and the fixed hazard reverse after the 2026-08-28 AER recording showed all three losing to a blocked-push spin-up and an angle-gated coast.

## Pursuit wiring

Only the last-ditch reverse is wired into `PursuitNavigation`. The velocity barrier is stated in m/s against the plant fit, and pursuit closes its loops in normalized command against an unfitted plant, so there is no honest envelope to enforce there. The barrier lives in `MotionProfileNavigation`. The tangent fields ride along in the shared settings struct but pursuit does not call `steer_around`.

## Reverse backstop

The wall reverse overrides `linear_x` to reverse within a distance (m) of a hazard edge when the heading also points at it. 0 disables it. It is a stopping response, and the numbers say it fires too late to save a full-speed approach. It is a backstop for what the steering layer cannot solve, mainly a hazard that appears close because the house bot drove into our path or its track initialised late. If a sweep shows it firing often, that is evidence the steering layer and the velocity barrier are mistuned, not evidence the backstop is working.
