# Empty circle solver

`solve_exact` in `include/target_selector/empty_circle_solver.hpp` finds the point in the field that is farthest from every wall, opponent and hazard. `SafestPointTarget` uses it as its run-away goal.

## Radius

`empty_circle_radius(field, x, y, opponents, hazards)` is the radius of the largest circle centered at `(x, y)` that stays inside the field rectangle, contains no opponent and touches no hazard.

- The field frame origin is the field center, so the wall term is `min(half_x - |x|, half_y - |y|)`. It is negative outside the rectangle.
- A hazard is a site with a radius, so its term is `distance_to_center - hazard.radius`, not `distance_to_center`. Hazard radii arrive already inflated (see `FieldHazard`), so a non-negative result means the whole robot fits, not just its centre.

## Solver

The optimum is a vertex of the generalized Voronoi diagram of the four wall lines plus the sites (opponents and hazards), so at least three constraints are active there.

1. Enumerate every triple of the 4 + N constraints.
2. Solve each triple in closed form.
3. Keep the feasible candidate with the largest true radius.

Degenerate triples (collinear or coincident sites) produce no candidate and are skipped. The surviving triples still cover the optimum.

Hazards join the site set as weighted points. The constraint families are the same, but the closed-form solves carry each site's radius, so a triple that mixes an opponent and a hazard goes through the same routine. Cost grows as the cube of (opponents + hazards). That is fine because a field holds a handful of hazards, not a hundred.

## Tie-break

Candidates tied on radius (the plateau when no opponent constrains the circle) resolve toward `tie_break_near` when given. The caller passes its own position so the goal is the plateau point closest to the robot instead of one across the field. Without `tie_break_near`, ties keep the first candidate in generation order.

## Reported constraint

`EmptyCircle::constraint` (`EmptyCircleConstraint`) names the family that produced the winning center, so a bad target can be traced to the generator that produced it. The hazard variants (`*_HAZARD`, `WALL_SITE_PAIR`, `SITE_TRIPLE`) mean at least one site in the triple was a hazard. `NONE` means nothing was considered. `SafestPointTarget` logs it as `source` in its solver diagnostics.

## Why this solver

It matched a 1 mm brute-force reference on every tick of six recorded fights and was the cheapest of the three methods compared, about 15 evaluations per tick. The comparison against the coarse grid and the Lipschitz branch and bound is in `docs/experiments/control/run_away_solver_report.md`.
