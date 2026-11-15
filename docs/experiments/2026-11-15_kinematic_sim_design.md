# 2026-11-15 kinematic sim design

## Notes moved from code comments (2026-11-15)

### `simulation/camera_utils.py`

Camera FOV is vertical, so fy is computed from height.
fx = fy assumes square pixels.

The arena floor is a plane, so its image is an exact homography of a top-down render of it, no ray marching, no geometry. That is what lets the simulated camera reuse the viewer's
composition instead of rendering the world a second way.

Args:
view_matrix: 4x4 camera-from-world, as built by `camera_view_matrix`.
intrinsics: (fx, fy, cx, cy) for the output image.
metres_per_pixel: scale of the top-down render.
top_down_size: side length of the (square) top-down render, in pixels.

Returns:
A 3x3 matrix suitable for ``cv2.warpPerspective``.

- Top-down pixels to field metres: x right, y up, origin at the centre of the render. (at `half = top_down_size / 2.0`)

Pixels whose ray points at or above the horizon get NaN, which is what a stereo camera reports
where it has no match.

### `simulation/kinematic_sim_server.py`

A lightweight alternative to the Genesis server (sim_server.py): same TCP protocol
(simulation/protocol.py), but 2D kinematic physics instead of rendering. Pair it with
config/headless_sim.toml on the C++ side (Noop perception + GroundTruthRobotFilter), so the real
PursuitNavigation runs unchanged and the resulting MCAP is scored by
playground/control_stage0/stage0_metrics.py exactly like a real fight.

The sim owns logical time: it sends its accumulated sim_time in each response, and the C++ side
adopts it (ManualClock), so the controller's dt is correct no matter how fast the loop free-runs.
Runs are deterministic (seeded) and can go far faster than real time.

Models the effects that actually drive overshoot:
- our robot's drivetrain, from the plant [our_robot] plant selects (simulation/plants/): the
first-order kinematic lag model, or the Mr Stabs Mk2 MuJoCo rigid body
- actuation latency (command ring buffer, in sim ticks)
- perception: the sensing model's latency, dropouts, noise and heading flips
(auto_battlebot/perception/sensing_model.py), plus the flat-plane projection bias
- square arena with walls

Usage:
source scripts/activate_python.sh
python simulation/kinematic_sim_server.py simulation/kinematic_sim.toml

- [sim] trace_csv columns. Poses are ground truth, not what perception reported; goal is the first opponent, which a go-to-point mission drives to. Pitch is 0 on the kinematic plant. (at `TRACE_HEADER = (`)
- Hazards cover most of the reachable arena; fall back to the last draw pushed clear. (at `return self._push_out(*candidate)`)

behaviours so a release resumes from the drop point rather than snapping back. A policy
opponent's plant owns its pose, so it cannot be dragged.

- A drive robot faces the way it moves, so circle and random_walk point their heading along their velocity. The RL environment's opponents are unicycles; a heading that never turned kept the weapon side facing one way while the robot slid around. (at `if behavior == "circle":`)

plus the flat-plane projection bias, or the true poses every tick when no model is set.

The sim captures one perception frame per tick, so the model's frame rate is the tick rate.

- One compositor, two consumers. The UI draws robot markers, the field border and hazard rings by projecting field points through the intrinsics above, so the frame it draws on has to be a plausible camera image or the whole overlay collapses into a few pixels. The floor is a plane, so the camera view is an exact homography of this top-down composition and needs no second renderer. (at `self._world = Viewer(cfg, self._obstacles)`)

Costs wall-clock time and nothing else: the sim owns logical time, so a slower render
makes the sim slower in real time and changes nothing the controller sees.

- a metric derived from the MCAP alone could not see a fall-in, because the run ends there.

### `simulation/mujoco_window.py`

Shows the contact geometry the plant actually steps: the hull meshes, wheels, skid, arena walls,
wall blocks and the opponents (a box or a cylinder each, per [[opponents]] collision_box). The
OpenCV window draws sprites at the same poses, so the two side by side show where the sprites and
the collision shapes disagree.

Passive mode: the sim server keeps stepping, and sync() copies the state to the render thread.

### `simulation/plants/base.py`

- reads `v`, `w` and the hazard fields; the EPISODE line reads the counters.

(N, 2) field-frame polygons for the viewer. Static blocks and arena walls are left out:
the viewer already draws those at their contact size.

Blocks are grown by the robot radius, the same way the kinematic plant grows the arena walls.
A hole swallows the robot when its centre crosses the lip, so holes are not grown.

- Discs of hazard_radius whatever shape the plant collides with: clearance is scored against the keep-out disc the controller plans around. (at `self._moving_blocks = [(x, y, r + self._robot_radius) for x, y, _, r i`)

### `simulation/plants/kinematic.py`

- Below this yaw rate the arc radius v/w blows up, so fall back to the straight-line form. (at `STRAIGHT_W = 1e-6`)
- centre-clamp against blocks grown by our radius is the same contact.
- A block is a wall with a curved face: push the chassis back out along the radius, which is what the rectangle clamp does for the arena walls. (at `for bx, by, br in self._hazards.blocks():`)

### `simulation/plants/mujoco_plant.py`

plant interface.

Walls and wall blocks are real contact geometry here, so the arena is not shrunk by the robot
radius the way the kinematic clamp shrinks it. Holes stay a centre-crosses-the-lip test, and
once the robot falls in it stops stepping. The hit counters count ticks with any contact, which
matches the kinematic plant's one-hit-per-tick behaviour while it pushes into a wall.

- Points around a cylinder rim or a sphere for the footprint hull; 1.3 mm of chord error on the 0.168 m opponent cylinder. (at `FOOTPRINT_SEGMENTS = 32`)

model does not use. A sphere gets three orthogonal great circles, close enough for the skid's
1.5 mm radius.

- Clearance and fall-in from the footprint centre, not the axle the pose tracks. (at `yaw = self._sim.pose()[2]`)

### `simulation/policy_opponent.py`

`[[opponents]] behavior = "policy"` runs `policy_bundle` through `BundlePolicy` the way
`RlPolicyNavigation` runs it on our robot, so a config can put the same policy on both robots.
The opponent steps the MuJoCo plant our robot uses, with the fit and pack voltage from
`[our_robot.mujoco]` and the same `[latency] command_ms`, and attacks our robot.

Each plant carries the other robot as a moving box (`collision_box`), placed at its footprint
centre every tick, which is the shape the RL environment gives its opponent. Two boxes pushed by
two plants is not one contact solve: each robot feels the other as an immovable mocap body.

The opponent sees ground truth: our footprint centre, its finite-difference velocity, no sigmas,
ages or stale flags. That is the observation the policy trained on with sensing off. Our robot
still sees the opponent through the sensing model and the C++ filter.

`pose()` is the opponent's footprint centre, not its axle, so the C++ side, the viewer and our
plant's contact box all see the robot where its footprint is, as the training opponent's pose is
its box centre.

- The transmitter's drive processor sits between the policy and the robot, as SimTransmitter puts it in front of our plant and the RL environment trains with it. (at `action = torch.tensor([self._policy.act(self._view())])`)

### `simulation/viewer.py`

- Manifest suffix for the underside render: the robot rolled 180 deg about its front axis, so the front still points +X and it rotates by yaw like the top view. (at `FLIPPED_SUFFIX = "_flipped"`)

pixels-per-metre (so blits scale to any window) and front-points-+X (so rotation is a plain
rotation by yaw with no per-asset offset). Missing sprites degrade to drawn discs.

- background per frame is free. The texture maps the whole arena, so it scales with it.
- The keep-out ring is the geometry the controller actually steers on. Showing it next to the real hole is the single most useful thing this window does. (at `keepout_px = int(round((obstacle.radius + inflate) * self._px_per_m))`)
- or a hit that the sprites do not explain shows up here.

Opponents are drawn in config order, so the index maps straight onto ``[[opponents]]``. A
run that adds opponents beyond the configured list (none do today) falls back to the first
entry rather than raising mid-frame.

Split out from `compose` because the simulated camera feed warps this into the camera's
view, and text and heading arrows drawn for a top-down window read as garbage once they
are projected into a perspective image.

- The sprite is centred on the footprint, which need not be the pose: the MuJoCo plant's pose is the axle. (at `cx, cy = footprint_center((x, y, yaw), plant.footprint_center_offset)`)
- Free-running at sim speed makes the window unusable for dragging things by hand, so realtime paces the wait to sim.dt. Headless sweeps never get here. (at `wait_ms = 1`)

