# Plan: live-select control strategy from the transmitter

## Goal

Let the operator switch the active control strategy live from the physical transmitter,
among: **transmitter** (manual/passthrough), **attack**, **retreat**, **orbit**. No UI
involvement: manual passthrough stays on the existing trainer enable switch, and a
transmitter switch selects among the autonomous strategies.

## Current state

### Navigation strategies

Interface `NavigationInterface` (`include/navigation/navigation_interface.hpp:26`):

```cpp
virtual bool initialize() = 0;
virtual VelocityCommand update(RobotDescriptionsStamped robots, FieldDescription field,
                               const TargetSelection &target) = 0;
virtual const NavigationVisualization &get_last_visualization() const = 0;
```

Existing impls (`include/navigation/`, `src/navigation/`):

- `PursuitNavigation` — **this is "attack"** (pursues/intercepts opponents, includes
  wall-reverse/back-away). Also `MotionProfileNavigation` (plant-model attack).
- `FixedVelocityNavigation`, `NoopNavigation`.
- **`retreat` and `orbit` do NOT exist** — a repo-wide grep for `attack|retreat|orbit`
  returns zero hits. They must be written as new `NavigationInterface` impls (same
  inputs, so feasible).

Selection is **TOML, resolved once at startup**. `make_navigation(config, clock)`
(`src/navigation/config.cpp:37`) is called once in `src/main.cpp:128`; `Runner` holds it
as a fixed `navigation_` `shared_ptr` (`include/runner.hpp:72`) and never reassigns it.
**No runtime navigation switching exists** — the profile switcher only writes a file and
says "Reboot to apply" (`src/runner.cpp:137-147`).

### "Transmitter" = manual passthrough (not a navigation strategy)

Manual mode is the **autonomy-disabled** path, not a nav strategy:

- `Runner::autonomy_enabled_` (`include/runner.hpp:87`);
  `handle_autonomy_toggle_request` (`src/runner.cpp:95-104`) calls
  `transmitter_->enable()` / `disable()`.
- `OpenTxTransmitter::send()` early-returns when disabled
  (`src/transmitter/opentx_transmitter.cpp:112`); with autonomy off, the physical RC
  pilot's sticks drive the robot directly (true passthrough).
- The tick always calls `navigation_->update(...)` then `transmitter_->send(command)`
  (`src/runner.cpp:448-450`); the transmitter gates whether the command is emitted.

So the four-way selector maps onto: **transmitter** = trainer enable switch disengaged
(already a physical hardware gate, no code change needed); the other three = autonomy
enabled + a specific nav strategy, selected by a separate transmitter switch.

### Transmitter channel input path (already exists)

The robot already reads the pilot's switch positions over the CRSF trainer link, and
there is a clean template to copy — the init button:

- `OpenTxTransmitter` parses incoming channel frames (`process_channel_updates`,
  `latest_channels_`, `include/transmitter/opentx_transmitter.hpp:59,66`) and exposes
  them via `get_channel_value(channel_idx)`
  (`src/transmitter/opentx_transmitter.cpp:180`).
- Channel indices come from transmitter config (`include/transmitter/config.hpp`):
  `init_button_channel`, `trainer_enable_channel`, etc.
- `TransmitterInterface::did_init_button_press()`
  (`include/transmitter/transmitter_interface.hpp:13`) is the interface-level accessor;
  the Runner polls it every tick (`src/runner.cpp:346-347`). A strategy switch follows
  the exact same pattern.
- The trainer enable switch already gates autonomous output in hardware:
  `get_channel_value(config_.trainer_enable_channel) < 0` kills output
  (`src/transmitter/opentx_transmitter.cpp:136`). This IS the "transmitter" (manual)
  selection — no new mechanism required for it.

## Approach

Add a runtime strategy selector driven by a transmitter switch channel, replicating the
init-button triple (transmitter config channel → `TransmitterInterface` accessor →
Runner per-tick poll), and give the Runner the ability to switch the active nav strategy
live. "Transmitter" (manual) is not part of the selector: it stays on the existing
trainer enable switch, so the selector only chooses among **attack / retreat / orbit**
(a 3-position switch).

### 1. Implement the missing strategies

- **RetreatNavigation** (`include/navigation/retreat_navigation.hpp` + `src/...cpp`):
  drive away from the selected target/opponent (invert the pursuit vector), respecting
  field bounds (reuse `PursuitNavigation`'s wall handling patterns).
- **OrbitNavigation**: circle the target at a configured radius/direction (tangential
  velocity around the target from `TargetSelection`), with wall-aware arc clipping:
  - The field is an axis-aligned rectangle (`FieldDescription.size`, centered at field
    center, `include/data_structures/field.hpp:33`). Inset it by a `boundary_margin` and
    test the orbit circle (target position, `radius`) against the inset rectangle.
  - **Circle fully inside**: orbit continuously in the configured direction (base case).
  - **Circle intersects a wall**: the full loop is not traversable. Clip the circle to
    the inset rectangle to get the angular interval(s) inside the field, pick the arc
    containing (or nearest to) our robot's current angle around the target, and drive
    back and forth along it: hold tangential velocity until the robot's angle reaches an
    arc endpoint (minus an `arc_endpoint_margin`), then reverse tangential direction.
    Latch the direction with hysteresis so noise near an endpoint doesn't chatter.
    Reuse `PursuitNavigation::distance_to_nearest_wall` / `wall_facing_angle`
    (`src/navigation/pursuit_navigation.cpp:257-266`) or hoist them into a shared helper.
  - **Circle mostly outside** (arc shorter than a configurable minimum sweep, e.g. target
    parked in a corner): degrade to holding position at the nearest in-bounds point on
    the circle rather than oscillating on a sliver.
- Register both with `REGISTER_CONFIG(NavigationConfiguration, ..., "RetreatNavigation" /
  "OrbitNavigation")` in `src/navigation/config.cpp` and add `make_navigation` branches.
  Give each a config struct in `include/navigation/config.hpp` (orbit: radius, speed,
  direction, boundary_margin, arc_endpoint_margin, min_arc_sweep; retreat: speed, etc.).

### 2. Define the strategy set and pre-build the strategies

`make_navigation` needs a fully-typed `NavigationConfiguration` subclass (via
`dynamic_cast`), so runtime construction needs the configs available. Two options:

- **Preferred — multiplexing navigation.** Add a `StrategyMuxNavigation` that implements
  `NavigationInterface`, holds a map `{ "attack" → shared_ptr<NavigationInterface>,
  "retreat" → ..., "orbit" → ... }` built once at startup from a new
  `[navigation.strategies]` config (each entry a normal nav config), plus an atomic
  "active strategy" key. Its `update()` delegates to the active child; its
  `get_last_visualization()` returns the active child's. Switching is just setting the
  atomic key — no reconstruction, no allocation in the hot loop, latency-safe.
- **Alternative — rebuild on switch.** Keep a set of parsed nav configs and call
  `make_navigation` on switch. Simpler config but constructs objects mid-run (possible
  latency spike; must happen off the tick or be cheap). Recommend the mux.

The "transmitter" choice is handled separately (below), not as a mux entry.

### 3. Transmitter plumbing (mirror the init button)

- Add `strategy_select_channel` (plus position thresholds) to the transmitter config
  (`include/transmitter/config.hpp`, next to `init_button_channel` /
  `trainer_enable_channel`).
- Add a `TransmitterInterface` virtual, e.g.
  `virtual std::optional<StrategySelection> get_strategy_selection()` returning the
  bucketed switch position (`ATTACK` / `RETREAT` / `ORBIT`), default `nullopt` (no
  selection, keep current) so `NoopTransmitter` / `PlaybackTransmitter` are unaffected.
- Implement in `OpenTxTransmitter`: bucket `get_channel_value(strategy_select_channel)`
  into low/mid/high with the thresholds. Debounce the same way the init button does
  (`opentx_transmitter.hpp:34-35`); return a stable value, not edges, so the Runner can
  detect changes itself.
- Give `SimTransmitter` a settable selection (keyboard or sim config) so strategies can
  be exercised in `run_simulation.sh` without hardware.

### 4. Runner poll + switch logic

In the tick, next to the existing `did_init_button_press()` poll
(`src/runner.cpp:346-347`), read `transmitter_->get_strategy_selection()` and, when it
differs from the current strategy, apply `strategy_mux_->set_active(name)` (atomic set).
Manual override needs no Runner logic: the trainer enable switch already gates output at
the transmitter level (`opentx_transmitter.cpp:136`), and the existing UI autonomy
toggle (`src/runner.cpp:95-104`) is untouched.

Optionally show the active strategy as a read-only label via
`UIState::set_current_strategy(...)` — display only, no UI input path.

Because switching is an atomic write on state the tick already touches, no blocking work
enters the perception loop.

### 5. Config

Add a `[navigation]` type `StrategyMuxNavigation` with sub-configs, plus the strategy
switch channel on the transmitter, e.g.:

```toml
[transmitter]
# existing params...
strategy_select_channel = 7  # confirm against the robot's OpenTX model channel map

[navigation]
type = "StrategyMuxNavigation"
default_strategy = "attack"

[navigation.strategies.attack]
type = "PursuitNavigation"
# ...existing pursuit params

[navigation.strategies.retreat]
type = "RetreatNavigation"
# radius/speed...

[navigation.strategies.orbit]
type = "OrbitNavigation"
# radius/direction/speed, boundary_margin, arc_endpoint_margin, min_arc_sweep...
```

Confirm the config parser supports nested per-strategy sections (the factory reads a
single `[navigation]` section today — `src/navigation/config.cpp:25`; nested parsing may
need a small addition, parsing each `strategies.*` sub-table through the same
`ConfigFactory<NavigationConfiguration>`).

## Testing

- Unit test `StrategyMuxNavigation`: switching the active key routes `update()` to the
  right child; `get_last_visualization()` follows the active child.
- Unit tests for `RetreatNavigation` / `OrbitNavigation` command directions given a
  synthetic target.
- Orbit wall cases: target near a wall (circle clipped, tangential direction reverses at
  arc endpoints, no chattering at the endpoint with a noisy angle), target in a corner
  (arc below min sweep, holds nearest in-bounds point), target mid-field (full circle,
  no reversal).
- Unit test the switch bucketing in `OpenTxTransmitter`: channel value → strategy,
  thresholds, debounce, and `nullopt` when no channel data has arrived.
- Simulation: `./scripts/run_simulation.sh`, drive the `SimTransmitter` selection,
  confirm behavior changes immediately.
- Hardware: flip the physical switch mid-run, confirm the strategy changes within one
  tick and the trainer enable switch still kills output in every strategy.

## Validation

```bash
./scripts/build_and_test.sh --gtest_filter=StrategyMux*:Retreat*:Orbit*
git diff --name-only HEAD | grep '\.cpp$' | xargs -r clang-tidy -p build-test/
./scripts/lint
```

## Open questions

- **Attack** = `PursuitNavigation` or `MotionProfileNavigation`? Both are attack-like.
  Recommend mapping "attack" to whichever the shipped configs use (`PursuitNavigation`,
  `config/_common.toml:89`), configurable.
- **Retreat/orbit semantics**: retreat target (nearest opponent vs. selected target),
  orbit radius/direction defaults — confirm desired behavior before implementing.
- **Orbit arc reversal dynamics**: does the drivetrain need a decel ramp approaching an
  arc endpoint, or is an instant tangential sign flip acceptable? `arc_endpoint_margin`
  can absorb overshoot either way; tune in simulation first.
- **Moving target vs. arc**: the clipped arc moves with the target. Recompute the arc
  every tick from the latest target pose (cheap: circle vs. rectangle intersection is
  closed-form), and only latch the back-and-forth direction, not the endpoints.
- **Switch/channel assignment**: which physical 3-position switch and which CRSF channel?
  Current map on mr_stabs: CH0/CH1 sticks, CH4 SF (arm), CH5 init button, trainer enable
  per config. Needs a free channel and an OpenTX model edit to mix the chosen switch onto
  it; confirm per robot.
- **Stale channel behavior**: if channel frames stop arriving (link loss),
  `get_strategy_selection()` returns the last value or `nullopt` — hold the current
  strategy either way, or fall back to a default? Recommend hold (trainer enable already
  fails safe on link loss).
- **Mux vs. rebuild**: confirm the multiplexing-navigation approach (recommended) over
  rebuilding strategies on each switch.
