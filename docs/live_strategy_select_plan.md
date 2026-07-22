# Plan: live-select control strategy from the UI

## Goal

Let the operator switch the active control strategy live from the touchscreen UI, among:
**transmitter** (manual/passthrough), **attack**, **retreat**, **orbit**.

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

So a four-way selector maps onto: **transmitter** = `transmitter_->disable()`; the other
three = autonomy enabled + a specific nav strategy.

### UI ↔ core communication (already bidirectional)

The UI is an **in-process LVGL GUI thread** (`UIManager::start()` →
`std::jthread`, `src/ui/lvgl_platform_bound/lvgl_ui_manager.cpp:33`). It communicates with
the Runner via shared `UIState` (`include/ui/ui_state.hpp`, mutex + `std::atomic`), **not**
sockets/ROS. It **already sends commands back to the core**, and there is a clean template
to copy — the profile dropdown:

- Command enum `UiCommandType` (`src/ui/lvgl_platform_bound/lvgl_ui_controller.hpp:11-20`):
  already has `SET_AUTONOMY_ENABLED`, `SELECT_PROFILE`, `SET_MANUAL_TARGET`, etc.
- `UiController::dispatch` writes to `UIState` atomics.
- `UIState` request fields drained each tick by `Runner::handle_ui_requests`
  (`src/runner.cpp:157-171`).
- Profile dropdown UI: `src/ui/lvgl_ui.cpp:593-631` (`profile_dropdown`, `profile_cb` at
  :101-107 → `controller->select_profile(...)`), with
  `UIState::set/get_available_profiles` (`ui_state.hpp:125-128`).

## Approach

Add a runtime strategy selector by replicating the profile-dropdown triple
(UI widget → `UiCommand` → `UIState` field → Runner drain), and give the Runner the
ability to switch the active nav strategy live. Treat "transmitter" as the
autonomy-disabled case rather than a nav object.

### 1. Implement the missing strategies

- **RetreatNavigation** (`include/navigation/retreat_navigation.hpp` + `src/...cpp`):
  drive away from the selected target/opponent (invert the pursuit vector), respecting
  field bounds (reuse `PursuitNavigation`'s wall handling patterns).
- **OrbitNavigation**: circle the target at a configured radius/direction (tangential
  velocity around the target from `TargetSelection`).
- Register both with `REGISTER_CONFIG(NavigationConfiguration, ..., "RetreatNavigation" /
  "OrbitNavigation")` in `src/navigation/config.cpp` and add `make_navigation` branches.
  Give each a config struct in `include/navigation/config.hpp` (radius, speed, etc.).

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

### 3. UI plumbing (mirror the profile dropdown)

- Add `UiCommandType::SELECT_STRATEGY` (`lvgl_ui_controller.hpp:11-20`) and a
  `controller->select_strategy(name)` that dispatches it.
- Add a `UIState` field: `std::atomic` requested-strategy (enum or small string id) with
  `set_requested_strategy` / `take_requested_strategy`, plus
  `set_available_strategies` / `set_current_strategy` for display (mirror
  `ui_state.hpp:52-67,125-128`).
- Add a strategy dropdown in `src/ui/lvgl_ui.cpp` next to the profile dropdown
  (:593-631), options `["transmitter", "attack", "retreat", "orbit"]`, callback →
  `controller->select_strategy(...)`.

### 4. Runner drain + switch logic

In `Runner::handle_ui_requests` (`src/runner.cpp:157-171`), drain the requested strategy
and apply:

- `"transmitter"` → `autonomy_enabled_ = false; transmitter_->disable();` (reuse the
  existing autonomy path, `:95-104`). Selecting any autonomous strategy →
  `autonomy_enabled_ = true; transmitter_->enable();` **and** set the mux's active key.
- `"attack"/"retreat"/"orbit"` → `strategy_mux_->set_active(name)` (atomic set).
- Update `UIState::set_current_strategy(...)` for display feedback.

Because switching is an atomic write and the transmitter enable/disable path already
exists, no blocking work enters the perception loop.

### 5. Config

Add a `[navigation]` type `StrategyMuxNavigation` with sub-configs, e.g.:

```toml
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
# radius/direction/speed...
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
- Simulation: `./scripts/run_simulation.sh` (or playback), switch strategies live via the
  UI, confirm behavior changes immediately and that selecting "transmitter" disables
  autonomous output.

## Validation

```bash
./scripts/build_and_test.sh --gtest_filter=StrategyMux*:Retreat*:Orbit*
git diff --name-only HEAD | grep '\.cpp$' | xargs -r clang-tidy -p build-test/
./scripts/check_and_fix
```

## Open questions

- **Attack** = `PursuitNavigation` or `MotionProfileNavigation`? Both are attack-like.
  Recommend mapping "attack" to whichever the shipped configs use (`PursuitNavigation`,
  `config/_common.toml:89`), configurable.
- **Retreat/orbit semantics**: retreat target (nearest opponent vs. selected target),
  orbit radius/direction defaults — confirm desired behavior before implementing.
- **Switch safety**: should switching to "transmitter" mid-match require a confirmation or
  the physical trainer switch? Note the existing hardware `trainer_enable_channel`
  interaction (`opentx_transmitter.cpp:136`).
- **Mux vs. rebuild**: confirm the multiplexing-navigation approach (recommended) over
  rebuilding strategies on each switch.
