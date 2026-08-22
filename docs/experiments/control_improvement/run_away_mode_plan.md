# Run-away mode plan

Add a second driving behavior: instead of chasing the nearest opponent, drive to the
safest open spot on the field. A switch on the transmitter picks which behavior is
active, so the driver can bail out mid-fight without touching the Jetson.

Nothing here is implemented yet. This is the design and the change list.

## Goal

- New target selector that returns the center of the largest circle fitting inside the
  field walls with no opponent in it.
- New transmitter input that reports which behavior mode the driver has selected.
- `OpenTxTransmitter` reads that input from an RC channel.
- Attack behavior (`NearestTarget`) is unchanged and stays the default.

## What the code already gives us

Facts checked against the current tree, so the plan does not assume behavior that
is not there:

- Field frame origin is the field center. Half extents are `field.size.size.x / 2.0`
  and `field.size.size.y / 2.0`. `pursuit_navigation.cpp:259` and
  `motion_profile_navigation.cpp` both use that convention.
- `TargetSelection` carries a `Pose2D` and a `Label`. Both navigation implementations
  read `target.pose.x` and `target.pose.y` and ignore `target.pose.yaw`. The run-away
  selector will leave `yaw` at 0.0 rather than compute a facing angle that no consumer
  reads.
- `ControlLoop::resolve_target()` (`src/control_loop/control_loop.cpp:69`) holds
  `previous_selected_target_` when the selector returns `std::nullopt`, so a selector
  that cannot produce an answer this frame does not need its own fallback.
- `ControlLoop::pump_input()` already polls the transmitter for edge-triggered and
  level state (`did_init_button_press()`, `is_connected()`) and stores it in atomics
  for the control thread. The mode read fits the same pattern.
- `OpenTxTransmitter::get_channel_value()` clamps to `[-1000, 1000]` and returns 0
  when no channel frame has arrived. `did_init_button_press()` shows the threshold
  comparison pattern to copy.
- Channel config fields are 0-indexed array positions, not radio channel labels.
  `ChannelsParser` builds one flat `std::array<int16_t, 32>` from the two phase
  packets (`channels_parser.cpp:90-94`), and `get_channel_value()` indexes it
  directly. Radio CH7 counted from 1 is config index 6. Existing values agree:
  `linear_channel = 0` is radio CH1.
- `process_channel_updates()` logs the whole 32-entry array under `channels`, but
  only when the array changes. Nothing decodes the switch positions, so reading a
  recording means knowing the config and counting array positions by hand.
- Config uses the `REGISTER_CONFIG` factory. Adding a target selector type means one
  config struct, one `REGISTER_CONFIG` line, and one branch in `make_target_selector`.

## Design decisions

### Mode lives on the transmitter, not in config

The driver flips it mid-match, so it cannot be a TOML field. It comes in over the same
CRSF channel stream the init button already uses.

### The mode is a parameter, not selector state

`TargetSelectorInterface::get_target()` gains a third parameter:

```cpp
virtual std::optional<TargetSelection> get_target(const RobotDescriptionsStamped &robots,
                                                  const FieldDescription &field,
                                                  BehaviorMode mode) = 0;
```

The alternative was a composite selector holding an attack selector, a run-away
selector, and a callback into the transmitter. That hides the coupling and forces
`make_target_selector()` to take a transmitter it otherwise has no reason to know
about. A parameter keeps every selector stateless with respect to mode and costs
two one-line edits, since `NearestTarget` and `NoopTarget` are the only existing
implementations plus one mock in `tests/`.

This does change `target_selector_interface.hpp`, which was not in the original ask.
Calling it out because it is the one scope expansion in this plan.

### One new selector handles both modes

`ControlLoop` keeps its single `target_selector_`, and the selector named in config
answers both modes. `SafestPointTarget` runs the safe-point search in `RUN_AWAY` and
delegates to a `NearestTarget` member in `ATTACK`.

The alternative, a selector that returns `std::nullopt` outside run-away mode, leaves
attack mode with no target at all and makes the config depend on which switch position
the driver happens to be in.

Setting `type = "NearestTarget"` keeps the current single-behavior build and ignores
the switch entirely. That is the fallback if the switch mapping goes wrong at an
event.

## Largest empty circle

Find the center of the largest circle that fits inside the field walls and contains no
opponent. That center is the target.

For a candidate center `c` and opponent positions `o_i`, the largest circle centered
at `c` has radius:

```
wall_clearance(c) = min(half_x - |c.x|, half_y - |c.y|)
opp_clearance(c)  = min over i of  distance(c, o_i)
radius(c)         = min(wall_clearance(c), opp_clearance(c))
```

Pick the `c` that maximizes `radius`. The wall term is the distance to the nearest of
the four walls, which is what keeps the circle inside the rectangle. The opponent term
keeps every opponent outside it.

With no opponent visible `opp_clearance` is unbounded, the radius collapses to
`wall_clearance`, and the argmax is the field center. That is the right answer for
"furthest from all four walls".

Our own position does not appear. The chosen spot is the most open place on the field
regardless of where we are, which is what "run away" should mean. Our pose only enters
as a tie-break (see target stability below).

This is the textbook largest-empty-circle problem with point sites in a rectangle.

### Opponents are points, not discs

`radius` above treats each opponent as a point, so the circle may overlap an
opponent's body by up to its half-width. For a 3 lb bot that is roughly 0.08 m.

Modeling opponents as discs would mean subtracting a body radius from each opponent
term. `RobotDescription` has a `size` field for exactly this, but it is only ever
populated by `ground_truth_robot_filter.cpp:51`, which is the simulation path. On
hardware it stays zero, so a disc model built on `size` would silently degrade back to
a point model in the case that matters.

Starting with points. If bench testing shows the circle hugging opponents, subtracting
a fixed `opponent_radius_m` from each opponent term is a one-line change. Not adding
the knob before there is evidence it is needed. The offset is not cosmetic: it changes
which cell wins whenever an opponent rather than a wall is the binding constraint.

### The property all three solvers lean on

`radius` is a min of 1-Lipschitz functions (distance to a line, distance to a point),
so it is itself 1-Lipschitz: moving the center by `d` changes the radius by at most
`d`. That gives every grid-based method a hard error bound instead of a hope, and it
gives the pruning method its bound. It is worth stating once because both A and C are
built on it.

`radius` is not concave, so there is no gradient method to reach for. An NHRL 3 lb
cage is roughly 2.44 m square; assume at most three opponents.

## Solver candidates

Three methods, distinct in kind rather than in a parameter setting. The experiment
below picks one; only the winner gets implemented in C++.

### A. Uniform coarse grid, single pass

Sweep a fixed grid, evaluate `radius` at each node, keep the running best. No
refinement.

At `h = 0.15` that is 17 x 17 = 289 candidates, each costing two wall comparisons and
one distance per opponent. Roughly 1.2k floating point operations per cycle. No
allocation, no branching, no degenerate cases.

Accuracy follows from the Lipschitz property: every field point is within `h / sqrt(2)`
of a node, so the best node's radius is within `h / sqrt(2)` of the true maximum. At
`h = 0.15` that is 0.106 m. The center can be further off than that whenever the
optimum sits on a plateau.

Fastest and simplest. The 0.106 m radius understatement is the cost, and it lands
directly on the "nowhere is safe" test, which compares the radius against the robot's
half-diagonal.

### B. Exact, by constraint-triple enumeration

The optimum is a vertex of the generalized Voronoi diagram of the four wall lines plus
the opponent sites. Equivalently, at the optimum at least three constraints are active,
so enumerate every triple from the `4 + N` constraints and solve each in closed form:

- three lines: for a rectangle two are opposite parallel walls, giving a point on the
  mid-line
- two lines and a point: quadratic
- one line and two points: quadratic
- three points: circumcenter

Each triple yields zero, one, or two candidate points. Check each for feasibility
inside the rectangle, evaluate its true radius against all constraints, keep the max.

Cost is `C(4 + N, 3)`: 4 triples with no opponent, 10 with one, 35 with three. Each
triple is a handful of operations plus a `4 + N` feasibility scan. That is roughly 1k
operations at three opponents, the same order as method A while being exact.

Building a full Voronoi diagram is not required and would be the wrong tool at this
size. The enumeration is the same answer without the data structure.

Degeneracies that must be handled, and the reason this is the most code of the three:

- Collinear sites have no circumcenter.
- Duplicate or near-coincident sites, which happen when two tracks land on one robot.
- Sites outside the field, which the filter can produce.
- With no opponent on a non-square field the solution set is a whole segment of the
  medial axis, not a point. The enumeration returns its endpoints; the tie-break picks
  between them.

### C. Lipschitz branch and bound

Start with a coarse grid at `h0 = 0.20`. Take the best value `r*`. For any cell, the
Lipschitz property bounds the best possible value inside it at `value_at_center +
h / sqrt(2)`, so any cell whose bound falls below `r*` cannot contain the optimum and
is discarded. Subdivide the survivors, halve `h`, repeat until `h / sqrt(2)` is under
the tolerance.

This converges to any requested accuracy while evaluating a small fraction of what a
uniform fine grid would. With one opponent most of the field prunes on the first pass.
Accuracy is a config knob rather than a consequence of the grid, and unlike method B
there are no degenerate geometric cases to enumerate.

Cost is data-dependent, which is the drawback. A crowded field prunes less, so the
worst case has to be measured rather than derived. That is one of the things the
experiment is for.

### Target stability

Grid argmax jitters between adjacent cells as an opponent moves, and navigation would
chase the jitter. Two rules:

- Keep the previous run-away target unless a new candidate beats its radius by
  `retarget_improvement_m`. Re-measure the held target's radius each cycle so a
  target that has gone bad is abandoned promptly.
- Break ties among cells of equal radius by preferring the one closest to our current
  pose, so we do not cross the field for nothing. Ties are common here: with no
  opponent on the field a broad plateau of cells share the same wall-limited radius.

Clear the held target when the mode leaves `RUN_AWAY`, so re-entering run-away starts
fresh instead of resuming a stale point.

### Missing our pose

Still return a target. The radius does not depend on our pose, so the search runs
normally and only the tie-break degrades: fall back to the first cell of maximum
radius in scan order. `NearestTarget` needs our pose and returns `std::nullopt`
without it, but the safe point does not, and a run-away request is exactly the moment
to keep producing an answer.

The one thing that does need our pose is `ATTACK` mode, which delegates to
`NearestTarget` and inherits its `std::nullopt`.

## Solver experiment

Pick the solver on measurements from real fights, not on intuition.

### Shape: C++ solvers, nanobind, Python for data only

All three solvers are written once, in C++, as production code. A nanobind module
exposes them, and Python drives the experiment: load recordings, hand the trace to
C++, get results back, aggregate, plot. Python never implements a solver and never
times one.

This buys correctness rather than speed. Porting a Python winner into C++ afterwards
leaves the thing that was measured and the thing that ships as different code, and
that drift surfaces as field behavior rather than as a failing test. Writing the
solvers in the library from the start means the experiment measures the exact objects
the app links.

It also lets the brute-force reference run on every tick instead of a sample. At 1 mm
on a 2.44 m field that is roughly 6M evaluations per tick, minutes per trace in
Python and seconds in release C++. Method B gets validated against ground truth
everywhere rather than on a subset.

The split:

- **Python** reads MCAP, builds the trace, calls into C++, aggregates results, writes
  figures. Uses `auto_battlebot.mcap_io` and `control_stage0/diag_io.py` rather than
  new MCAP decoding.
- **C++** holds the four solvers, runs them over the trace, and does all timing.

Nothing reads MCAP from C++. The library already depends on foxglove/mcap
(`CMakeLists.txt:69`), but the recorded payloads are ROS1-serialized `MarkerArray`
and the repo only ever serializes, never deserializes. Keeping I/O on the Python side
sidesteps that entirely and reuses loaders that already work.

### Where the solvers live

`include/target_selector/empty_circle_solver.hpp` and
`src/target_selector/empty_circle_solver.cpp`, in `auto_battlebot_lib` from day one:

```cpp
struct EmptyCircle {
    Pose2D center;
    double radius = 0.0;
    int evaluations = 0;
};

EmptyCircle solve_coarse_grid(const Size &field, const std::vector<Pose2D> &opponents,
                              double resolution_m);
EmptyCircle solve_exact(const Size &field, const std::vector<Pose2D> &opponents);
EmptyCircle solve_branch_and_bound(const Size &field, const std::vector<Pose2D> &opponents,
                                   double tolerance_m);
EmptyCircle solve_brute_force(const Size &field, const std::vector<Pose2D> &opponents,
                              double resolution_m);
```

`SafestPointTarget::solve()` calls whichever one wins. The two losers get deleted once
the report is written; leaving three solvers in the library to rot is how a config
knob nobody understands appears two years later.

`evaluations` rides along in the result so the portable cost metric comes from the
solver itself rather than from a Python-side estimate.

### Timing, in C++ only

The binding takes a whole trace and returns a per-tick result table, so the language
boundary is crossed twice per run instead of twice per tick:

```cpp
// Called once. Loops internally, times each call, returns N rows.
BatchResult run_batch(const TraceView &trace, Method method, double parameter, int repeats);
```

Each tick is timed with `std::chrono::steady_clock` around the solver call and nothing
else. No allocation inside the timed region: hoist the opponent vector out of the loop
and reuse it.

Two measurement honesty points:

- `steady_clock::now()` costs roughly 25 ns per call on x86, so two calls per tick put
  about 50 ns of timer overhead against a solve that may only take 1 to 5 µs. That is
  1 to 5%. The `repeats` parameter runs the same tick R times inside one timed region
  and divides, which amortizes the timer away. Report `repeats = 1` as the honest
  number and use a higher value only to confirm the small ones are not timer noise.
- `repeats > 1` is also cache-warm, which the real loop is not. Treat it as a lower
  bound, not as the number that goes in the report.

Build the module from `build/`, the release tree. `build-test/` is a debug build and
timing three solvers there would answer nothing.

Input is the existing Jetson fight recordings:

```
data/recordings/auto_battlebot_main_2026-05-02_*_repaired*.mcap
```

Jetson recordings specifically, not laptop SVO re-runs. Desktop replay frames are
affine-warped a few percent against live Jetson frames, and replay timing differs, so
opponent positions from a laptop re-run are not the positions the robot actually saw.
Nothing here needs the stack re-run: the recordings already carry everything.

### One gap to close first

`diag_io.load_robot_tracks()` returns which robots were present per tick, not where
they were. The `robot_bounds` CUBE markers already carry `pose.position`, so the fix
is a sibling function, `load_robot_positions()`, returning per-tick x/y per frame id.
`load_field_size()` already recovers the arena rectangle from `/field_markers`.

### Binding

`src/bindings/run_away_solver_ext.cpp`, a nanobind module named
`run_away_solver_ext`, exposing exactly two things:

- `run_batch(field_w, field_h, opponents, method, parameter, repeats)` taking the
  trace as numpy arrays and returning a numpy record array of per-tick
  `center_x, center_y, radius, evaluations, elapsed_ns`.
- The `Method` enum.

Nothing else crosses. No per-tick calls, no solver exposed individually, no timing
helper on the Python side. Keeping the surface this small is what keeps the timing
honest: there is no way to accidentally measure the boundary instead of the solver.

Opponents arrive as a flat `(x, y)` array plus a per-tick count offset array, since
tick opponent counts vary from 0 to 3. A ragged list of Python lists would allocate
per tick inside the timed region.

### Scripts

One stage per script, each writing something inspectable, so a bad trace is caught
before it silently pollutes the comparison:

1. `extract_traces.py` reads one MCAP and writes `traces/<recording>.csv`: one row per
   `/robot_markers` tick with timestamp, field width and height, our x/y, opponent x/y
   per track, and opponent count.
2. `compare_solvers.py` loads a trace, calls `run_batch` once per method, and writes
   per-tick `results.csv` and an aggregated `summary.csv`. This is the whole Python
   side of the comparison: marshal, call, aggregate.
3. `plot_results.py` writes figures to
   `docs/experiments/control_improvement/assets/`.

No `solvers.py`. That file was the port-and-drift risk.

### Establishing truth

Brute force at 1 mm on every tick, not a sample. In release C++ the roughly 6M
evaluations per tick run in the low tens of milliseconds, so a full trace is a
coffee-break job rather than an overnight one.

Then check method B against it everywhere. If B matches to floating point on every
tick of every recording, B is exact and its degenerate-case handling is sound. Any
disagreement is a bug in B, and finding it here beats finding it on the field.

Running brute force on all ticks rather than a subset is worth the compute
specifically because B's failure modes are degenerate configurations: collinear sites,
coincident tracks, sites outside the rectangle. Those are rare per tick and certain
across a fight, which is exactly the shape a sampled check misses.

### Metrics

Per method, over all ticks:

- Radius error against the reference: mean, p95, max.
- Center displacement against the reference: mean, p95, max.
- Frame-to-frame center jitter, as median absolute step between consecutive ticks.
  This drives navigation directly. A method that is accurate but hops between tied
  plateau cells is worse in practice than one slightly off but stable.
- Candidate evaluations per tick.
- Wall time per tick, with the caveat below.
- Fraction of ticks where the best radius falls below our half-diagonal. This is the
  "nowhere is safe" rate, and it answers the open question at the end of this plan as
  a side effect.

Split every metric by opponent count (0, 1, 2 or more). With no opponent on the field
the solution is a large tied plateau, so center displacement will look awful while
radius error is exactly zero. Pooling those ticks with the rest produces a summary
that is arithmetically true and completely misleading. The perception dropout baseline
says the opponent track is live under half of frames, so this is most of the data, not
an edge case.

### What the timing numbers are worth

They are real C++ release-build numbers on the code that ships, which is as good as
this gets on x86. Report `evaluations` alongside them anyway: it is machine
independent, and it is what predicts Jetson behavior before anyone runs it there.

x86 timings still do not settle the question. The 60 ms budget applies on an Orin
Nano, which has different cache behavior and a much weaker single core. Build the
module on the Jetson with the bindings option on and re-run the batch there before
treating any timing as final. That run needs no camera and no radio: it reads the same
trace CSV.

The estimates in the solver section put all three within an order of magnitude of each
other and all far under budget. If the measurements agree, the decision comes down to
fidelity, jitter, and how much code each one is, and the report should say that
plainly rather than manufacture a speed winner out of a 3 µs difference.

### Deliverable

`docs/experiments/control_improvement/run_away_solver_report.md`: the metric table,
the figures, the pick, and the reason. That report is what unblocks section 7 below.

## Changes

### 1. `include/enums/behavior_mode.hpp` (new)

```cpp
#pragma once

namespace auto_battlebot {
enum class BehaviorMode { ATTACK, RUN_AWAY };
}  // namespace auto_battlebot
```

Matches the existing one-enum-per-header style in `include/enums/`.

### 2. `include/transmitter/transmitter_interface.hpp`

Add alongside `is_connected()`:

```cpp
/** Behavior mode selected by the driver. Defaults to ATTACK for transmitters with
 *  no mode input. */
virtual BehaviorMode behavior_mode() const { return BehaviorMode::ATTACK; }
```

Non-pure with a default, so `NoopTransmitter`, `PlaybackTransmitter`,
`SimTransmitter`, and the test mock in `tests/test_control_loop.cpp` need no edit.

### 3. `include/transmitter/config.hpp`

Two new fields on `OpenTxTransmitterConfiguration`, parsed in `parse_fields()`:

```cpp
/** RC channel carrying the behavior mode switch (radio CH7). Reads above
 *  behavior_mode_threshold to select RUN_AWAY, otherwise ATTACK. */
int behavior_mode_channel = 6;
int behavior_mode_threshold = 500;
```

Named `behavior_mode_channel` rather than `run_away_channel` because the channel
carries the mode selection, not a one-way trigger.

Two existing defaults change:

```cpp
int trainer_enable_channel = 5;  // was 7
/** Init button is disabled. Neither the mrs_buff_mk3 transmitter nor mr_stabs_mk2
 *  maps a button for it; field re-initialization is a touchscreen action. 31 is the
 *  last array slot and nothing is mapped there, so the threshold never trips. */
int init_button_channel = 31;  // was 5
```

Radio CH6 now carries the trainer enable switch. Until now it was not broadcast on
any output channel, so `get_channel_value(7)` returned 0 every cycle. The linear
acceleration limiter reads `get_channel_value(config_.trainer_enable_channel) < 0`
to decide whether to hold the ramp at standstill
(`opentx_transmitter.cpp`, `limit_linear_acceleration`). With the channel dark that
branch never fired and the limiter always ran. Pointing the config at a channel that
actually carries the switch turns that hold-at-standstill path on for the first time.
Watch for it on the first bench run: a command that used to ramp immediately will now
sit at zero until the switch is engaged.

Update the doc comment on `trainer_enable_channel` too. It currently says SG is "not
broadcast by default", which is no longer true for this model.

### Disabling the init button

The mrs_buff_mk3 transmitter has no button for it, and mr_stabs_mk2 does not use the
feature either. Field re-initialization comes from the touchscreen, which is a
separate path: `runner.cpp:336` collects UI requests into `should_reinit_field`, and
`runner.cpp:345` only ORs the transmitter edge onto it. Killing the channel leaves
the touchscreen route untouched.

Index 31 works as a disable because nothing is mapped there and the threshold is 500.
It is a real array slot, not an out-of-range sentinel: phase-1 packets fill indices
16 through 31 (`channels_parser.cpp:91-93`), so index 31 carries radio CH32. An unused
channel reads 0 or a large negative, neither of which crosses 500.

Worth knowing, since this is a convention rather than an enforcement:
`did_init_button_press()` already returns false unconditionally for a negative index
(`opentx_transmitter.cpp:199`), and `get_channel_value()` guards the same way. Setting
`init_button_channel = -1` would be a disable the code enforces rather than one that
depends on CH32 staying quiet. Going with 31 as asked; noting -1 because the guard is
already there if the convention ever bites.

Either way the `switch_states` diagnostic below reports the init button state, so a
CH32 that turns out to carry something shows up as a logged flag rather than as
unexplained mid-match field re-initialization.

### 4. `include/transmitter/opentx_transmitter.hpp` and `src/transmitter/opentx_transmitter.cpp`

```cpp
BehaviorMode behavior_mode() const override;
```

Implementation mirrors `did_init_button_press()` but is level-triggered, not edge-
triggered, and has no latch:

```cpp
BehaviorMode OpenTxTransmitter::behavior_mode() const {
    return get_channel_value(config_.behavior_mode_channel) > config_.behavior_mode_threshold
               ? BehaviorMode::RUN_AWAY
               : BehaviorMode::ATTACK;
}
```

`get_channel_value()` is already `const` and returns 0 when no channel frame has
arrived, so a disconnected radio reports `ATTACK`.

### 4a. Switch state diagnostic

The `channels` entry already carries every raw value, but decoding it means knowing
the config and counting array positions. Add a second entry that logs the decoded
switch states directly, so a replay shows what the transmitter believed without
cross-referencing anything. The duplication with `channels` is deliberate: the raw
array stays the ground truth for debugging a bad mapping, and the decoded entry is
what gets read when reconstructing a fight.

New private method called from `update()`, right after `process_channel_updates()`:

```cpp
void OpenTxTransmitter::log_switch_states() {
    const int mode_raw = get_channel_value(config_.behavior_mode_channel);
    const int trainer_raw = get_channel_value(config_.trainer_enable_channel);
    const int init_raw = get_channel_value(config_.init_button_channel);
    const bool run_away = mode_raw > config_.behavior_mode_threshold;
    const bool trainer_enabled = trainer_raw < 0;
    const bool init_pressed = init_raw > config_.init_button_threshold;
    // ... skip if unchanged since the last logged state ...
    logger_->info("switch_states", {{"behavior_mode", run_away ? "run_away" : "attack"},
                                    {"behavior_mode_raw", mode_raw},
                                    {"trainer_enabled", static_cast<int>(trainer_enabled)},
                                    {"trainer_raw", trainer_raw},
                                    {"init_button", static_cast<int>(init_pressed)},
                                    {"init_button_raw", init_raw}});
}
```

Details that matter:

- Log the raw value next to each decoded flag. A switch mapped to the wrong channel
  or a threshold set wrong both look like "flag never changes", and the raw value is
  what separates them.
- Log on change, plus once on the first channel frame. A level state that only logged
  on transitions would be absent from a recording where the driver never touched the
  switch, which is exactly the case where you want to confirm it read `attack` the
  whole time. Mirror the `channels_changed` test in `process_channel_updates()`,
  where an unset `latest_channels_` counts as changed.
- Use `info`, not `debug`. These three flags decide whether the robot moves at all
  and which behavior it runs. They should survive a default log level.
- Trainer enable is active when the channel reads negative, matching the existing
  comparison in `limit_linear_acceleration()`. Do not silently normalize it to
  "positive means on"; the sign is a property of the OpenTX model.
- Include the init button state even though the channel is disabled. It is the only
  thing that would catch index 31 turning out to carry a live signal, and a field
  re-initialization firing mid-match with no visible cause is expensive to diagnose
  after the fact.

Keeping this in `update()` leaves `behavior_mode()` const and side effect free, so
`ControlLoop` can poll it without ordering concerns.

### 5. `include/target_selector/target_selector_interface.hpp`

Add the `BehaviorMode mode` parameter to `get_target()`.

### 6. `include/target_selector/nearest_target.hpp`, `noop_target.hpp`, `src/target_selector/nearest_target.cpp`

Add the unused parameter. Both ignore it.

### 7. `include/target_selector/safest_point_target.hpp` and `src/target_selector/safest_point_target.cpp` (new)

```cpp
class SafestPointTarget : public TargetSelectorInterface {
   public:
    explicit SafestPointTarget(const SafestPointTargetConfiguration &config);
    std::optional<TargetSelection> get_target(const RobotDescriptionsStamped &robots,
                                              const FieldDescription &field,
                                              BehaviorMode mode) override;

   private:
    /** Radius of the largest circle centered at (x, y) that clears the walls and every
     *  opponent. */
    double circle_radius(double x, double y, const FieldDescription &field,
                         const std::vector<Pose2D> &opponents) const;

    /** Center of the largest empty circle. Body is whichever solver the experiment
     *  picks; the signature does not change either way. */
    std::optional<Pose2D> solve(const FieldDescription &field,
                                const std::vector<Pose2D> &opponents) const;

    NearestTarget attack_selector_;
    double retarget_improvement_m_;
    std::optional<Pose2D> held_target_;
};
```

`solve()` is a thin call into `empty_circle_solver.hpp`, which the experiment already
built and measured. Nothing gets ported: the winning function is already in
`auto_battlebot_lib`, and this step is wiring plus deleting the two losers.

Everything except which solver `solve()` names is fixed regardless of the outcome, so
this file can be written before the report lands.

`get_target()` delegates to `attack_selector_` in `ATTACK` and clears `held_target_`.
In `RUN_AWAY` it collects opponent positions once, calls `solve()`, and applies the
stability rules above.

Collect opponents into a local `std::vector<Pose2D>` before solving rather than
re-walking `robots.descriptions` inside the solver. The filtering is a `group ==
Group::THEIRS` test over a handful of entries, but hoisting it keeps the hot path to
arithmetic. Reuse `pose_to_pose2d()` from `transform_utils.hpp` and the
`Group::THEIRS` pattern from `nearest_target.cpp:14-30`.

The unit tests below cover the same geometric cases the experiment already checked
against brute force, so they are a regression net over code that has been measured
rather than a first check of new code.

### 8. `include/target_selector/config.hpp` and `src/target_selector/config.cpp`

```cpp
struct SafestPointTargetConfiguration : public TargetSelectorConfiguration {
    /** Radius (m) a new candidate must beat the held target by before retargeting.
     *  Suppresses solver jitter as opponents move. */
    double retarget_improvement_m = 0.15;

    SafestPointTargetConfiguration() { type = "SafestPointTarget"; }
    // PARSE_FIELD_DOUBLE for each
};
```

The accuracy knob depends on which solver wins and cannot be written until the
experiment reports. Method A needs `grid_resolution_m`, method C needs
`solver_tolerance_m`, and method B needs neither because it is exact. Add the one the
winner needs and nothing else.

`wall_margin_m` and `opponent_weight` are gone with the score function that needed
them: the circle radius is a real distance, so there is nothing left to weight, and
the inset is redundant once the radius is what gets maximized.

Plus `REGISTER_CONFIG(TargetSelectorConfiguration, SafestPointTargetConfiguration,
"SafestPointTarget")` and a branch in `make_target_selector()`.

### 9. `src/control_loop/control_loop.cpp` and `include/control_loop/control_loop.hpp`

In `pump_input()`, next to the existing `transmitter_connected_.store(...)`:

```cpp
behavior_mode_.store(static_cast<int>(transmitter_->behavior_mode()));
```

`std::atomic<int>` rather than `std::atomic<BehaviorMode>` to match how
`autonomy_applied_` is already stored. `resolve_target()` takes the mode and passes
it to `target_selector_->get_target()`.

The manual-target override in `resolve_target()` stays ahead of the selector call, so
a UI-set target still wins in either mode.

Add the mode to `ControlOutput` so the UI and MCAP recording show which behavior
produced a given target. Without it, replays of a run-away sequence look like the
attack selector went haywire.

Log the winning circle radius next to the target as well. It is the one number that
says whether the chosen spot was actually open or merely the best of a bad set, and
the open question at the end of this plan cannot be answered without it in recordings.

### 10. `config/_common.toml`

The `[transmitter]` section currently sets none of the channel fields and rides on the
header defaults. Make the mapping explicit there, since it now describes real switch
positions on the radio rather than fallback values:

```toml
init_button_channel = 31     # disabled; no transmitter maps a button, reinit is a touchscreen action
trainer_enable_channel = 5   # radio CH6
behavior_mode_channel = 6    # radio CH7, attack vs run away
```

Both the header default and this line move to 31 so the two agree. A header default
that says one thing while every config says another is how the CH6 collision would
have gone unnoticed.

Leave `[target_selector] type = "NearestTarget"` as is. Switch a single robot config
to `SafestPointTarget` for bench testing before promoting it to `_common.toml`.

## Transmitter channel mapping

The OpenTX model now broadcasts both switches. Config indices are 0-based, radio
labels are 1-based, so every index is one less than the label:

| Radio channel | Config index | Config field             | Purpose            |
| ------------- | ------------ | ------------------------ | ------------------ |
| CH1           | 0            | `linear_channel`         | Linear / left      |
| CH2           | 1            | `angular_channel`        | Angular / right    |
| CH6           | 5            | `trainer_enable_channel` | Trainer enable     |
| CH7           | 6            | `behavior_mode_channel`  | Attack vs run away |
| unmapped      | 31           | `init_button_channel`    | Disabled           |

A three-position switch on CH7 works as well as a two-position one, since the
comparison is a single threshold. Only the top position selects run away.

Index 31 is not mapped to anything. It holds the disabled init button so the field
never re-initializes from a channel read.

That also settles a collision this remap would otherwise have created:
`init_button_channel` used to default to 5, which is the index CH6 now carries.
Leaving both there would have made every trainer enable flip register as an init
button press and re-initialize the field mid-match.

Confirm the mapping on the first bench run by flipping each switch and watching
`switch_states`. Expect `behavior_mode` and `trainer_enabled` to move and
`init_button` to stay put.

## Tests

Every geometric case has a hand-checkable answer, so assert on the radius, not just
the position. A test that only checks the center passes for the wrong reason when the
clearance terms are wrong in compensating ways.

- `tests/test_safest_point_target.cpp` (new):
  - No opponents on a 2.44 m square field: center at the origin and radius 1.22 m,
    both within the chosen solver's accuracy bound.
  - No opponents on a 3.0 x 2.0 field: radius 1.0 m, limited by the short axis, and
    the center on the long axis rather than pinned to the origin. Catches an
    implementation that assumes a square field.
  - Two coincident opponent tracks on one robot, and an opponent outside the field
    rectangle. Both come from the filter in practice and both are degenerate for the
    exact solver. Skip these two only if the experiment picks a grid method, which
    handles them without special cases.
  - One opponent at the origin of a square field: the four corners tie, the winner is
    the corner nearest our pose, and its radius equals the wall clearance there.
  - Opponent moved off-center: the largest circle sits on the far side, and its radius
    is strictly greater than the equidistant case.
  - Opponent hard against one wall: the circle does not straddle that wall, verified
    by the radius being no more than the center's distance to it.
  - `ATTACK` mode returns the same answer as `NearestTarget` for the same input.
  - Held target survives a small opponent move and is abandoned when a new candidate
    beats it by more than `retarget_improvement_m`.
  - Mode transition `RUN_AWAY` to `ATTACK` to `RUN_AWAY` clears the held target.
  - No our-robot in `descriptions`: still returns a target in `RUN_AWAY`, returns
    `std::nullopt` in `ATTACK`.
- `tests/test_config.cpp`: one case parsing a `[target_selector]` section with
  `type = "SafestPointTarget"` and both fields.
- `tests/test_control_loop.cpp`: a mock transmitter that reports `RUN_AWAY` and an
  assertion that the mode reaches the selector.
- `OpenTxTransmitter` channel decoding: feed a synthetic channel array and assert
  `behavior_mode()` returns `RUN_AWAY` above the threshold, `ATTACK` below it, and
  `ATTACK` before any channel frame arrives. If there is no existing seat for driving
  the transmitter without a serial port, cover this through `ChannelsParser` plus a
  direct test of the threshold comparison rather than building one.

## Build

New `.cpp` files under `src/` need a CMake reconfigure; the source glob is not
`CONFIGURE_DEPENDS`. Files under `tests/` do not: that glob is
`CONFIGURE_DEPENDS` (`CMakeLists.txt:380`).

```bash
cmake -S . -B build
./scripts/build_and_test.sh --gtest_filter=SafestPointTarget*
./scripts/lint
```

### nanobind module

Add nanobind by `FetchContent` next to the existing dependencies, and gate the module
behind an option so it is not required to build or deploy the app:

```cmake
option(BUILD_PYTHON_BINDINGS "Build the nanobind solver module" OFF)
```

The module links `auto_battlebot_lib`, so it exposes the same objects the app does.

Three wiring details:

- Build it from `build/`, the release tree, with `-DBUILD_PYTHON_BINDINGS=ON`. This is
  the whole reason the experiment can report timings at all.
- Leave `pyproject.toml` on its setuptools backend and emit the `.so` to a fixed path
  that `compare_solvers.py` adds to `sys.path`. Switching to scikit-build-core would
  rework a working pure-Python package for no benefit here.
- Compiler flags are `-Wall -Wextra -Werror`. nanobind headers may not survive that,
  so relax them on the module target the same way the tree already does for other
  third-party targets rather than weakening them project-wide.

Turn the option on when building on the Jetson for the confirmation run, and leave it
off for normal deploys so the Jetson image does not need Python dev headers.

## Open question

The circle framing gives a crisp test for "nowhere is safe": if the best radius is
smaller than our own half-diagonal, no spot on the field fits the robot clear of
everything, and the selector is returning the least bad option while presenting it
like a good one.

The options are returning `std::nullopt` so navigation keeps doing whatever it was
doing, or driving to the least bad spot anyway. The solver experiment reports how
often the case comes up, since it is measuring the radius on real fight data already,
so this should be answerable from the report rather than guessed at.

## Next steps

Two tracks that do not block each other. The transmitter work needs no solver, and
the solver experiment needs no hardware.

Transmitter track:

1. Implement the enum, the transmitter method, the channel defaults, and the
   `switch_states` diagnostic.
2. Bench test the channel mapping with the radio connected: flip each switch and
   confirm `switch_states` reports the flag you expect, with `init_button` static.
3. Confirm the trainer enable hold-at-standstill path behaves. It is live for the
   first time now that CH6 carries the switch.

Solver track:

4. Add `load_robot_positions()` to `diag_io.py` and run `extract_traces.py` over the
   May Jetson recordings. Check a trace by hand against a known frame before trusting
   the rest.
5. Write `empty_circle_solver.{hpp,cpp}` with all four solvers, plus the nanobind
   module and the CMake option.
6. Validate method B against brute force on every tick of every trace. Fix B until it
   matches or until the mismatch is understood.
7. Run the comparison on x86, then rebuild on the Jetson with
   `-DBUILD_PYTHON_BINDINGS=ON` and re-run the batch there against the same traces.
8. Write `run_away_solver_report.md` and pick a solver.

Then:

9. Implement `SafestPointTarget` calling the chosen solver, and delete the other two.
10. Bench test with a static opponent stand-in before running it against a real robot.
11. Replay an SVO from `config/playback/` with `SafestPointTarget` configured and
    confirm the attack path is unchanged against `NearestTarget` while the switch reads
    `ATTACK`.
