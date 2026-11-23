# Feature map

`docs/features.toml` lists every user-facing feature with the places a person reaches it and the
proofs that exercise it from there. Lint checks it against the tree; a local web page shows it live
with the latest test results and staleness. Code: `auto_battlebot/features/`,
`auto_battlebot/verify/{run_store,coverage,gcda,only}.py`, `tools/feature_map/`.

```bash
scripts/feature_map                                     # UI at http://<tailscale-ip>:8780/
scripts/feature_map check                               # what lint runs
scripts/feature_map draft                               # append a stub per unclaimed reach point
scripts/feature_map reach                               # every discovered reach point
scripts/feature_map map                                 # the joined map as JSON
```

## Registry

One `[features.<key>]` table per feature:

| Field | Holds |
| --- | --- |
| `name` | user-facing name |
| `reach` | reach point ids, see below; never `type:` ids |
| `variants` | `{ FactoryType = [proof ids] }`, one key per `REGISTER_CONFIG` type the feature selects between; each key claims `type:<FactoryType>` |
| `code` | owned paths; a trailing `/` means a directory |
| `proofs` | entry-point proof ids |
| `supporting` | proof ids of unit tests of internals |
| `links` | `MonitorLink` names, lower case (`crsf_telemetry`) |
| `docs` | doc paths |
| `status` | `draft` or `reviewed` |

`[ignored]` maps a reach id to the reason it is not a feature. Top-level `enforce` switches the
completeness rules on.

## Reach discovery

`auto_battlebot/features/reach.py` reads the tree directly, so lint needs no build:

| Kind | Id | Read from |
| --- | --- | --- |
| `command`, `status` | `command:/command/set_autonomy` | rows of `include/remote/protocol.hpp` |
| `type` | `type:EspNowTransmitter` | `REGISTER_CONFIG(...)` under `src/` |
| `cli` | `cli:auto_battlebot.rl.lockstep_eval`, `cli:training/gpu_queue.py`, `cli:scripts/lint` | modules under `auto_battlebot/` with a module-level `def main`, outside `tests/`; a module without an `if __name__ == "__main__":` block is reached through the scripts under `training/`, `simulation/` or `logo/` that import its `main`, so those scripts are the reach points; every file at the top of `scripts/` but `README.md` |
| `channel` | `channel:SA` | first column of the Sticks and Switches tables in `firmware/config/CHANNEL_CONVENTION.md` |
| `readout` | `readout:TopDown` | `web/src/components/*.svelte` that import the status or diagnostics stores or subscribe |
| `profile` | `profile:mr_stabs_mk2_aer_zed_box` | `config/*.toml` without a leading `_`, except `profiles.toml` |
| `unit` | `unit:viz-relay` | `service/*.service` |
| `topic` | `topic:/tf` | rows of `include/publisher/viz_topics.hpp` |

Foxglove topics live in one table, `kVizTopics` in `include/publisher/viz_topics.hpp`: topic,
encoding, latch, rate (`PER_FRAME`, `EVENT`, `CONFIGURED`), and whether it is recorded. Call sites
build an `OutputChannel` from a row, and a row with `recorded = false` never reaches the MCAP
recorder. `/diagnostics/<module>` topics come from `diagnostics_topic()` and are not reach points.
`build/viz_topic_dump` prints the table as JSON; a test checks the header parse against it and the
remote protocol parse against `build/remote_protocol_dump`.

## Proof ids

| Runner | Example | Exists when |
| --- | --- | --- |
| `gtest` | `gtest:EspNow.SubstitutesUnderSa` | a `TEST`, `TEST_F`, `TEST_P`, or `TYPED_TEST` with that suite and name under `tests/` |
| `pytest` | `pytest:auto_battlebot/rl/tests/test_env.py::test_mask` | the file has that top-level `test*` function or `Test*` class method; a `[param]` suffix is ignored |
| `vitest` | `vitest:web/src/lib/connection.test.ts::retries` | an `it(` or `test(` with that title in that file under `web/src` or `tools/` |
| `native` | `native:test_pid::Pid.ProportionalOnError` | a gtest declaration in `firmware/mr_stabs_mk2/test/<dir>/` |
| `playback` | `playback:clyde:opponent_coverage` | clip in `auto_battlebot/verify/playback_clips.toml`, metric in `hashes` plus `CLIP_METRICS` |
| `mcap` | `mcap:crsf_telemetry_link`, `mcap:topic_rate:/tf` | a `PerformanceCheck` name, or a registered viz or `/status/` topic |
| `hil` | `hil:flip_sa` | a step id in `auto_battlebot/verify/hil_checklist.toml` |
| `sim` | `sim:attack:reaches_target` | never yet: the nav plan 3.5 smoke tier has no scenarios |

Existence comes from reading the source, so lint needs no build. Parameterized gtests are named
without their instantiation prefix and `/N` suffix.

## Lint

`scripts/lint` runs `check --quiet`. Errors always fail it:

- the registry does not parse, or has unknown fields or a bad `status`
- a listed reach id, variant type, proof id, link, code path, or doc does not exist
- a reach id is claimed twice, or both claimed and ignored

With `enforce = true` these gaps fail it too; before that they are printed as `gap:` lines:

- a reach point no feature claims and `[ignored]` does not list
- a feature with no entry-point proof, or a variant with no proof
- a feature still at `status = "draft"`

`draft` appends stubs for unclaimed points with `code` guessed from the defining file and
`proofs = []`. Lint never writes the registry.

## Run store

Every `scripts/verify` run, including the Stop hook's and the verifier's, writes
`data/verify/runs/<utc>_<commit>[-<dirty hash>].json` with each proof's status (`passed`,
`failed`, `skipped`) and duration. The dirty hash is the content hash of every path that differs
from `HEAD`. The store keeps the newest 500 runs.

| Suite | Report it reads |
| --- | --- |
| gtest | `--gtest_output=json` |
| pytest, synthetic | `--junitxml` |
| web, tools | vitest `--reporter=json` |
| firmware | `pio test --json-output-path` |
| playback | `auto_battlebot.verify.playback check --results`: `playback:` metrics, `mcap:topic_rate:` per registered topic, `mcap:<check>` per monitor check |

A playback metric passes when it equals this machine class's lockstep baseline, or sits inside
`CROSS_CLASS_TOLERANCE` against another class's. A `PER_FRAME` topic passes with at least 95% as
many messages as frames; any other topic passes when present. A check fails when it went red.

## Coverage

`scripts/verify --coverage` (or `python -m auto_battlebot.verify.coverage --suite gtest`) writes
`data/verify/coverage/<commit>.json`: per proof id, the first-party files it executed (its
`touches`).

- gtest: a `build-coverage/` tree (`-DAUTO_BATTLEBOT_COVERAGE=ON`, gcc `--coverage`). Each case
  runs alone with its own `GCOV_PREFIX`. A run of no tests is the baseline; a function counts as
  executed when its arc counters differ from the baseline. Each function is attributed to its own
  source file from `gcov-dump -l` of the `.gcno`, so inline header code lands on the header and
  system and `third_party/` functions drop out. The gcno index is cached in
  `data/verify/coverage/gcno_index.json`
- pytest: `pytest-cov` with `--cov-context=test` over `auto_battlebot`, `scripts`, `training`,
  `simulation`, slow tests left out
- firmware: each `pio test` suite built with `--coverage` into a scratch build dir; every case in a
  suite shares the suite's files
- playback: each clip once in lockstep through `build-coverage/auto_battlebot`; every
  `playback:<clip>:*` id shares the clip's files and every `mcap:` id gets the union

Run it on an idle box; it takes the time listed in
`docs/experiments/platform/2026-11-22_proof_coverage_cost.md`.

## Staleness

A passing proof is stale when a file in its scope changed since the commit it last passed on, from
`git diff <commit>` plus untracked files. The scope is its coverage `touches`, else the `code` of
every feature that lists it. A proof whose last run was on exactly the current commit and dirty hash
is never stale. A failed last result is `failed`; never run (or only skipped) is `unrun`. Flakes
count the commit and dirty-hash pairs on which a proof both passed and failed.

A feature takes its worst proof state in the order failed, stale, unrun, passing; a feature with no
proofs is a gap. Stale never fails lint. `scripts/verify --stale` reruns every stale proof:
`--only` with the stale ids. `--only` maps ids to suites: gtest cases into `--gtest_filter`, pytest
node ids as targets, native ids to `pio test -f <suite>`, vitest ids to the `web` or `tools`
suite, playback ids to their clips, and `mcap:` ids to every clip. `hil:` and `sim:` ids cannot run
there and exit 2.

## UI

`python -m auto_battlebot.features.serve` binds the machine's Tailscale IPv4 (`tailscale ip -4`) on port 8780 (8765 is the Foxglove relay), so any tailnet device can open it. Without Tailscale it falls back to 127.0.0.1; `--host` overrides both. The Vite dev proxy defaults to 127.0.0.1, so set `FEATURE_MAP_API=http://<tailscale-ip>:8780` or run the server with `--host 127.0.0.1`. It
builds `tools/feature_map/` into `auto_battlebot/features/static/` (gitignored) when the bundle is
missing or older than its source, and says to run `npm ci` in `tools/feature_map` when
`node_modules` is missing.

A thread hashes, once a second, the mtimes of `docs/features.toml`, `data/verify/runs/`,
`data/verify/coverage/`, `.git/HEAD` and `.git/refs`, plus every changed working-tree path and its
mtime. When the hash moves it rebuilds the map and pushes a `change` event on `/api/events`; the
page refetches `/api/map`. `/api/diff` adds what `scripts/verify --changed` would run (about 5 s,
so only while the diff overlay is on).

| View | Shows |
| --- | --- |
| Overview | one lane per pipeline stage (camera → perception → filter → target → navigation → transmit → robot) with its counts and a chip per feature; features in two or more stages draw as bands across the lanes they span; platform features sit below the lanes, grouped by reach kind. A stage header opens the map filtered to it; a chip opens the side panel |
| Map | reach → feature → proofs in fixed columns, one proof node per runner per feature; click a group to expand it, a node to select its feature and fade the rest |
| Impact | reach → feature → module → proofs: each proof hangs off the modules its coverage runs through; infrastructure modules collapse into one node, and a toolbar button expands them |

A feature's stages come from its `code` and reach paths; a transmitter channel counts as robot.
`stage` is the stage with the most votes, `stages` lists every one hit, and a feature with none is
platform.

Each proof's `modules` come from its coverage, else its feature's `code`, with test sources dropped.
`include/<x>/` folds into `src/<x>/`. A proof whose coverage spans over a third of a root's modules
(the runner and playback tests) collapses to `<root>/*`, drawn as "whole app". A module is
infrastructure when more than half the features that reach its root use it.

The map and impact columns sit at fixed x. Features are ordered by stage, then name; every other
column is ordered by the mean row of its already-placed neighbors, and edges are S-curves that
leave and enter level. Wheel scrolls the canvas; ctrl or cmd plus wheel zooms. The diff overlay highlights features whose `code` or `touches` overlap
the working changes. The side panel shows each proof's state, duration, last passing commit,
flakes, and the files that made it stale; each variant's config section, keys and defaults
(`build/config_schema_dump`), and the profiles that select it; links with their latest `mcap:`
grade; recent commits and churn since the last green proof. Buttons copy `scripts/verify --only`
commands; the page runs nothing.

`build/config_schema_dump` probes each `REGISTER_CONFIG` type with a `ConfigDefaultsRecorder` in
`PROBE_SCHEMA` mode, where a missing required field is recorded instead of thrown. A type that
needs a nested table (`DeepLabMaskModel`'s `[engine]`) reports a probe error with the fields read
before it. Each top-level profile is resolved through its `extends` chain.

`tools/feature_map` is Svelte 5, Vite and Cytoscape.js (preset layout), at `web/`'s versions. `scripts/verify` runs its `npm run check` and vitest as the `tools` suite; `scripts/lint`
runs prettier over it.
