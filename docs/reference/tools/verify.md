# scripts/verify and the agent gates

`scripts/verify` runs every test suite from one command. The Claude Code hooks in
`.claude/settings.json` run the affected part of it on every stop and require an independent
verifier verdict for code changes. The code is `auto_battlebot/verify/`; tests are
`auto_battlebot/verify/tests/test_verify.py`.

## Commands

```bash
scripts/verify                          # every suite, in full
scripts/verify --fast                   # lint --quick first, then every suite without slow tests
scripts/verify --changed                # suites the git working changes affect
scripts/verify --changed a.cpp b.py     # suites these paths affect
scripts/verify --changed --files-from list.txt
scripts/verify --suite gtest --suite web
scripts/verify --changed --list src/crsf/crsf_parser.cpp   # print the selection, run nothing
scripts/verify --only gtest:EspNow.Sends pytest:a/tests/test_x.py::test_a   # exactly these proofs
scripts/verify --stale                  # every proof the feature map shows stale
scripts/verify --coverage               # record which files each proof executes
```

Exit code 0 is pass, 1 is a failing step, 2 is a usage error. `--playback` and `--realtime` run
the playback regression tiers (`docs/reference/tools/playback_regression.md`); alone they run
only that tier, and with `--fast`, `--changed`, or `--suite` the suites run alongside. `--sim` and
`--hil` exit 2 until the nav plan 3.5 smoke tier and phase 6 of
`docs/plans/verification_plan.md` land.

Step logs go to `.claude/state/verify-logs/<step>.log` (gitignored), or `--log-dir`. The Stop hook
and `hooks suites` log to `.claude/state/verify-logs/<change hash>/`, so two runs never share a log. A failing step
prints its last 40 lines.

Every run writes its per-proof results to the run store, `data/verify/runs/` (gitignored); the
feature map reads it. `--only`, `--stale`, and `--coverage` are described in
`docs/reference/tools/feature_map.md`.

## Suites

| Suite | Steps | Full run on the dev box (2026-11-20) |
| --- | --- | --- |
| `gtest` | `cmake` if `build-test/Makefile` is missing, memory-capped `make -j6 auto_battlebot_test`, then the binary with `--gtest_brief=1` | 68 s for 1,013 tests, plus the build |
| `pytest` | `venv/bin/python -m pytest -n 8 --dist loadfile` (the `testpaths` in `pyproject.toml`); `--fast` adds `-m "not slow"` | 110 s for 1,089 tests with every slow test (2026-11-21) |
| `synthetic` | `pytest training/synthetic` | 2 s for 458 tests |
| `firmware` | `pio test -e native` in `firmware/mr_stabs_mk2` | 22 s for 216 cases |
| `web` | `npm run check`, then `npm test` (vitest) in `web/` | under 30 s |
| `tools` | `npm run check`, then `npm test` in each `tools/<name>/` with a `package.json` | 2 s for `feature_map` |

Lint runs first and alone, because `scripts/lint --quick` rewrites files. The suites then run in
parallel, one thread per suite; steps within a suite run in order and stop at the first failure.

The build runs under `systemd-run --user --scope -p MemoryMax=16G` when user systemd is present
(the box crashed on an uncapped `make -j32`). `VERIFY_JOBS` overrides the job count.

pytest runs on 8 xdist workers, whole files per worker, with `OMP_NUM_THREADS` and the BLAS thread
counts set to 4. Without the thread cap each worker's torch takes every core and the run is slower
than serial: FastTD3 went from 20 s to 700 s. A single test file runs in-process with no workers.

Tests over 5 s carry `@pytest.mark.slow`, and `--fast` (the Stop hook) leaves them out. The full
`scripts/verify` still runs them. The slow set: the warp zero-friction rollout (85 s), three
`test_sysid_coverage.py` cases, two `test_mk2_pcba.py` cases, the FastTD3, PPO and toy EKF cases in
`test_rl_core.py`, and `test_sensing_model.py::test_measured_fraction_matches_tables`. The Stop
hook's pytest set for a filter-and-RL change dropped from about 250 s to 17 s.

### One suite run per change

`venv/bin/python -m auto_battlebot.verify.hooks suites --session <id>` runs the fast suites on the
session's change and records `verify-pass-<hash>`. The verifier runs it instead of
`scripts/verify`, and the Stop hook skips its own run when the marker exists, so the suites run
once per change hash whichever of the two gets there first.

## Selection

`--changed` maps each path to suites:

| Path | Selects |
| --- | --- |
| `tests/**/*.cpp` | gtest suites declared in that file (`TEST`, `TEST_F`, `TEST_P`, `TYPED_TEST`) |
| `include/**`, `tests/**` headers | gtest suites of every test source whose depfile lists the header |
| `src/<m>/<f>.cpp` | as its header `include/<m>/<f>.hpp`, else every header in `include/<m>/` |
| `CMakeLists.txt`, `cmake/`, `config/`, `third_party/` | all of gtest |
| a C++ file no test includes, or no depfiles yet | all of gtest |
| `firmware/mr_stabs_mk2/`, `firmware/shared/` | firmware |
| `web/`, `include/remote/` | web |
| `training/synthetic/` | synthetic |
| `conftest.py`, `pyproject.toml` | all of pytest |
| any path | each `<root>/**/tests/test_*.py` outside `training/synthetic/` whose first-party import closure contains it, or that names it in a string literal (full path, file name, or dotted module) |

Depfiles are `build-test/CMakeFiles/auto_battlebot_test.dir/tests/**/*.o.d`, so header mapping
follows real transitive includes and needs one earlier build. The Python import closure follows
`auto_battlebot.*` imports, including relative ones, and the imports of any
`.py` file a test names by path.

A code file (`.py`, `.cpp`, `.hpp`, `.sh`, `.ts`, `.svelte`, ...) that selects nothing prints as
`no test covers <path>`. It does not fail the run.

## Agent hooks

All state lives in `.claude/state/` (gitignored), keyed by session id.

| Hook | Command | Does |
| --- | --- | --- |
| SessionStart | `hooks session-start` | records `HEAD` in `base-<session>.txt` |
| PostToolUse on Write, Edit, MultiEdit, NotebookEdit | `hooks touch` | records `HEAD` as the base for a session that has none yet |
| Stop | `hooks stop` | runs `verify --fast --changed` over the touched files, then checks the verdict |
| SubagentStop, matcher `verifier` | `hooks subagent-stop` | records the verifier's verdict |

The touched files are read from git: `git diff --name-only --no-renames <base>` (commits since the
base and working-tree edits, deletions included) plus untracked files that are not ignored. Edits
made through Bash and commits made during the session count the same as edit-tool writes. Paths
under `.claude/state/` and `.claude/worktrees/` are left out. The working tree is shared, so
uncommitted edits someone else makes during the session count as part of the change too. The
verifier records no base, so it never opens a change of its own.

### Stop

1. No touched files: pass.
2. The change hash is a SHA-256 over each touched path and its current bytes. A pass is cached
   as `verify-pass-<hash>`, so an unchanged change does not re-run.
3. A failing step blocks the stop with the step summary.
4. When a touched path is under `src/`, `include/`, `firmware/`, `auto_battlebot/`, or `tests/`,
   the stop needs a `PASS` verdict for the current hash. No verdict blocks with the session id
   and hash the verifier needs. A `FAIL` verdict blocks with its findings.
5. The same hash blocking for the same reason a third time lets the stop through with the reason,
   so the user decides. A finding (file and check) that failed in two verdicts lets the stop
   through the same way.

### Verifier

`.claude/agents/verifier.md`, Sonnet, tools Read, Grep, Glob, Bash. It gets the session id, the
change hash, and the user's request, and reads the change itself:

```bash
venv/bin/python -m auto_battlebot.verify.hooks change --session <id>   # hash, base, files, diff
venv/bin/python -m auto_battlebot.verify.hooks mutant --session <id>   # mutation worktree
```

`mutant` makes `.claude/worktrees/mutant-<hash>` at the base commit, copies in the current
touched test files (paths under a `tests/` or `test/` directory, `test_*`, `*_test.*`), and
symlinks the untracked `data/`, `third_party/`, and `web/node_modules/` content from the main
checkout. A new test that passes there does not test the change.

The verifier ends its final message with one line:

```
VERDICT {"hash": "<hash>", "result": "PASS" | "FAIL", "findings": [{"file", "line", "check", "detail"}]}
```

The verdict reaches `verdict-<hash>.json` two ways. The SubagentStop hook reads the verifier's
final message, or the last `SubagentHandback` message in `agent_transcript_path` when the final
message is empty (a background subagent reports through that tool, so `last_assistant_message`
arrives empty). The Stop hook also scans `<session transcript>/subagents/agent-*.meta.json` for
finished agents with `agentType` `verifier` and reads the VERDICT from their transcripts. Either
way, only a verdict naming the current hash counts.

While `background_tasks` in the Stop event lists a running verifier, the stop goes through
without a block, so waiting on a verifier does not use up the two blocks.
