# auto-battlebot

Autonomous aim-assist and control system for NHRL combat robot competitions. Runs on a Jetson Orin NX (JetPack 7.2) with a ZED 2i camera. End-to-end latency target: under 60ms. Recordings, keypoint corpora, and eval ground truth are tied to ZED SVO replay.

Area rules load when a session reads files there: `auto_battlebot/` (Python layout), `src/` and
`include/` (C++), `training/` (GPU queue), `firmware/`, `tools/`, `docs/` (full writing rules).

## Build

```bash
./scripts/build.sh                                         # release build, output: build/
./scripts/build_and_test.sh                                # debug + GoogleTest, output: build-test/
./scripts/build_and_test.sh --gtest_filter=KeypointTest.*  # run specific tests
./scripts/clean_build.sh                                   # remove build artifacts
```

## Run

Configs `extends` `config/_common.toml`; replay profiles and `recordings.toml` are in `config/playback/`.

```bash
./scripts/build_and_run.sh -c config/playback/mr_stabs_mk2_playback.toml --recording <name>
./scripts/build_and_run.sh --list-recordings          # catalog, and which files this machine has
./scripts/run_simulation.sh                           # simulation mode
./scripts/build_and_run.sh -c config/mr_stabs_mk2_fid_zed_box.toml  # full hardware mode
```

## Python

- Run Python in `venv/`: `source scripts/activate_python.sh` (create it with
  `scripts/setup_python.sh`). Do not use `uv run` or create a `uv.lock`.
- Shared library code goes in `auto_battlebot/`, an installed package and the only directory that
  is both importable and type-checked. `training/`, `scripts/`, `simulation/`, `tests/`, and `logo/`
  are not packages. Layout rules: `auto_battlebot/CLAUDE.md`.
- Do not launch training directly. Submit it to the GPU queue (`training/CLAUDE.md`).

## Architecture

Config-driven factory pattern. The active TOML config selects which implementation of each interface gets instantiated at startup. Main loop in `Runner`:

```
camera -> perception (field mask + robot blobs + keypoints) -> filter -> target selection -> navigation -> transmit
```

All code is in namespace `auto_battlebot`. Interfaces live in `include/<module>/`, implementations in `src/<module>/`, factories wire them together based on config.

- No ROS. Live viz and recordings use Foxglove schemas over the Foxglove SDK
  (`docs/reference/recording/foxglove_recording_format.md`). Do not add ROS dependencies
- Prefer TOML config over compile-time switches for behavior changes

## Comments

Don't write code comments. That covers `//`, `/* */`, `#`, TOML comments, and Python docstrings
longer than one line. One-line docstrings are allowed and do not count.

- Reason for a change: the commit message
- Measurement, eval result, fitted-value provenance: a dated doc under `docs/experiments/<topic>/`
- Invariant: a test, or a type or name that makes the wrong use impossible
- A block that needs explaining: a named function
- History ("previously", "no longer") and plan checkpoints: nowhere
- A workaround that needs a comment to survive review needs a test or a fix instead
- A comment moved into a doc is still a comment. Docs hold measurements and how the code works
  now, never a defense of a specific line. No `Workarounds` sections

Exempt forms (shebangs, `#pragma`, `// NOLINT(check)`, `# noqa: CODE`, format toggles, license
headers) and the `DATASHEET:` exception are listed in `docs/reference/tools/comment_budget.md`.
`scripts/comment_budget.py` enforces the rule against `.comment-baseline.json`; a file not listed
has a budget of 0, and a PostToolUse hook checks every edited file.

- Never add, raise, or edit a `.comment-baseline.json` entry without the user's explicit approval
  in this conversation. Propose the comment and wait. A test, a type, or a fix comes first
- There is no opt-out marker and no `--allow` flag. Move files with `git mv` to keep the budget

## Testing

C++ tests are GoogleTest in `tests/`, built in `build-test/`. Playback is the primary regression test without hardware.

`scripts/verify` runs every suite: gtest, pytest (every `tests/test_*.py` under the `testpaths` in
`pyproject.toml`), synthetic (`training/synthetic/`), firmware native (`pio test -e native`), and
the web check and vitest. Details and the file-to-suite map: `docs/reference/tools/verify.md`.

```bash
scripts/verify                        # every suite, slow tests included (about 2 min)
scripts/verify --fast                 # lint --quick first, then every suite without slow tests
scripts/verify --changed a.cpp b.py   # only what these paths affect (--list to print them)
scripts/verify --suite gtest          # only the named suites (gtest, pytest, synthetic, firmware, web)
scripts/verify --playback             # lockstep clips, hashed against the baseline (about 2 min)
scripts/verify --realtime             # real-time clips against A/A bounds (about 3 min)
```

## Validation

Before committing or reporting a feature complete, run `./scripts/lint` (formatters, then
clang-tidy, `ruff check`, `mypy`; `--quick` skips the slow two, `--dry-run` changes nothing). It
needs `build/` for clang-tidy. Formatters: `.clang-format` (C++), `ruff`, `taplo`.

The Stop hook runs `scripts/verify --fast --changed` (lint, then affected suites) over every file
that differs from the commit the session started on (git diff plus untracked files), and a
failure blocks the stop. It skips when every changed file is a `.md` or an image. A change under `src/`, `include/`, `firmware/`, `auto_battlebot/`, or
`tests/` also needs a PASS from the `verifier` agent (`.claude/agents/verifier.md`); the blocked
stop says which session id and change hash to give it, along with the user's request verbatim.

While a verifier runs, do not edit files. Any edit changes the change hash, so the verifier
re-reads a moving diff and its verdict lands on a hash the Stop hook no longer asks about.
Wait for the verdict, then edit, then spawn a new verifier for the new hash. The verifier runs
every command in the foreground; a PreToolUse hook (`hooks bash-guard`) blocks background and
detached commands for it.

## Docs layout

`docs/README.md` indexes every folder. A new doc goes in the first folder that fits:

- `docs/guides/`: steps you follow start to finish
- `docs/reference/<subsystem>/`: how the code works now, no results or history
- `docs/setup/`: how a machine or service is configured
- `docs/experiments/<topic>/YYYY-MM-DD_<name>.md`: a measurement or result
- `docs/plans/`: active plans only. When a plan is done, move its lasting facts to `reference/` and delete it

Nothing new goes at the top of `docs/`. Move docs with `git mv` and update every path that cites them.

## Writing

Applies to everything generated in this repo: reports, commit messages, PR bodies, and docs. The
full rules and word list are in `docs/CLAUDE.md`; `scripts/no_ai_slop_check.sh` enforces the word
list on write.

- Lead with the point. Use active voice. Make every sentence earn its place.
- Be concrete: names, numbers, dates, mechanisms. Report the metric and the delta, and cite the
  run, recording, or commit.
- Keep the author's meaning, voice, and edge. Don't invent claims, examples, or stats.
- Cut binary contrasts ("not X, it's Y"), colon reveals, throat-clearing openers, `-ing` analysis
  tails, importance puffery, rhetorical setups, kicker endings, and summary recaps.
- No em dashes in short copy, and none of the banned words listed in `docs/CLAUDE.md`.

## Platforms

- Deployment: Jetson Orin NX on JetPack 7.2 / L4T R39.2 (aarch64, Ubuntu 24.04, Python 3.12, CUDA 13.2, TensorRT 10.16) or JetPack 6.2.1 / L4T R36.4 (Ubuntu 22.04, Python 3.10, CUDA 12.6, TensorRT 10.3), the only JetPack 6 release `install/check_jetson_orin.sh` accepts. The ZED Box Mini and the CI Jetson run it (`docs/setup/zed_box_mini.md`). The install scripts branch on the L4T major and take the venv's Python version from `/usr/bin/python3`, because apt builds `python3-libnvinfer` for the system python only
- Python floor is 3.10 for JetPack 6. Import it as `from auto_battlebot.compat import tomllib`, and use no 3.11+ stdlib API outside `training/synthetic/` (Blender's 3.11). ruff and mypy target 3.10 and catch most of it
- `BUILD_WITH_ZED` defaults to whether the ZED SDK is installed: ON on the ZED Box Mini and dev machines (SVO playback)
- TensorRT engines are not portable across TensorRT versions. The `aarch64_sm87` engines must be built on each box's own TensorRT (10.16 on JetPack 7, 10.3 on JetPack 6); the filename is unchanged by a rebuild, so old and new look identical on disk
- Dev: Ubuntu 22/24 x86_64 with NVIDIA GPU
- `pyproject.toml` has platform-conditional deps. Do not flatten them.

## What to avoid

- No blocking calls in the main perception loop. The latency budget is tight.
- Do not modify files under `data/` (MCAP recordings, SVO files, TensorRT engines).
- `runs/` holds model training outputs only. Put analysis results, scratch output, and anything else somewhere else.
