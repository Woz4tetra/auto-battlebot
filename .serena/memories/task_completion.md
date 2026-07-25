# Task completion checklist

Before reporting a feature complete:

## Python
- `venv/bin/mypy scripts/ simulation/ training/`
- `ruff check` (also runs automatically via the Stop hook)

## C++
- Build test target: `./scripts/build_and_test.sh` (build-test/ must exist for the checks
  below).
- Static analysis on modified files:
  `git diff --name-only HEAD | grep '\.cpp$' | xargs -r clang-tidy-18 -p build-test/`
  Use the versioned binary — the clang toolchain is pinned to `.llvm-version`.
- clang-format --dry-run runs automatically via the Stop hook.

## Regression
- Playback mode with an SVO recording is the primary hardware-free regression path. Run a
  `*_desktop.toml` config against a recording rather than trusting a green build alone.
- Analyze real Jetson recordings for perception fidelity; do not substitute laptop SVO
  re-runs.

## Formatting
- `./scripts/apply_formatting` — but only on files you changed (shared working tree).