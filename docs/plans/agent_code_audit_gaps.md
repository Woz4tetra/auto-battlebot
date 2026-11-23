# Agent code audit gaps

Practices for auditing agent-written code that this repo does not have yet. Compiled 2026-11-22 by
checking the repo against three write-ups:

- [A grounded take on agentic coding](https://iximiuz.com/en/posts/grounded-take-on-agentic-coding/) (Ivan Velichko)
- [How Claude Code works in large codebases](https://claude.com/resources/articles/how-claude-code-works-in-large-codebases-best-practices-and-where-to-start) (Anthropic)
- [Agentic Code Review](https://addyosmani.com/blog/agentic-code-review/) (Addy Osmani)

Already in place, so not listed: a real dev environment for end-to-end runs (playback, sim, lockstep
and real-time tiers), checks that run before review (Stop hook plus `scripts/verify --fast
--changed`), an independent verifier subagent, the comment budget, and the workaround ledger.

The tree is about 220,000 lines of C++, Python, TypeScript and Svelte across 2,395 tracked files.

Order: 1, 2 and 5 first. 1 and 2 close holes the verifier passed over on 2026-11-22; 5 is a
settings change.

## 1. Mutation checks that work on new files

- Issue: `hooks.make_mutant` reverts the change's non-test files to the session's base commit and
  expects the new tests to fail. A new file has no base version, so the tests fail on import and
  the check passes vacuously. All three verifier runs on the 2026-11-22 feature map work reported
  this for `tools/feature_map/` and `auto_battlebot/features/`.
- Fix: mutate the diff's added lines instead of reverting files: flip a comparison, drop a
  statement, swap a constant. Require at least one test to fail per mutant. Run one mutant at a
  time under a 6 GB memory cap; a batched run has run out of memory and killed the login. Candidates: mutmut for Python,
  Stryker for TypeScript, a small line mutator for C++.
- Done when: the verifier reports a killed/survived count per changed file, and a deliberately
  toothless test on a new file fails the verdict.

## 2. Flag weakened tests

- Issue: nothing in `auto_battlebot/verify/hooks.py` or `.claude/agents/verifier.md` looks for
  removed assertions, new skips, loosened tolerances, or deleted test files. Agents rewrite
  assertions to match broken behavior.
- Fix: a lint step over the diff of every test file (`tests/`, `*/tests/`, `*.test.ts`) that
  counts assertions (`assert`, `EXPECT_`/`ASSERT_`, `expect(`) before and after. It fails on a drop,
  on a new `pytest.skip`/`xfail`/`DISABLED_`/`it.skip`, on a widened `approx`/`toBeCloseTo`/
  `EXPECT_NEAR` tolerance, and on a deleted test file, unless the commit message names the test.
- Done when: a test fixture with each of those edits fails the step, and the verifier reads its
  output first.

## 3. Checks outside agent sessions

- Issue: there is no `.github/workflows/`. The Stop hook and verifier run only when an agent
  stops, and `scripts/git-hooks/pre-commit` runs only `comment_budget.py check --staged`. Commits
  made by hand or pushed from another machine skip gtest, pytest, playback and lint.
- Fix: a `pre-push` hook in `scripts/git-hooks/` that runs `scripts/verify --fast --changed`
  against the upstream branch. Add CI later if a second contributor appears.
- Done when: pushing a commit with a failing pytest is refused locally.

## 4. Layered CLAUDE.md

- Issue: the root `CLAUDE.md` is 458 lines and no subdirectory has one. Every session loads the
  GPU queue, firmware lint and Blender Python rules whatever it touches.
- Fix: keep only pointers and cross-cutting rules at the root (build, verify, comments, writing,
  what to avoid). Move local rules beside their code:
  - `auto_battlebot/CLAUDE.md`: where Python code goes, the subpackage table, test layout
  - `training/CLAUDE.md`: the GPU training queue
  - `firmware/CLAUDE.md`: firmware formatting and PlatformIO notes
  - `tools/CLAUDE.md`: the standalone tool layout
  - `src/CLAUDE.md`: config struct, enum and thread-safety conventions
- Done when: the root file is under 150 lines and each moved section lives in exactly one file.

## 5. Hard guards on protected paths

- Issue: "Do not modify files under `data/`" is prose only. `.claude/settings.json` has no
  `permissions.deny`. On 2026-11-21 an empty `$f` in `mv data/recordings/$f` moved the whole
  recordings directory.
- Fix: add `permissions.deny` for `Write` and `Edit` under `data/`, `third_party/`,
  `firmware/BlHeli-Passthrough/` and generated bundles (`auto_battlebot/features/static/`). Add a
  `bash-guard` rule that blocks `mv`, `rm` and redirects whose target is under `data/`.
- Done when: an Edit to `data/` and `mv data/recordings/ x` are both refused in a session.

## 6. Review depth by risk

- Issue: every change under `src/`, `include/`, `firmware/`, `auto_battlebot/` and `tests/` gets
  the same verifier pass. A transmit-path or firmware change carries far more risk than a dataset
  CLI.
- Fix: a path-to-tier table read by `hooks.stop`. High-risk paths (`src/transmitter/`,
  `src/navigation/`, `src/control_loop/`, `src/command_shaper/`, `firmware/`) also need a
  `scripts/verify --playback` pass and a "safety-relevant" line in the verdict. Low-risk paths
  (`auto_battlebot/datasets/`, `docs/`, `tools/`) skip the verifier when the suites pass.
- Done when: the Stop hook names the tier, and a firmware edit cannot clear without a playback run.

## 7. A second, differently built reviewer

- Issue: the verifier is the same model family that wrote the code, so it shares its blind spots.
- Fix: on high-risk tiers only, add one independent pass: semgrep or CodeQL rules for C++ and
  Python, or a review from another vendor's model. Its output feeds the verifier as findings to
  confirm or reject, never as the verdict.
- Done when: a high-risk change shows the second reviewer's findings in the verifier's report.

## 8. Backward trace from code to features

- Issue: the feature map checks that each reach point has a feature, but nothing lists code that
  no feature's proofs cover. Agent-added code with no feature behind it is dead code or scope
  creep.
- Fix: from the per-proof coverage the map already loads, list every `src/`, `auto_battlebot/` and
  `firmware/` file that no proof touches. Show it in the map and in `scripts/feature_map check` as
  a report-only count.
- Done when: the map shows the uncovered file list, and a new unreached file appears in it.

## Next steps

1. Build item 5 (settings and `bash-guard` rule).
2. Build item 2 (weakened-test lint) with fixtures for each edit kind.
3. Replace the revert mutant with line mutation (item 1), Python first.
