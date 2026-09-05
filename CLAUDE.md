# auto-battlebot

Autonomous aim-assist and control system for NHRL combat robot competitions. Runs on a Jetson Orin Nano with a ZED 2i stereo camera. End-to-end latency target: under 60ms.

## Build

```bash
./scripts/build.sh                                         # release build, output: build/
./scripts/build_and_test.sh                                # debug + GoogleTest, output: build-test/
./scripts/build_and_test.sh --gtest_filter=KeypointTest.*  # run specific tests
./scripts/clean_build.sh                                   # remove build artifacts
```

## Run

```bash
./scripts/build_and_run.sh -c config/playback.toml  # replay SVO recording, no hardware needed
./scripts/run_simulation.sh                          # simulation mode
./scripts/build_and_run.sh -c config/main.toml       # full hardware mode
```

Use playback mode for development and regression testing.

## Formatting

Run before committing:

```bash
./scripts/lint          # apply formatters, then run all lint/type checks
./scripts/lint --quick  # skip the slow checks (clang-tidy, mypy)
```

- C++: Google style, 4-space indent, 100-char line limit (`.clang-format`)
- The clang toolchain is pinned to the major version in `.llvm-version` — the
  unversioned distro packages differ per release (14 on 22.04/JetPack, 18 on
  24.04) and reformat the tree inconsistently. Every platform install sets it
  up; to repair it alone run `./install/install_llvm_toolchain.sh`, and
  `scripts/lint` uses the versioned
  binaries and skips the step if they are absent.
- Python: `ruff`
- TOML: `taplo`

## Python

Python scripts run in the project virtual environment at `venv/`. Activate it before running any Python:

```bash
source scripts/activate_python.sh   # activates venv/ (create it first with scripts/setup_python.sh)
```

Do not use `uv run` or create a `uv.lock`. The venv is the intended environment for `scripts/`, `playground/`, `training/`, and `simulation/`.

### Where Python code goes

`auto_battlebot/` and `playground/` are installed packages (`pip install -e .`), so their
modules import as `auto_battlebot.<module>` and `playground.<subpackage>.<module>` from
anywhere. `training/`, `scripts/`, `simulation/`, and `logo/` are not packages.

- **Shared library code goes in `auto_battlebot/`.** It is the only directory that is both
  importable and type-checked. Anything two callers need lives here, `playground/calibration/`
  drivers included (`auto_battlebot/calibration/`).
- `playground/` holds runnable analysis scripts: one CLI per file. It is formatted and linted
  by ruff like the rest of the tree, but stays out of mypy. When a helper in there grows a
  second caller, move it to `auto_battlebot/` rather than importing across script directories.
- `training/` is standalone scripts and is deliberately **not** a package, so a `training/`
  module can never be a shared dependency. Code that `playground/` or `simulation/` needs from
  there moves to `auto_battlebot/` first.
- No `sys.path` manipulation to reach first-party code, and therefore no `# noqa: E402`. The
  only legitimate `sys.path` writes are for build artifacts under `build/` (import them lazily
  inside the function that needs them) and for Blender's embedded Python, which ignores
  `PYTHONPATH`.

Adding a new package directory means adding it to `[tool.setuptools.packages.find]` in
`pyproject.toml` and re-running `pip install -e .`.

## GPU training queue

Several agents share the three A6000s. Every training arm runs DDP across all three, so
the box runs one job at a time. Do not launch training directly; submit it:

```bash
venv/bin/python training/gpu_queue.py submit --name B_s384x640 --by <agent> -- \
  venv/bin/python training/yolo/train.py training/data/nhrl_robots_bbox_2class yolo26s \
  -d 0 1 2 -b 96 -e 100
venv/bin/python training/gpu_queue.py status          # --json for parsing
venv/bin/python training/gpu_queue.py logs 3 --tail 40
```

`submit` starts the worker if none is running and sets `NCCL_P2P_DISABLE=1` for
multi-GPU jobs. The worker waits for the GPUs to go idle before each job, so a run
started outside the queue delays it rather than colliding with it. Check `status`
before submitting, and never kill a job you did not submit -- use `cancel <id>`.

State and logs live in `runs/queue/` (gitignored). Use `--priority` to put a short
scoring or export job ahead of a queued multi-hour train.

## Architecture

Config-driven factory pattern. The active TOML config selects which implementation of each interface gets instantiated at startup. Main loop in `Runner`:

```
camera -> perception (field mask + robot blobs + keypoints) -> filter -> target selection -> navigation -> transmit
```

All code is in namespace `auto_battlebot`. Interfaces live in `include/<module>/`, implementations in `src/<module>/`, factories wire them together based on config.

## Conventions

- New component: add interface to `include/<module>/`, implementation to `src/<module>/`, register in the factory
- Prefer TOML config over compile-time switches for behavior changes
- No full ROS. The project uses `miniroscpp` intentionally. Do not add `package.xml` or full ROS dependencies
- Compiler flags are `-Wall -Wextra -Werror`. Fix warnings, do not suppress them

## Testing

Tests use GoogleTest in `tests/`. Build with `build-test/` (debug + `BUILD_TESTING=ON`). Playback mode with SVO recordings is the primary way to regression-test without hardware.

## Validation

Before reporting a feature complete, run:

```bash
# Formatting, C++/Python lint, and Python type checking, all in one pass.
# Needs build/ to exist first (scripts/build.sh) for the clang-tidy step.
./scripts/lint
```

This applies formatters, then runs `clang-tidy` (C++ lint over `src/`), `ruff check`
(Python lint), and `mypy` (types; scope set by `mypy.ini`). Use `--dry-run` to check
without modifying files and `--quick` to skip the slow checks.

Hooks run `./scripts/lint --quick` automatically on every stop.

## Writing

Applies to everything generated in this repo: reports, analysis write-ups,
commit messages, PR bodies, docs, and code comments. Adapted from the
`no-ai-slop` skill; `scripts/no_ai_slop_check.sh` enforces the wordlist
automatically on write, and the rest is on you.

Preserve the author's point and voice while making the writing clearer. Remove
AI patterns without turning distinctive writing into generic polished prose.

### Principles

- **Make the minimum effective edit.** Fix AI patterns, errors, repetition, and
  unclear passages. Leave strong sentences alone.
- **Lead with the point when the setup adds nothing.** Cut generic
  throat-clearing. Keep an aside or admission when it creates real context.
- **Keep the meaning.** Don't invent claims, examples, stats, or opinions. If
  something is unclear, ask.
- **Open it up, don't dumb it down.** Keep the substance, nuance, and precision.
  Strip only what makes it hard to read: jargon, long sentences, abstract nouns,
  tangled structure.
- **Use active voice.** "The team shipped it Tuesday" beats "the decision
  emerged." Never let inanimate things do human verbs.
- **Be concrete and specific.** "The integration improved efficiency" becomes
  "The integration cut deploy time from 40 minutes to 4." Names, numbers, dates,
  mechanisms, and examples beat abstractions.
- **Protect the specific fact.** Don't smooth a useful detail into generic
  importance. This matters most in eval and replay write-ups: report the metric
  and the delta, not that a change was significant.
- **Make verbs do the work.** "Made a decision" becomes "decided." "Has the
  ability to" becomes "can."
- **Make every sentence earn its place.** Cut empty qualifiers. Keep "I think"
  or "maybe" when they express real uncertainty.
- **Untangle sentences without flattening the cadence.** Split genuinely hard
  sentences. Keep fragments and changes in pace that are clear and characteristic.
- **Preserve useful edge.** Keep strong opinions, blunt language, humor, and
  honest admissions. Don't replace them with safer wording.

### Words to cut

Banned outright: delve, foster, leverage, utilize, facilitate, empower,
streamline, cutting-edge, paradigm shift, game changer, this is huge, this
changes everything, tapestry, realm, beacon, multifaceted, meticulous, intricate,
paramount, transformative, elevate, embark, supercharge, ever-evolving.

The upstream skill also bans "robust" and "harness"; both are legitimate
technical terms here (robust estimator, wiring harness, test harness) and are
exempt in this repo.

Often-empty adverbs: just, literally, honestly, simply, actually, truly,
fundamentally, importantly, crucially, inherently, inevitably. Cut them when they
add nothing. Keep them when they carry emphasis, uncertainty, or contrast.

Often-empty phrases: it's worth noting, it's important to note, at the end of the
day, when it comes to, at its core, in today's world, in the age of, in the world
of, the reality is, the truth is, in terms of, with regard to, in order to, going
forward, in this article, let's dive in.

### Patterns to cut

**Binary contrasts.** "This is not X. It's Y." / "The question isn't X, it's Y."
State Y directly. "The question isn't the model. It's the eval." becomes "The
eval matters more than the model."

**Throat-clearing openers.** "Here's the thing," "Let me be clear," "I'll be
honest," "The uncomfortable truth is." Cut them and state the point.

**Faux-insight setups.** "What most people get wrong," "Here's what nobody tells
you," "The part everyone misses." Cut the setup and let the claim stand alone.

**Colon reveals.** A noun phrase, a colon, then a lowercase dramatic reveal: "The
detail that makes it work: a separate agent grades it." Rewrite as a plain
sentence. Use colons for lists, labels, and quotes, not fake drama.

**Superficial analysis.** Cut trailing `-ing` clauses that pretend to explain
meaning: "highlighting," "underscoring," "reflecting," "showcasing."

**Importance puffery.** "Stands as a testament," "marks a pivotal moment," "plays
a vital role," "underscores its significance." State the fact and let the reader
judge whether it matters.

**Weasel attribution.** "Experts agree," "studies show," "widely regarded as."
Name the source or cut the claim. Never invent one. In this repo the source is
usually a specific run, recording, or commit -- cite it.

**Fake-strong verbs.** Prefer "is" and "has" when they are clearer. "The app
serves as a centralized hub for X" becomes "The app tracks X."

**Synonym cycling.** If the clear word is right, repeat it. Don't rotate terms
for style, and never rotate a term that names a component in this codebase.

**Negative listing.** "Not a X. Not a Y. A Z." Just say Z.

**Dramatic fragmentation.** "X. And Y. And Z." or "That's it. That's the whole
thing." Use complete sentences.

**Robotic rhythm.** Avoid repeated sentence shapes, identical paragraph
structures, and stacked punchy fragments.

**Rhetorical setups.** "What if I told you...", "Think about it:", "Plot twist:",
and self-answered "Question? Answer." pairs. Drop them and make the point.

**Fake-profound kickers.** Cut the final "deep" line that turns the point into a
metaphor or mic-drop. Don't rewrite it into a better metaphor -- delete it and
end on the clearest concrete sentence already present.

**Summary-recap endings.** "In conclusion," "Ultimately," "Overall," or a final
paragraph restating the piece. End on the last concrete point or next action.

**Formatting slop.** Emoji in headings, bold sprinkled mid-sentence, bullet lists
where two sentences of prose would read better, headers over two-sentence
sections. Format should follow the content, not decorate it.

**Em dashes.** Not a default rhythm crutch. None in short copy; 1-2 in longer
drafts if they clearly beat commas, periods, or parentheses.

## Platforms

- Deployment: Jetson Orin Nano (aarch64, TensorRT 10, CUDA)
- Dev: Ubuntu 22/24 x86_64 with NVIDIA GPU
- `pyproject.toml` has platform-conditional deps. Do not flatten them.

## What to avoid

- No blocking calls in the main perception loop. The latency budget is tight.
- Do not modify files under `data/` (MCAP recordings, SVO files, TensorRT engines).
