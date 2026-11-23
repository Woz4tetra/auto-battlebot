# auto_battlebot (Python package)

`auto_battlebot/` is an installed package (`pip install -e .`), so its modules import as
`auto_battlebot.<subpackage>.<module>` from anywhere. `training/`, `scripts/`, `simulation/`,
`tests/`, and `logo/` are not packages.

## Where Python code goes

- **Shared library code goes in `auto_battlebot/`.** It is the only directory that is both
  importable and type-checked. Anything two callers need lives here. Its subpackages:

  | Subpackage | Holds |
  | --- | --- |
  | `recording/` | Foxglove MCAP read and write, ZED SVO, diagnostics loaders |
  | `perception/` | Python mirrors of the C++ detector path (`trt_yolo`, `camera_geometry`), camera and cage calibration loaders, field pose fits (`field_pose`), the sysid tag pose smoother, the batched opponent filter mirror (`opponent_filter`) |
  | `segmentation/` | DeepLab field-mask builder, checkpoint metadata, label parsing, fixed-camera floor hull (`field_hull`) |
  | `control/` | The grey-box drivetrain plant the C++ filter mirrors |
  | `mujoco_sim/` | Mr Stabs Mk2 rigid-body model: Onshape lumping, MJCF, MuJoCo Warp rollouts, the CMA-ES plant fit |
  | `eval/` | Detector scoring: GT loading, detectors, metrics, bootstrap, plots |
  | `calibration/` | `jig/`, `apriltag/`, `match/`, `sysid/`: the plant-fit paths and their CLIs |
  | `datasets/` | Cage-high video download and annotation, image cache cleanup |
  | `nav/` | Navigation field tests and their lockstep reproductions |
  | `rl/` | RL environments, policies, lockstep eval, checkpoint export, sensing-model fits |
  | `features/` | The feature registry, checks, and the feature map server |
  | `verify/` | `scripts/verify` suites, selection, playback tiers, Stop hook |
  | `tensorrt_build` | Engine builder scaffolding shared by both export CLIs |

- No Python library sources under `scripts/`. A recurring analysis CLI is a module in the
  `auto_battlebot/` subpackage it belongs to, with a `main()`, run as
  `python -m auto_battlebot.<pkg>.<module>`, and with a test. `scripts/` holds only top-level
  entry points (`scripts/verify`, `scripts/lint`, `scripts/mcap_auto_percentage.py`) and no
  `scripts/<area>/` subdirectories of Python. `auto_battlebot/tests/test_script_imports.py` fails
  on a `.py` under `scripts/*/`.
- CLIs write to `data/<area>/` (gitignored) by default, anchored at
  `auto_battlebot.paths.REPO_ROOT`. Data files a CLI reads (`waveforms.toml`,
  `jig_calibration.toml`) sit next to the module.
- Tests live in a `tests/` directory next to the source they test: `auto_battlebot/eval/tests/`,
  `training/yolo/tests/`, `scripts/tests/`, `firmware/mr_stabs_mk2/scripts/tests/`. Test data
  goes in `tests/data/` beside them. There is no top-level Python test directory; `tests/` at the
  root holds the C++ gtest sources and `tests/fixtures/`. Test file names stay unique across the
  tree, because pytest imports each `tests/` directory by basename. Reach the repo root with
  `auto_battlebot.paths.REPO_ROOT`, never `Path(__file__).parents[N]`.
- A web front end that a Python module serves goes in a `static/` directory beside that module
  (`auto_battlebot/calibration/sysid/static/`). A frontend with a build step keeps its source in
  `tools/<name>/` (`tools/CLAUDE.md`).
- Tests load a non-package script with `auto_battlebot.script_loader.load_script("training/x/y.py")`.
- Golden fixture generators live in `tests/fixtures/generators/`. Synthgen scene pipelines live in
  `training/synthetic/scenes/`. Promoted MuJoCo plant fits live in `training/rl/plant_fits/`.
- A one-off analysis runs from the agent's scratchpad, and its result goes in a dated doc under
  `docs/experiments/` with the script inline or a link to the commit that held it. Nothing lands
  in the tree without a second use and a test.
- `training/` is standalone scripts and is deliberately **not** a package, so a `training/`
  module can never be a shared dependency. Code that `scripts/` or `simulation/` needs from
  there moves to `auto_battlebot/` first.
- No `sys.path` manipulation to reach first-party code, and therefore no `# noqa: E402`. The
  only legitimate `sys.path` writes are for build artifacts under `build/` (import them lazily
  inside the function that needs them) and for Blender's embedded Python, which ignores
  `PYTHONPATH`.

Adding a new package directory means adding it to `[tool.setuptools.packages.find]` in
`pyproject.toml` and re-running `pip install -e .`.
