# Python packaging and lint config

`pyproject.toml` is read by pip, setuptools, ruff and mypy. The sections below explain the entries that need a reason.

## `[project]`

| Field | Meaning |
| --- | --- |
| `requires-python = ">=3.10"` | 3.10 is JetPack 6's system python, the only one its TensorRT bindings are built for. |

## `[project] dependencies`

| Dependency | Meaning |
| --- | --- |
| `numpy` (two lines) | `>=1.26,<2` on aarch64 with Python below 3.11, because JetPack 6's NVIDIA PyTorch wheel is built against the NumPy 1 ABI and that is the only box matching. JetPack 7 runs Python 3.12 and takes NumPy 2 wheels from the upstream CUDA index. x86 dev machines run 3.11+ and get NumPy 2. |
| `pytest` | Runs `training/synthetic/tests` (synthgen pure modules). |
| `pycocotools` | Backend for torchmetrics `MeanAveragePrecision`, used by `training/model_eval`. |
| `av` | H.264 decode for `/camera/video`, used by `auto_battlebot.recording.mcap_io`. |
| `pyserial` | `auto_battlebot/calibration/jig/velocity_jig_drive.py` drives the OpenTX trainer link with it for drivetrain characterization. |
| `flask` | `firmware/espnow_dongle/scripts/espnow_diag.py` serves the ESP-NOW link diagnostics page with it. |
| `Pillow` | `auto_battlebot/calibration/apriltag/make_print_tags.py` exports print-accurate AprilTag PDFs with it. |
| `pycuda`, `torch`, `tensorrt` | x86_64 only. They are often unavailable or different on Jetson (aarch64). |
| `mujoco`, `mujoco-warp`, `warp-lang`, `cma`, `coacd` | x86_64 only. `auto_battlebot/mujoco_sim` fits the Mr Stabs Mk2 rigid body on the dev GPUs. Pinned because the MuJoCo Warp API still changes between releases. |

## `[tool.setuptools.packages.find]`

`include = ["auto_battlebot*"]`: `auto_battlebot` is the only installed package.

## Tooling scope

ruff lints and formats the whole tree. `mypy.ini` excludes `scripts/{calibration,datasets,eval,nav,rl,sim_assets}/`, the analysis CLIs moved there from the old `playground/` tree. Anything two callers share belongs in `auto_battlebot/`, which mypy checks.

## `[tool.ruff]`

| Field | Meaning |
| --- | --- |
| `per-file-target-version` for `training/synthetic/**` | `py311`. Blender-side synthetic scripts run on Blender's bundled 3.11, never in the project venv. |
| `lint.ignore` `E402` | Module level import not at top of file. |
| `lint.ignore` `Q003` | Avoidable escaped quote. |
| `lint.unfixable` `F401` | Unused imports are reported, not auto-removed. |
| `lint.unfixable` `F841` | Unused variables are reported, not auto-removed. |

## `[tool.ruff.lint.per-file-ignores]`

| Glob | Meaning |
| --- | --- |
| `scripts/*`, `simulation/*.py`, `training/**/*.py` (`INP`) | Top-level scripts are run directly, not imported as a package. |
| `tests/fixtures/generators/*.py` (`INP`) | Golden fixture generators are run directly. |
| `hardware/**/*.py` (`INP`) | CAD part files, loaded by the build123d-part check script rather than imported. |
| `scripts/{calibration,datasets,eval,nav,rl,sim_assets}/*.py`, `training/synthetic/scenes/**/*.py` (`C901`) | Analysis scripts are one long linear `main()` per file that reads a recording and writes plots. Splitting them to satisfy the complexity limit costs more than it buys. Library code they call does not get this exemption; it lives in `auto_battlebot/`. |

## mypy

`mypy.ini` sets `ignore_errors` for `auto_battlebot.mujoco_sim.warp_kernels`: its kernel annotations are calls such as `wp.array(dtype=float)` that warp evaluates at runtime.
