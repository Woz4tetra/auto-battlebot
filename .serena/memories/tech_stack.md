# Tech stack

## C++
- Language server: clangd (`/usr/bin/clangd`). Needs `compile_commands.json`; a root
  symlink -> `build-test/compile_commands.json` exists (gitignored).
- Build: CMake. GoogleTest for tests.
- miniroscpp (NOT full ROS). ZED SDK, TensorRT, CUDA, DepthAI (OAK) for calibration.
- CMake globs sources but is NOT CONFIGURE_DEPENDS: re-run `cmake -S . -B build` after
  adding a new .cpp.

## Python
- Runs in the project venv at `venv/` (create with scripts/setup_python.sh, activate with
  `source scripts/activate_python.sh`). Do NOT use `uv run` or create uv.lock.
- Tooling in venv: ruff, mypy (+ dmypy). `pyproject.toml` has platform-conditional deps
  (x86_64 dev vs aarch64 Jetson) — do not flatten them.
- Analysis tools live in `training/`; shared code in the `auto_battlebot/` Python package.
  YOLO annotation format (not COCO). In-repo label editors, not Label Studio.

## Platforms
- Deploy: Jetson Orin Nano (aarch64, TensorRT 10, CUDA).
- Dev: Ubuntu 22/24 x86_64 with NVIDIA GPU.