# Build notes

## Configure once

`install/build_cpp_project.sh` runs `cmake` configure only when `Makefile` is missing. After that, the generated Makefiles contain a `cmake_check_build_system` hook that re-runs cmake when `CMakeLists.txt` or any cmake file changes. Running cmake unconditionally regenerates internal files (`compiler_depend.make`, `Makefile2`) on every invocation, which causes spurious "Built target" output and cmake overhead even when nothing changed.

The guard is on the generated `Makefile`, not `CMakeCache.txt`. cmake writes the cache early in configure but only emits the `Makefile` once generation completes. If a prior configure was interrupted, the cache exists without a `Makefile`. Guarding on the cache would skip configure and fail at `make` with "No targets specified and no makefile found". Guarding on the `Makefile` re-runs cmake to finish the interrupted configure.

## Without the ZED SDK

The source glob in `CMakeLists.txt` is recursive, so `BUILD_WITH_ZED=OFF` filters the ZED backends out instead of wrapping an explicit list. `PointCloudFieldFilter` goes with them: it needs a depth image, and outside simulation the ZED is the only camera that produces one.

## PCL

PCL is only reachable from `PointCloudFieldFilter` and `data_structures/point_cloud.hpp`. It stays unconditional while `FieldDescriptionWithInlierPoints` carries a cloud. It goes when the depth field filter does.

## Web dashboard

`scripts/build.sh` builds the web dashboard that `viz_relay` serves, through `scripts/build_web.sh`. It is optional. Without Node the build skips it and the relay answers 503 with a note.

## CMake dependencies

- Crow and Asio (web dashboard HTTP server) are header-only and linked to `viz_relay` only.
- nlohmann_json (remote protocol, `include/remote/`) comes from the release's single-header URL tarball, which is far smaller than the repository.
- mcap ships no CMake target, only headers plus a single-translation-unit implementation define.
- CUDA sources (the GPU preprocess kernels): nvcc comes from the toolkit that FindCUDA picked, since `/usr/local/cuda/bin` is not on `PATH` on the Jetsons. The kernels must be built by the same toolkit whose static runtime FindCUDA links (`CUDA_USE_STATIC_CUDA_RUNTIME`), so the process has one CUDA runtime, shared by TensorRT's stream and the kernels.
- Foxglove SDK: the C++ wrapper sources compile in this tree on top of the prebuilt Rust static library, which `install/install_foxglove_sdk.sh` unpacks into `third_party/foxglove`.
