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

## CMake dependency workarounds

- Crow and Asio (web dashboard HTTP server) are header-only and linked to `viz_relay` only. They are populated with `SOURCE_SUBDIR` pointing at a directory with no `CMakeLists.txt`, which skips Crow's own `find_package` for Asio. The `viz_relay` target wires the include paths itself.
- nlohmann_json (remote protocol, `include/remote/`) comes from the release's single-header URL tarball, which is far smaller than the repository. `DOWNLOAD_EXTRACT_TIMESTAMP` needs CMake 3.24. JetPack 6 ships 3.22, which reads the unknown keyword as more URL entries and fails with "At least one entry of URL is a path", so the option is passed only on 3.24 and newer.
- mcap ships no CMake target, only headers plus a single-translation-unit implementation define.
- RTNeural runs `RlPolicyNavigation`'s network and is included only by `src/rl_policy/runtimes/rtneural_runtime.cpp`. It is used header-only with the Eigen backend against system Eigen 3, so its own `CMakeLists.txt` is never added: that file puts its bundled Eigen copy on the include path. `SOURCE_SUBDIR` names a directory with no `CMakeLists.txt`, which makes `FetchContent_MakeAvailable` populate without `add_subdirectory`. No submodules are fetched, since xsimd is for a backend this build does not use. `SYSTEM` keeps its headers out of `-Werror`.
- ZED: when `BUILD_WITH_ZED` is on and `find_package` found nothing, the file calls `find_package(ZED 5 REQUIRED)` again so the error names the missing SDK.
- CUDA sources (the GPU preprocess kernels): nvcc comes from the toolkit that FindCUDA picked, since `/usr/local/cuda/bin` is not on `PATH` on the Jetsons. The kernels must be built by the same toolkit whose static runtime FindCUDA links (`CUDA_USE_STATIC_CUDA_RUNTIME`), so the process has one CUDA runtime, shared by TensorRT's stream and the kernels.
- Foxglove SDK: the C++ wrapper sources compile in this tree on top of the prebuilt Rust static library, which `install/install_foxglove_sdk.sh` unpacks into `third_party/foxglove`. The SDK's own headers trip `-Wdeprecated-declarations` under C++20, so they are system headers and the wrapper compiles with warnings off. Nothing of ours is in it.
- There is no `-DDEBUG` in the debug flags: nothing in the tree tests it, and the macro collides with `foxglove::schemas::Log::LogLevel::DEBUG` in the SDK headers.
- nvcc rejects the GCC warning flags, so `.cu` files pass them through to the host compiler and promote nvcc's own warnings separately. The CUDA flags carry no `-O`: `CMAKE_BUILD_TYPE` already passes one and nvcc warns on a second. ptxas optimizes device code regardless of the host `-O` level.
- `NvjpgJpegEncoder` loads its plugin from the directory `auto_battlebot_lib` is in, so the plugin builds beside the library.
