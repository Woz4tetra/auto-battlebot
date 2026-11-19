# Install notes

Facts behind the choices in `install/*.sh` and `CMakeLists.txt`.

## Install script contract

`install/fail_loudly.sh` is sourced first from every install entry point. A failing command then prints its file, line, and text before the script exits, instead of `set -e` dropping back to the prompt in silence.

- `-E` carries the ERR trap into functions, sourced files, and command substitutions.
- The ERR trap only records the location. The EXIT trap prints it, since EXIT fires on every exit path.
- Install functions must not set their own traps or change shell options for the caller. `scripts/lint` rejects both in its "install scripts" step (`scripts/check_install_option_leaks.sh`). Helpers may repeat `set -e`. A `set -o pipefail` that leaked from `install_opencv` once killed the Jetson venv step with no message. `install/install_opencv.sh` is exempt because its function body is a subshell.

## OpenCV

`install/install_opencv.sh` builds OpenCV from source into `/usr/local`, with resume support. Every platform gets 4.14.0.

- `cv::aruco` moved from the contrib module `opencv_aruco` into core `opencv_objdetect` at 4.7, and the API changed with it. `FiducialFieldFilter` needs 4.7 or newer everywhere, or it needs version guards it would never shed. apt ships 4.5.4 on jammy and 4.6.0 on noble, both on the wrong side of that break.
- 4.14.0 rather than the 4.10.0 that JetPack 6 produced: OpenCV only gained CUDA 13 support in opencv#27636, merged after the 4.12.0 release, and JetPack 7 is CUDA 13.2.
- `opencv_contrib` is fetched only for CUDA builds. Nothing here compiles against a contrib module, but `cudev` and the `cuda*` modules live there and `WITH_CUDA=ON` fails configure without them.
- The apt runtime libraries stay installed, because other packages link them and removing `libopencv-dev` cascades. CMake searches `/usr/local` first, and `CMakeLists.txt` asserts the version floor (`find_package(OpenCV 4.10 ...)`), so the source build wins without uninstalling anything.
- If configure fails after an install, `build/CMakeCache.txt` still pins the old `OpenCV_DIR`. Run `./scripts/clean_build.sh`.
- Script basis: https://github.com/AastaNV/JEP/blob/master/script/install_opencv4.10.0_Jetpack6.1.sh
- `opencv_has_cuda` uses the imported target `opencv_cudaarithm` as the CUDA marker. A CUDA build exports it. A CPU-only build of the same version does not.
- The installer skips only when the installed version is new enough. Testing `pkg-config --exists opencv4` alone is true on any machine that ever had apt OpenCV, which is every desktop, so the upgrade silently did nothing. It also let JetPack's 4.8.0 pre-empt the Jetson build.
- Version alone is not enough on JetPack 7. It ships an OpenCV newer than the pinned version that passes this check and the `find_package(OpenCV)` floor but has no CUDA modules, so the source build would be skipped and the CUDA loss would surface only at runtime. When CUDA is requested, the installer requires the CUDA modules too.

## Foxglove SDK

The upstream zip contains a top-level `foxglove/` directory, so `install/install_foxglove_sdk.sh` extracts it directly into `third_party/`.

## MCAP CLI

Ubuntu's `~/.profile` only adds `~/.local/bin` to `PATH` when the directory exists at login. On a fresh flash the directory `install/install_mcap_cli.sh` just created is not on `PATH` until the next login, so the script prints a note.

## Node

`install/install_node.sh` installs Node.js 22 from NodeSource for building the web dashboard (`scripts/build_web.sh`). Ubuntu 24.04's apt ships Node 18, and current Vite needs 20.19 or newer. NodeSource publishes amd64 and arm64, so the dev boxes and the Jetson build the page the same way. A node already on `PATH` at 20.19 or newer (nvm, say) is left alone.

## LLVM toolchain

`install/install_llvm_toolchain.sh` installs the pinned clang-format, clang-tidy, and clangd. The unversioned distro packages track whichever LLVM the release ships (14 on Ubuntu 22.04 and JetPack 6, 18 on Ubuntu 24.04), and those versions disagree about formatting: clang-format 14 writes `override{}`, 18 writes `override {}`. An unpinned toolchain rewrites the tree differently depending on which machine ran `scripts/lint`, so every machine pins the version in `.llvm-version`.

Ubuntu 24.04 carries `clang-*-18` in its own repos. Older releases do not, so apt.llvm.org is added for them. It publishes amd64 and arm64, which covers both the dev boxes and the Jetson.

## clang-tidy cache

`install/install_clang_tidy_cache.sh` installs ctcache, pinned to commit `0c7fee26e09ae8a393ed7d56164102da118ab23a`. ctcache does not cut releases, and the hash it computes is part of our cache key, so an unpinned upgrade would quietly invalidate every entry.

Upstream ships the wrapper that `run-clang-tidy` points at: a `clang-tidy` script at the repo root that reads the real binary from `CTCACHE_CLANG_TIDY` and execs the Python client beside it. The client is stdlib-only, so nothing is installed into the venv. Its `requirements.txt` covers the optional server mode, which we do not use.

## Legacy service rename

`install/rename_legacy_services.sh` renames the systemd units that earlier installs created under old names. Each unit keeps its enabled and running state, and the old and new app units never run side by side.

- It is idempotent: on a box with no old unit it does nothing. `service/build_and_install_service.sh` runs it before installing anything, so a deploy migrates a box on its own.
- Delete it once no box has an old name left.
- A unit file is copied with every old unit name in it rewritten (`dashboard-wifi` has `Requires=` on the firewall unit, the kiosk orders `After=` the relay). The polkit rule that lets the app start the Wi-Fi unit is rewritten the same way.
- The `viz_relay` port 80 drop-in is deleted, not moved, because `service/viz-relay.service` carries `--http-port 80` itself now.
- Helper scripts under `/usr/local/sbin` keep their names. Only units were renamed.
- It records unit states before stopping anything, because stopping the firewall unit also stops the Wi-Fi unit that `Requires=` it. `RENAMES` lists the firewall before the Wi-Fi unit and the relay before the app, so restoring in that order satisfies `Requires=` and `After=`.
