#!/bin/bash

build_cpp_project() {
    set -e

    local SCRIPT_DIR
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    
    # Parse command line arguments
    local BUILD_TESTING_FLAG
    local BUILD_TYPE
    BUILD_TESTING_FLAG="OFF"
    BUILD_TYPE="Release"
    for arg in "$@"; do
        if [ "$arg" = "--test" ]; then
            BUILD_TESTING_FLAG="ON"
            BUILD_TYPE="Debug"
        fi
    done

    echo "Creating build directory..."
    cd "${SCRIPT_DIR}/../"
    local BUILD_DIR
    # AUTO_BATTLEBOT_BUILD_DIR lets a caller pick its own build tree. The docker
    # playback container sets it to build-docker so its CMakeCache.txt, which records
    # container compiler and dependency paths, never collides with the host build/.
    # Use different build directory when testing is enabled
    if [ -n "${AUTO_BATTLEBOT_BUILD_DIR:-}" ]; then
        BUILD_DIR="${AUTO_BATTLEBOT_BUILD_DIR}"
    elif [ "$BUILD_TESTING_FLAG" = "ON" ]; then
        BUILD_DIR="build-test"
    else
        BUILD_DIR="build"
    fi
    mkdir -p "${BUILD_DIR}"
    cd "${BUILD_DIR}/"

    # Prefer Ninja: faster builds, and it parallelizes by default with no -j needed.
    # Falls back to Make on any environment where ninja-build hasn't been installed yet
    # (e.g. install/base_packages.txt was added to but the install script hasn't rerun).
    local GENERATOR_ARGS=()
    local BUILD_SYSTEM_FILE="Makefile"
    local GENERATOR_NAME="Unix Makefiles"
    if command -v ninja >/dev/null 2>&1; then
        GENERATOR_ARGS=(-G Ninja)
        BUILD_SYSTEM_FILE="build.ninja"
        GENERATOR_NAME="Ninja"
    fi

    # A build directory configured by a previous run with the other generator (e.g.
    # before ninja-build was installed here) can't be reconfigured in place -- cmake
    # refuses with "generator does not match the generator used previously". Detect
    # that and wipe the stale cache instead of failing.
    if [ -f "CMakeCache.txt" ] && ! grep -q "^CMAKE_GENERATOR:INTERNAL=${GENERATOR_NAME}$" CMakeCache.txt; then
        echo "Build directory was configured with a different generator; reconfiguring..."
        rm -rf CMakeCache.txt CMakeFiles
    fi

    # Each FetchContent dependency (miniroscpp, tomlplusplus, ...) drives its own git
    # clone through a separate mini CMake project in _deps/<name>-subbuild, with its own
    # CMakeCache.txt recording whatever generator populated it. _deps/ persists across
    # image rebuilds by design (see docs/docker_playback.md), so a subbuild directory
    # can equally predate whichever generator this run picked, and the same "generator
    # does not match" refusal then aborts FetchContent_Populate for that dependency.
    # These directories are disposable CMake-populate drivers, not the fetched sources
    # (_deps/<name>-src) or their compiled output (_deps/<name>-build), so it's safe to
    # remove the whole thing rather than surgically patch it: cmake regenerates it from
    # scratch, and FetchContent's own stamp check re-detects the already-cloned source
    # in *-src and skips re-cloning it.
    for subbuild_cache in _deps/*-subbuild/CMakeCache.txt; do
        [ -f "${subbuild_cache}" ] || continue
        if ! grep -q "^CMAKE_GENERATOR:INTERNAL=${GENERATOR_NAME}$" "${subbuild_cache}"; then
            echo "Reconfiguring $(dirname "${subbuild_cache}") for the ${GENERATOR_NAME} generator..."
            rm -rf "$(dirname "${subbuild_cache}")"
        fi
    done

    # Only run cmake configure on the first build. After that, the generated build
    # files contain a hook that automatically re-runs cmake if CMakeLists.txt or any
    # cmake file actually changes. Running cmake unconditionally regenerates internal
    # files on every invocation, causing spurious "Built target" output and slow cmake
    # overhead even when nothing has changed.
    #
    # Guard on the generated build file, not CMakeCache.txt: cmake writes the cache
    # early in the configure step but only emits the build file once generation
    # completes. If a prior configure was interrupted, the cache exists without a
    # build file; guarding on the cache would skip configure and fail at the build
    # step with "no work to do" / "no makefile found". Guarding on the build file
    # correctly re-runs cmake to finish an interrupted configure.
    if [ ! -f "${BUILD_SYSTEM_FILE}" ]; then
        echo "Running cmake with BUILD_TESTING=${BUILD_TESTING_FLAG} and BUILD_TYPE=${BUILD_TYPE}..."
        cmake .. "${GENERATOR_ARGS[@]}" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" -DBUILD_TESTING="${BUILD_TESTING_FLAG}" -DBUILD_DEBUG="${BUILD_TESTING_FLAG}" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    fi

    echo "Building project..."
    if command -v ninja >/dev/null 2>&1; then
        ninja
    else
        make -j"$(nproc)"
    fi

    # .clangd (shared by every environment) points CompileFlags.CompilationDatabase at
    # the literal "build" directory. When AUTO_BATTLEBOT_BUILD_DIR points elsewhere (the
    # docker dev/playback containers' build-docker), clangd sees no compilation database
    # at all and can't resolve FetchContent headers (miniros, CLI11, ...) or the
    # auto_battlebot namespace -- every include and every project symbol looks
    # undeclared. Symlink only: a plain host build regenerates a real
    # build/compile_commands.json via CMAKE_EXPORT_COMPILE_COMMANDS and overwrites this.
    if [ -n "${AUTO_BATTLEBOT_BUILD_DIR:-}" ] && [ "${AUTO_BATTLEBOT_BUILD_DIR}" != "build" ]; then
        mkdir -p "${SCRIPT_DIR}/../build"
        ln -sf "../${BUILD_DIR}/compile_commands.json" "${SCRIPT_DIR}/../build/compile_commands.json"
    fi

    echo "Build complete!"
}
