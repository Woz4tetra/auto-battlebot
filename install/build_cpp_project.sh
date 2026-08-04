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

    # Only run cmake configure on the first build. After that, the generated
    # Makefiles contain a cmake_check_build_system hook that automatically
    # re-runs cmake if CMakeLists.txt or any cmake file actually changes.
    # Running cmake unconditionally regenerates internal files (compiler_depend.make,
    # Makefile2, etc.) on every invocation, causing spurious "Built target" output
    # and slow cmake overhead even when nothing has changed.
    #
    # Guard on the generated Makefile, not CMakeCache.txt: cmake writes the cache
    # early in the configure step but only emits the Makefile once generation
    # completes. If a prior configure was interrupted, the cache exists without a
    # Makefile; guarding on the cache would skip configure and fail at `make` with
    # "No targets specified and no makefile found". Guarding on the Makefile
    # correctly re-runs cmake to finish an interrupted configure.
    if [ ! -f "Makefile" ]; then
        echo "Running cmake with BUILD_TESTING=${BUILD_TESTING_FLAG} and BUILD_TYPE=${BUILD_TYPE}..."
        cmake .. -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" -DBUILD_TESTING="${BUILD_TESTING_FLAG}" -DBUILD_DEBUG="${BUILD_TESTING_FLAG}"
    fi

    echo "Building project..."
    make -j"$(nproc)"

    echo "Build complete!"
}
