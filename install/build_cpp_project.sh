#!/bin/bash

build_cpp_project() {
    set -e

    local SCRIPT_DIR
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    
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
    if [ -n "${AUTO_BATTLEBOT_BUILD_DIR:-}" ]; then
        BUILD_DIR="${AUTO_BATTLEBOT_BUILD_DIR}"
    elif [ "$BUILD_TESTING_FLAG" = "ON" ]; then
        BUILD_DIR="build-test"
    else
        BUILD_DIR="build"
    fi
    mkdir -p "${BUILD_DIR}"
    cd "${BUILD_DIR}/"

    if [ ! -f "Makefile" ]; then
        echo "Running cmake with BUILD_TESTING=${BUILD_TESTING_FLAG} and BUILD_TYPE=${BUILD_TYPE}..."
        cmake .. -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" -DBUILD_TESTING="${BUILD_TESTING_FLAG}" -DBUILD_DEBUG="${BUILD_TESTING_FLAG}" \
            -DAUTO_BATTLEBOT_SANITIZER="${AUTO_BATTLEBOT_SANITIZER:-none}"
    fi

    echo "Building project..."
    make -j"$(nproc)" ${AUTO_BATTLEBOT_MAKE_TARGET:-}

    echo "Build complete!"
}
