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
    mkdir -p build
    cd build/

    # Add LibTorch to CMAKE_PREFIX_PATH if it exists
    local LIBTORCH_DIR="/usr/local/libtorch"
    if [ -d "$LIBTORCH_DIR" ]; then
        echo "Found LibTorch at $LIBTORCH_DIR"
        export CMAKE_PREFIX_PATH="$LIBTORCH_DIR:${CMAKE_PREFIX_PATH:-}"
    fi

    echo "Running cmake with BUILD_TESTING=${BUILD_TESTING_FLAG} and BUILD_TYPE=${BUILD_TYPE}..."
    cmake .. -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" -DBUILD_TESTING="${BUILD_TESTING_FLAG}" -Wno-dev

    echo "Building project..."
    make -j"$(nproc)"

    echo "Build complete!"
}
