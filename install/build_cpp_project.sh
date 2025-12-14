#!/bin/bash

build_cpp_project() {
    set -e

    local SCRIPT_DIR
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    
    # Parse command line arguments
    local BUILD_TESTING_FLAG
    BUILD_TESTING_FLAG="OFF"
    for arg in "$@"; do
        if [ "$arg" = "--test" ]; then
            BUILD_TESTING_FLAG="ON"
        fi
    done

    echo "Creating build directory..."
    cd "${SCRIPT_DIR}/../"
    mkdir -p build
    cd build/


    echo "Running cmake with BUILD_TESTING=${BUILD_TESTING_FLAG}..."
    cmake .. -DBUILD_TESTING="${BUILD_TESTING_FLAG}" -Wno-dev

    echo "Building project..."
    make -j"$(nproc)"

    echo "Build complete!"
}
