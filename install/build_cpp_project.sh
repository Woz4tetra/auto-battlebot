#!/bin/bash

build_cpp_project() {
    set -e

    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

    echo "Creating build directory..."
    cd "${SCRIPT_DIR}/../"
    mkdir -p build
    cd build/


    echo "Running cmake..."
    cmake .. -Wno-dev

    echo "Building project..."
    make -j"$(nproc)"

    echo "Build complete!"
}
