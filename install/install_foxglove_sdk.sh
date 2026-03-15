#!/bin/bash

# Install Foxglove C++ SDK (pre-built binary) into third_party/foxglove/
# Supports x86_64 and aarch64 Linux targets.

install_foxglove_sdk() {
    local SCRIPT_DIR
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local PROJECT_ROOT
    PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
    local FOXGLOVE_VERSION="0.20.0"
    local FOXGLOVE_DIR="$PROJECT_ROOT/third_party/foxglove"

    if [ -f "$FOXGLOVE_DIR/include/foxglove/schemas.hpp" ]; then
        echo "Foxglove SDK v${FOXGLOVE_VERSION} already installed at $FOXGLOVE_DIR"
        return 0
    fi

    local ARCH
    ARCH="$(uname -m)"
    local PLATFORM
    if [ "$ARCH" = "x86_64" ]; then
        PLATFORM="x86_64-unknown-linux-gnu"
    elif [ "$ARCH" = "aarch64" ]; then
        PLATFORM="aarch64-unknown-linux-gnu"
    else
        echo "Error: Unsupported architecture: $ARCH (expected x86_64 or aarch64)"
        exit 1
    fi

    local ZIP_NAME="foxglove-v${FOXGLOVE_VERSION}-cpp-${PLATFORM}.zip"
    local URL="https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv${FOXGLOVE_VERSION}/${ZIP_NAME}"
    local TMP_ZIP="/tmp/${ZIP_NAME}"

    echo "Downloading Foxglove SDK v${FOXGLOVE_VERSION} for ${PLATFORM}..."
    curl -fsSL "$URL" -o "$TMP_ZIP"

    echo "Extracting to $FOXGLOVE_DIR..."
    mkdir -p "$PROJECT_ROOT/third_party"
    # The zip contains a top-level "foxglove/" directory; extract directly into third_party/
    unzip -q "$TMP_ZIP" -d "$PROJECT_ROOT/third_party"
    rm -f "$TMP_ZIP"

    echo "Foxglove SDK v${FOXGLOVE_VERSION} installed successfully at $FOXGLOVE_DIR"
}
