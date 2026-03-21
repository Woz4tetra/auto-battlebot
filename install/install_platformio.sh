#!/bin/bash

install_platformio() {
    if command -v pio >/dev/null 2>&1; then
        echo "PlatformIO is installed. Skipping PlatformIO installation."
        return 0
    fi

    echo "Installing PlatformIO CLI..."
    pip install platformio

    if command -v pio >/dev/null 2>&1; then
        echo "PlatformIO installed successfully: $(pio --version)"
    else
        echo "Error: PlatformIO installed but 'pio' not found on PATH."
        return 1
    fi
}
