#!/bin/bash

install_platformio() {
    if command -v pio >/dev/null 2>&1; then
        echo "PlatformIO is installed. Skipping PlatformIO installation."
        return 0
    fi

    echo "Installing PlatformIO CLI..."
    pip3 install --user platformio

    local PIO_PATH="$HOME/.local/bin"
    if ! echo "$PATH" | grep -q "$PIO_PATH"; then
        echo "Adding $PIO_PATH to PATH in ~/.bashrc..."
        echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
        export PATH="$PIO_PATH:$PATH"
    fi

    if command -v pio >/dev/null 2>&1; then
        echo "PlatformIO installed successfully: $(pio --version)"
    else
        echo "Error: PlatformIO installed but 'pio' not found on PATH."
        echo "Try: source ~/.bashrc"
        return 1
    fi
}
