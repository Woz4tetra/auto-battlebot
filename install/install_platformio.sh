install_platformio() {
    # esptool 4.11 (bundled by the espressif32 platform) imports intelhex at
    # image-build time, but does not pull it in. Without it every ESP32 build
    # fails at "Building firmware.bin".
    pip install intelhex

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
