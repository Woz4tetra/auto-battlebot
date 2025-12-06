#!/bin/bash

# Check if device is a Jetson Orin Nano
check_jetson_orin_nano() {
    # Check if this is a Jetson device
    if [ ! -f /etc/nv_tegra_release ]; then
        echo "Error: This is not a NVIDIA Jetson device"
        exit 1
    fi

    # Check device model
    if [ -f /proc/device-tree/model ]; then
        local model
        model=$(cat /proc/device-tree/model)
        if [[ ! "$model" =~ "Orin Nano" ]]; then
            echo "Error: This script requires a Jetson Orin Nano"
            echo "Detected device: $model"
            exit 1
        fi
    else
        echo "Error: Cannot determine device model"
        exit 1
    fi

    echo "Jetson Orin Nano detected"
}
