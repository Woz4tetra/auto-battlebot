#!/bin/bash

# Check if device is a Jetson Orin (Nano or NX)
check_jetson_orin() {
    if [ ! -f /etc/nv_tegra_release ]; then
        echo "Error: This is not a NVIDIA Jetson device"
        exit 1
    fi

    if [ ! -f /proc/device-tree/model ]; then
        echo "Error: Cannot determine device model"
        exit 1
    fi

    # The device-tree string is NUL-terminated; tr drops it so the shell does
    # not warn about a null byte in the command substitution.
    local model
    model=$(tr -d '\0' < /proc/device-tree/model)
    if [[ ! "$model" =~ "Orin" ]]; then
        echo "Error: This script requires a Jetson Orin (Nano or NX)"
        echo "Detected device: $model"
        exit 1
    fi

    # The install path branches on the L4T major for the PyTorch wheel source and expects
    # the Python version JetPack ships. Without this, a JetPack older than 6 is accepted
    # here and fails much later, at wheel install or engine load.
    local l4t_line l4t_major
    l4t_line=$(head -n 1 /etc/nv_tegra_release 2>/dev/null || true)
    if [[ "$l4t_line" =~ R([0-9]+) ]]; then
        l4t_major="${BASH_REMATCH[1]}"
        if [ "$l4t_major" -lt 36 ]; then
            echo "Error: L4T R${l4t_major} is older than JetPack 6 (R36) and is not supported"
            exit 1
        fi
        echo "Jetson Orin detected: $model (L4T R${l4t_major})"
    else
        echo "Error: could not parse the L4T version from /etc/nv_tegra_release"
        exit 1
    fi
}
