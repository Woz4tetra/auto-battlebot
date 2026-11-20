#!/bin/bash

read_device_tree_model() {
    tr -d '\0' <"${1:-/proc/device-tree/model}"
}

check_jetson_orin() {
    if [ ! -f /etc/nv_tegra_release ]; then
        echo "Error: This is not a NVIDIA Jetson device"
        exit 1
    fi

    if [ ! -f /proc/device-tree/model ]; then
        echo "Error: Cannot determine device model"
        exit 1
    fi

    local model
    model=$(read_device_tree_model)
    if [[ ! "$model" =~ "Orin" ]]; then
        echo "Error: This script requires a Jetson Orin (Nano or NX)"
        echo "Detected device: $model"
        exit 1
    fi

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
