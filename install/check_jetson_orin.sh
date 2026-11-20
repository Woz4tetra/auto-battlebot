#!/bin/bash

read_device_tree_model() {
    tr -d '\0' <"${1:-/proc/device-tree/model}"
}

check_jetson_orin() {
    local tegra_release="${NV_TEGRA_RELEASE_FILE:-/etc/nv_tegra_release}"
    if [ ! -f "$tegra_release" ]; then
        echo "Error: This is not a NVIDIA Jetson device"
        exit 1
    fi

    local model_file="${DEVICE_TREE_MODEL_FILE:-/proc/device-tree/model}"
    if [ ! -f "$model_file" ]; then
        echo "Error: Cannot determine device model"
        exit 1
    fi

    local model
    model=$(read_device_tree_model "$model_file")
    if [[ ! "$model" =~ "Orin" ]]; then
        echo "Error: This script requires a Jetson Orin (Nano or NX)"
        echo "Detected device: $model"
        exit 1
    fi

    local l4t_line l4t_major l4t_revision
    l4t_line=$(head -n 1 "$tegra_release" 2>/dev/null || true)
    if [[ "$l4t_line" =~ R([0-9]+) ]]; then
        l4t_major="${BASH_REMATCH[1]}"
        if [ "$l4t_major" -lt 36 ]; then
            echo "Error: L4T R${l4t_major} is older than JetPack 6.2.3 (R36.5) and is not supported"
            exit 1
        fi
        if [ "$l4t_major" -eq 36 ]; then
            l4t_revision=""
            if [[ "$l4t_line" =~ REVISION:\ ([0-9]+) ]]; then
                l4t_revision="${BASH_REMATCH[1]}"
            fi
            if [ -z "$l4t_revision" ] || [ "$l4t_revision" -lt 5 ]; then
                echo "Error: L4T R36.${l4t_revision:-unknown} is not supported. Reflash to JetPack 6.2.3 (R36.5) or JetPack 7.2 (R39.2)."
                exit 1
            fi
        fi
        echo "Jetson Orin detected: $model (L4T R${l4t_major})"
    else
        echo "Error: could not parse the L4T version from $tegra_release"
        exit 1
    fi
}
