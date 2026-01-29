#!/bin/bash
# Step 0: Install TensorRT runtime (and dev headers) on Ubuntu 22.04 / 24.04.
# Tries apt install first; if packages are missing, adds NVIDIA CUDA keyring and retries.
# TensorRT is required for DeepLab and YOLO inference.

set -e

install_tensorrt_runtime_ubuntu() {
    if dpkg -s libnvinfer-dev >/dev/null 2>&1; then
        echo "TensorRT runtime (libnvinfer-dev) is already installed."
        return 0
    fi

    echo "Installing TensorRT runtime (libnvinfer-dev)..."

    if sudo apt-get update -qq && sudo apt-get install -y libnvinfer-dev; then
        echo "TensorRT runtime installed successfully."
        return 0
    fi

    # Packages not in default repos; try adding NVIDIA CUDA keyring (may include TensorRT on some setups)
    local KEYRING_URL=""
    local KEYRING_DEB="/tmp/cuda-keyring.deb"
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        case "${VERSION_ID}" in
            22.04)
                KEYRING_URL="https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb"
                ;;
            24.04)
                KEYRING_URL="https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb"
                ;;
            *)
                echo "Unsupported Ubuntu version: ${VERSION_ID}. Cannot add NVIDIA repo automatically."
                print_tensorrt_instructions
                exit 1
                ;;
        esac
        if wget -q -O "${KEYRING_DEB}" "${KEYRING_URL}" && sudo dpkg -i "${KEYRING_DEB}"; then
            rm -f "${KEYRING_DEB}"
            sudo apt-get update -qq
            if sudo apt-get install -y libnvinfer-dev; then
                echo "TensorRT runtime installed successfully (via NVIDIA repo)."
                return 0
            fi
        else
            rm -f "${KEYRING_DEB}"
        fi
    fi

    echo "Could not install TensorRT runtime automatically."
    print_tensorrt_instructions
    exit 1
}

print_tensorrt_instructions() {
    echo ""
    echo "Please add the NVIDIA TensorRT repository and install manually:"
    echo "  https://docs.nvidia.com/deeplearning/tensorrt/install-guide/"
    echo ""
    echo "Then run: sudo apt-get install -y libnvinfer-dev"
    echo ""
}
