#!/bin/bash

# Install LibTorch for Ubuntu
install_libtorch_ubuntu() {
    local download_url="${1}"
    local install_dir="${2:-/usr/local}"
    local libtorch_dir="$install_dir/libtorch"
    
    # Check if LibTorch is already installed
    if [ -d "$libtorch_dir" ] && [ -f "$libtorch_dir/share/cmake/Torch/TorchConfig.cmake" ]; then
        echo "LibTorch is already installed at $libtorch_dir"
        return 0
    fi
    
    echo "LibTorch not found. Installing for Ubuntu..."
    
    
    # Detect CUDA availability
    if command -v nvcc &> /dev/null; then
        echo "CUDA detected. Installing LibTorch with CUDA support..."
    else
        echo "WARNING: CUDA not detected. Installing CPU-only LibTorch..."
        echo "         CPU-only LibTorch will be significantly slower for inference."
        echo "         If you have a GPU, make sure CUDA toolkit is installed."
        echo ""
        read -p "Continue with CPU-only installation? (y/N): " -n 1 -r
        echo ""
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            echo "Installation cancelled."
            return 1
        fi
    fi
    
    # Download LibTorch
    local temp_dir
    temp_dir=$(mktemp -d)
    echo "Downloading LibTorch to $temp_dir..."
    
    if ! wget -q --show-progress "$download_url" -O "$temp_dir/libtorch.zip"; then
        echo "Error: Failed to download LibTorch"
        rm -rf "$temp_dir"
        return 1
    fi
    
    echo "Extracting LibTorch..."
    if ! unzip -q "$temp_dir/libtorch.zip" -d "$temp_dir"; then
        echo "Error: Failed to extract LibTorch"
        rm -rf "$temp_dir"
        return 1
    fi
    
    # Install LibTorch
    echo "Installing LibTorch to $install_dir..."
    if [ "$install_dir" = "/usr/local" ] || [[ "$install_dir" == /usr/* ]]; then
        sudo mv "$temp_dir/libtorch" "$libtorch_dir"
        sudo chmod -R 755 "$libtorch_dir"
    else
        mkdir -p "$(dirname "$libtorch_dir")"
        mv "$temp_dir/libtorch" "$libtorch_dir"
        chmod -R 755 "$libtorch_dir"
    fi
    
    # Clean up
    rm -rf "$temp_dir"
    
    echo "LibTorch installed successfully at $libtorch_dir"
    echo ""
    echo "To use LibTorch with CMake, add the following to your environment:"
    echo "export CMAKE_PREFIX_PATH=$libtorch_dir:\$CMAKE_PREFIX_PATH"
    
    return 0
}
