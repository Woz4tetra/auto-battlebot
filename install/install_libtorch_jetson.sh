#!/bin/bash

# Install LibTorch for Jetson (ARM64) by linking to PyTorch installation
install_libtorch_jetson() {
    local install_dir="${1:-/usr/local}"
    local libtorch_dir="$install_dir/libtorch"
    
    # Check if LibTorch is already installed
    if [ -d "$libtorch_dir" ] && [ -f "$libtorch_dir/share/cmake/Torch/TorchConfig.cmake" ]; then
        echo "LibTorch is already installed at $libtorch_dir"
        return 0
    fi
    
    echo "LibTorch not found. Setting up for Jetson..."
    echo ""
    echo "Note: Jetson devices use PyTorch installed via pip/conda."
    echo "LibTorch C++ API uses the same backend as PyTorch Python."
    
    # Try to find torch through Python
    local python_torch
    python_torch=$(python3 -c "import torch; print(torch.__path__[0])" 2>/dev/null || echo "")
    
    if [ -n "$python_torch" ]; then
        echo "Found PyTorch at: $python_torch"
        
        # Check if torch C++ libraries exist
        if [ -f "$python_torch/lib/libtorch.so" ] || [ -f "$python_torch/lib/libtorch_cpu.so" ]; then
            echo "PyTorch C++ libraries found. Creating symlink..."
            
            if [ "$install_dir" = "/usr/local" ] || [[ "$install_dir" == /usr/* ]]; then
                sudo ln -sf "$python_torch" "$libtorch_dir"
            else
                mkdir -p "$(dirname "$libtorch_dir")"
                ln -sf "$python_torch" "$libtorch_dir"
            fi
            
            echo "LibTorch linked successfully at $libtorch_dir"
            echo ""
            echo "To use LibTorch with CMake, add the following to your environment:"
            echo "export CMAKE_PREFIX_PATH=$libtorch_dir:\$CMAKE_PREFIX_PATH"
            return 0
        else
            echo "Warning: PyTorch C++ libraries not found in PyTorch installation."
            echo "You may need to install a version of PyTorch with C++ support."
        fi
    fi
    
    echo ""
    echo "ERROR: Could not find a suitable PyTorch installation."
    echo ""
    echo "For Jetson devices, please install PyTorch first:"
    echo "  1. Follow NVIDIA's PyTorch installation guide for Jetson"
    echo "  2. Typically: sudo apt install python3-torch"
    echo "  3. Or use NVIDIA's provided wheels from:"
    echo "     https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048"
    echo ""
    echo "After installing PyTorch, run this script again."
    
    return 1
}
