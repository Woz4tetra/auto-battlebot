#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# Build and Install CUDA Interop Unity Plugin
# =============================================================================
#
# This script builds the CudaInteropPlugin native library and installs it
# to the Unity project's Plugins folder.
#
# Requirements:
#   - CUDA Toolkit 12.x installed
#   - CMake 3.18+
#   - GCC/G++ with C++17 support
#   - OpenGL development libraries (libgl1-mesa-dev)
#
# Usage:
#   ./build_and_install_unity_plugin.sh
#
# Environment variables:
#   BUILD_TYPE      - Release (default) or Debug
#   ENABLE_CUDA     - ON (default) or OFF
#   ENABLE_OPENGL   - ON (default) or OFF
#   CUDA_HOME       - Path to CUDA toolkit (auto-detected if not set)
#
# =============================================================================

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
UNITY_PLUGINS_DIR="${ROOT_DIR}/../auto-battlebot-sim/Assets/Plugins"
UNITY_LINUX_DIR="${UNITY_PLUGINS_DIR}/x86_64"

BUILD_TYPE="${BUILD_TYPE:-Release}"
ENABLE_CUDA="${ENABLE_CUDA:-ON}"
ENABLE_OPENGL="${ENABLE_OPENGL:-ON}"

echo "=============================================="
echo "  CUDA Interop Unity Plugin Builder"
echo "=============================================="
echo ""
echo "Build configuration:"
echo "  BUILD_TYPE:    ${BUILD_TYPE}"
echo "  ENABLE_CUDA:   ${ENABLE_CUDA}"
echo "  ENABLE_OPENGL: ${ENABLE_OPENGL}"
echo ""
echo "Directories:"
echo "  Source:        ${ROOT_DIR}"
echo "  Build:         ${BUILD_DIR}"
echo "  Unity Plugins: ${UNITY_LINUX_DIR}"
echo ""

# Check for CUDA and detect CUDA_HOME
CUDA_CMAKE_ARGS=""
if [[ "${ENABLE_CUDA}" == "ON" ]]; then
    # Find nvcc
    if command -v nvcc &> /dev/null; then
        NVCC_PATH=$(command -v nvcc)
        NVCC_VERSION=$(nvcc --version | grep "release" | awk '{print $5}' | tr -d ',')
        echo "Found CUDA: nvcc ${NVCC_VERSION} at ${NVCC_PATH}"
        
        # Auto-detect CUDA_HOME from nvcc path if not set
        if [[ -z "${CUDA_HOME:-}" ]]; then
            # nvcc is typically at /usr/local/cuda/bin/nvcc or similar
            CUDA_HOME="$(dirname "$(dirname "${NVCC_PATH}")")"
        fi
    else
        echo "ERROR: CUDA enabled but nvcc not found in PATH"
        echo "Please install CUDA Toolkit and ensure nvcc is in your PATH"
        exit 1
    fi
    
    # Verify CUDA_HOME
    if [[ -n "${CUDA_HOME:-}" ]] && [[ -d "${CUDA_HOME}" ]]; then
        echo "Using CUDA_HOME: ${CUDA_HOME}"
        CUDA_CMAKE_ARGS="-DCMAKE_CUDA_COMPILER=${CUDA_HOME}/bin/nvcc"
    fi
fi

# Check for CMake
if ! command -v cmake &> /dev/null; then
    echo "ERROR: cmake not found"
    exit 1
fi
CMAKE_VERSION=$(cmake --version | head -n1)
echo "Found CMake: ${CMAKE_VERSION}"

# Create directories
mkdir -p "${BUILD_DIR}"
mkdir -p "${UNITY_LINUX_DIR}"

echo ""
echo "Configuring with CMake..."
echo ""

# shellcheck disable=SC2086
cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DENABLE_CUDA="${ENABLE_CUDA}" \
    -DENABLE_OPENGL="${ENABLE_OPENGL}" \
    -DUNITY_PLUGINS_DIR="${UNITY_PLUGINS_DIR}" \
    ${CUDA_CMAKE_ARGS}

echo ""
echo "Building..."
echo ""

cmake --build "${BUILD_DIR}" --config "${BUILD_TYPE}" -j "$(nproc)"

echo ""
echo "Installing..."
echo ""

cmake --install "${BUILD_DIR}" --config "${BUILD_TYPE}"

# Verify installation
echo ""
echo "Verifying installation..."

PLUGIN_PATH="${UNITY_LINUX_DIR}/libCudaInteropPlugin.so"

if [[ -f "${PLUGIN_PATH}" ]]; then
    PLUGIN_SIZE=$(du -h "${PLUGIN_PATH}" | cut -f1)
    echo "✓ Installed: ${PLUGIN_PATH} (${PLUGIN_SIZE})"
    
    # Check library dependencies
    echo ""
    echo "Library dependencies:"
    ldd "${PLUGIN_PATH}" | grep -E "(cuda|libGL)" || echo "  (none found - may be statically linked)"
else
    echo "ERROR: Expected ${PLUGIN_PATH} to exist after install"
    exit 1
fi

if [[ -f "${UNITY_PLUGINS_DIR}/cuda_interop.h" ]]; then
    echo "✓ Installed: ${UNITY_PLUGINS_DIR}/cuda_interop.h"
fi

# Clean up any old plugin files that might cause conflicts
OLD_PLUGINS=(
    "${UNITY_LINUX_DIR}/CudaInteropPlugin.so"  # Without lib prefix
)

for old_plugin in "${OLD_PLUGINS[@]}"; do
    if [[ -f "${old_plugin}" ]]; then
        echo "Removing old plugin: ${old_plugin}"
        rm -f "${old_plugin}" "${old_plugin}.meta" 2>/dev/null || true
    fi
done

echo ""
echo "=============================================="
echo "  Build complete!"
echo "=============================================="
echo ""
echo "Unity C# usage:"
echo '  [DllImport("CudaInteropPlugin")]'
echo '  private static extern CudaInteropError CudaInterop_Initialize(int deviceId);'
echo ""
echo "Note: Unity must use OpenGL graphics API for CUDA Interop to work."
echo "Set in Project Settings > Player > Other Settings > Graphics API."
echo ""
