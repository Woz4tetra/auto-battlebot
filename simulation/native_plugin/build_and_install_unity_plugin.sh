#!/usr/bin/env bash
set -euo pipefail

# Builds the GpuMemoryShare native plugin and installs it into Unity:
#   simulation/auto-battlebot-sim/Assets/Plugins/x86_64/
#
# Unity C# uses: [DllImport("GpuMemoryShare")]
# On Linux, that resolves to libGpuMemoryShare.so.
# IMPORTANT: Do NOT also install/copy a second file (e.g. GpuMemoryShare.so),
# otherwise Unity will report "Multiple plugins with the same name".

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
UNITY_PLUGINS_DIR="${ROOT_DIR}/../auto-battlebot-sim/Assets/Plugins"
UNITY_LINUX_DIR="${UNITY_PLUGINS_DIR}/x86_64"

BUILD_TYPE="${BUILD_TYPE:-Release}"
ENABLE_CUDA="${ENABLE_CUDA:-ON}"
ENABLE_OPENGL="${ENABLE_OPENGL:-ON}"

echo "[GpuMemoryShare] Build type: ${BUILD_TYPE}"
echo "[GpuMemoryShare] ENABLE_CUDA=${ENABLE_CUDA} ENABLE_OPENGL=${ENABLE_OPENGL}"
echo "[GpuMemoryShare] Unity plugins dir: ${UNITY_LINUX_DIR}"

mkdir -p "${BUILD_DIR}"
mkdir -p "${UNITY_LINUX_DIR}"

cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DENABLE_CUDA="${ENABLE_CUDA}" \
  -DENABLE_OPENGL="${ENABLE_OPENGL}" \
  -DUNITY_PLUGINS_DIR="${UNITY_PLUGINS_DIR}"

cmake --build "${BUILD_DIR}" --config "${BUILD_TYPE}" -j

cmake --install "${BUILD_DIR}" --config "${BUILD_TYPE}"

if [[ ! -f "${UNITY_LINUX_DIR}/libGpuMemoryShare.so" ]]; then
  echo "[GpuMemoryShare] ERROR: expected ${UNITY_LINUX_DIR}/libGpuMemoryShare.so to exist after install" >&2
  exit 1
fi

# Remove legacy symlink if present (prevents Unity duplicate-plugin error)
rm -f "${UNITY_LINUX_DIR}/GpuMemoryShare.so" "${UNITY_LINUX_DIR}/GpuMemoryShare.so.meta" 2>/dev/null || true

echo "[GpuMemoryShare] Installed:"
echo "  - ${UNITY_LINUX_DIR}/libGpuMemoryShare.so"
echo "  - ${UNITY_PLUGINS_DIR}/gpu_memory_share.h"
