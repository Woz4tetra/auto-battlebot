#!/bin/bash

set -e

get_cuda_version() {
    local ver
    ver=$(nvcc --version 2>/dev/null | grep -oP 'release \K[0-9]+\.[0-9]+' | head -1)
    if [ -z "$ver" ] && [ -f /usr/local/cuda/version.txt ]; then
        ver=$(grep -oP 'CUDA Version \K[0-9]+\.[0-9]+' /usr/local/cuda/version.txt 2>/dev/null | head -1)
    fi
    if [ -z "$ver" ] && [ -d /usr/local/cuda ]; then
        ver=$(ls -d /usr/local/cuda-[0-9]*.[0-9]* 2>/dev/null | sort -V | tail -1 | grep -oP 'cuda-\K[0-9]+\.[0-9]+')
    fi
    if [ -z "$ver" ]; then
        echo "Error: could not detect the CUDA version (no nvcc, no /usr/local/cuda-*)." >&2
        echo "The CUDA version picks the wheel index, and a wrong guess installs a torch that fails at the first kernel launch." >&2
        return 1
    fi
    echo "$ver"
}

get_l4t_major() {
    local first_line
    first_line=$(head -n 1 "${NV_TEGRA_RELEASE_FILE:-/etc/nv_tegra_release}" 2>/dev/null || true)
    if [[ "$first_line" =~ R([0-9]+) ]]; then
        echo "${BASH_REMATCH[1]}"
    else
        echo ""
    fi
}

get_jetson_python_version() {
    /usr/bin/python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")'
}

get_jetson_torch_install_url() {
    if [ -n "${TORCH_INSTALL:-}" ]; then
        return 0
    fi
    local first_line
    first_line=$(head -n 1 "${NV_TEGRA_RELEASE_FILE:-/etc/nv_tegra_release}" 2>/dev/null || true)
    local JP_VERSION
    if [[ "$first_line" =~ R([0-9]+)\.([0-9]+) ]]; then
        local l4t_major="${BASH_REMATCH[1]}"
        local l4t_minor="${BASH_REMATCH[2]}"
        JP_VERSION=$(( (l4t_major % 30) * 10 + l4t_minor ))
    elif [[ "$first_line" =~ R([0-9]+).*REVISION:\s*([0-9.]+) ]]; then
        local r="${BASH_REMATCH[1]}"
        local rev="${BASH_REMATCH[2]}"
        local rev_major="${rev%%.*}"
        JP_VERSION=$(( (r % 30) * 10 + rev_major ))
    else
        JP_VERSION="61"
    fi
    local WHEEL_NAME JP_PREFIX
    case "$JP_VERSION" in
        6[1-9]) JP_PREFIX="v61"; WHEEL_NAME="torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl" ;;
        *) echo "Error: no PyTorch wheel for JP_VERSION ${JP_VERSION}; JetPack 6.2.3 (R36.5) is the oldest supported" >&2; return 1 ;;
    esac
    TORCH_INSTALL="https://developer.download.nvidia.com/compute/redist/jp/${JP_PREFIX}/pytorch/${WHEEL_NAME}"
}

ensure_jetson_torch_in_venv() {
    if [ ! -f "${NV_TEGRA_RELEASE_FILE:-/etc/nv_tegra_release}" ]; then
        return 0
    fi
    if python -c "import torch" 2>/dev/null; then
        echo "Jetson: PyTorch already in venv; using it (will not install from PyPI)."
        return 0
    fi
    local l4t_major
    l4t_major=$(get_l4t_major)

    if [ -n "$l4t_major" ] && [ "$l4t_major" -ge 39 ]; then
        local cuda_ver index_url
        cuda_ver=$(get_cuda_version) || return 1
        index_url="https://download.pytorch.org/whl/cu${cuda_ver//./}"
        echo "Jetson: installing torch and torchvision from ${index_url} (CUDA ${cuda_ver})..."
        pip install --no-cache-dir --index-url "$index_url" torch torchvision
        return 0
    fi

    install_jetson_torch_wheel
}

install_jetson_torch_wheel() {
    get_jetson_torch_install_url
    echo "Jetson: installing PyTorch wheel so dependencies use it instead of PyPI..."
    pip install "numpy==1.26.1"
    pip install --no-cache-dir "$TORCH_INSTALL"
}

install_cusparselt_linux_aarch64() {
    local CUSPARSELT_NAME="libcusparse_lt-linux-aarch64-0.6.3.2-archive"
    local CUSPARSELT_TMP
    CUSPARSELT_TMP=$(mktemp -d)
    echo "Installing cuSPARSELt ($CUSPARSELT_NAME)..."
    curl --retry 3 -fLs -o "$CUSPARSELT_TMP/$CUSPARSELT_NAME.tar.xz" \
        "https://developer.download.nvidia.com/compute/cusparselt/redist/libcusparse_lt/linux-aarch64/$CUSPARSELT_NAME.tar.xz"
    tar -xf "$CUSPARSELT_TMP/$CUSPARSELT_NAME.tar.xz" -C "$CUSPARSELT_TMP"
    sudo cp -a "$CUSPARSELT_TMP/$CUSPARSELT_NAME/include/"* /usr/local/cuda/include/
    sudo cp -a "$CUSPARSELT_TMP/$CUSPARSELT_NAME/lib/"* /usr/local/cuda/lib64/
    sudo ldconfig
    rm -rf "$CUSPARSELT_TMP"
}

install_pytorch_jetson() {
    if [ ! -f /etc/nv_tegra_release ]; then
        echo "Error: This is not a Jetson device. PyTorch Jetson install is for Tegra only."
        exit 1
    fi

    local SCRIPT_DIR
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local PROJECT_ROOT
    PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
    local VENV_DIR="$PROJECT_ROOT/venv"

    local _python="${VENV_DIR}/bin/python"
    if [ ! -x "$_python" ]; then
        _python="python3"
    fi
    if "$_python" -c "import torch" 2>/dev/null; then
        local _torch_ver
        _torch_ver=$("$_python" -c "import torch; print(torch.__version__)")
        echo "PyTorch already installed ($_torch_ver). Skipping."
        return 0
    fi

    local PY_VER
    PY_VER=$(get_jetson_python_version) || return 1
    local L4T_MAJOR
    L4T_MAJOR=$(get_l4t_major)

    echo "Installing PyTorch prerequisites (python3-pip, libopenblas-dev)..."
    sudo apt-get -y update
    sudo apt-get install -y python3-pip libopenblas-dev

    local CUDA_VER
    CUDA_VER=$(get_cuda_version) || return 1
    echo "Detected CUDA version: $CUDA_VER"
    if { [ -z "$L4T_MAJOR" ] || [ "$L4T_MAJOR" -lt 39 ]; } \
        && ! ldconfig -p | grep -q 'libcusparseLt\.so\.0'; then
        install_cusparselt_linux_aarch64
    fi

    if [ ! -d "$VENV_DIR" ]; then
        echo "Creating project venv at $VENV_DIR (python${PY_VER})..."
        if ! command -v "python${PY_VER}" &>/dev/null; then
            echo "Error: python${PY_VER} not found. Install it (e.g. python${PY_VER}-venv) and re-run."
            exit 1
        fi
        "python${PY_VER}" -m venv "$VENV_DIR"
        local SITE_PACKAGES="$VENV_DIR/lib/python${PY_VER}/site-packages"
        local SYS_PYTHON_PATH="/usr/lib/python${PY_VER}/dist-packages"
        if [ -d "$SYS_PYTHON_PATH" ]; then
            echo "$SYS_PYTHON_PATH" > "$SITE_PACKAGES/jetson_system_packages.pth"
        fi
    fi

    local ACTIVATE_SH="$VENV_DIR/bin/activate"
    if ! grep -q 'jetson.*LD_LIBRARY_PATH' "$ACTIVATE_SH" 2>/dev/null; then
        echo "Jetson: adding CUDA library path to venv activate script..."
        cat >> "$ACTIVATE_SH" << 'JETSON_ACTIVATE_EOF'

# Jetson: prepend CUDA/cuDNN paths so PyTorch in venv sees CUDA (detect installed version)
if [ -f /etc/nv_tegra_release ]; then
    _jetson_ld_path=""
    _cuda_ver=$(nvcc --version 2>/dev/null | grep -oP "release \\K[0-9]+\\.[0-9]+" | head -1)
    for _p in $( [ -n "$_cuda_ver" ] && echo "/usr/local/cuda-$_cuda_ver/lib64" ) /usr/local/cuda/lib64 /usr/lib/aarch64-linux-gnu; do
        [ -d "$_p" ] && _jetson_ld_path="${_jetson_ld_path:+$_jetson_ld_path:}$_p"
    done
    [ -n "$_jetson_ld_path" ] && export LD_LIBRARY_PATH="${_jetson_ld_path}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    unset _p _jetson_ld_path _cuda_ver
fi
JETSON_ACTIVATE_EOF
    fi

    source "$VENV_DIR/bin/activate"
    pip install --upgrade pip
    ensure_jetson_torch_in_venv
    echo "PyTorch for Jetson installed in $VENV_DIR"
    python -c "import torch; print('PyTorch', torch.__version__, 'CUDA', torch.cuda.is_available())"
}
