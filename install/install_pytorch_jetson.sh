#!/bin/bash
# Install PyTorch on Jetson per NVIDIA docs:
# https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html
#
# Prerequisites: JetPack installed. Optional: run scripts/setup_python.sh first to
# create the project venv; if the venv does not exist, this script creates it and
# installs only PyTorch (then run setup_python.sh to install the rest of the project).

set -e

# Detect installed CUDA version (e.g. 12.6). Uses nvcc --version, else version.txt, else default.
get_cuda_version() {
    local ver
    ver=$(nvcc --version 2>/dev/null | grep -oP 'release \K[0-9]+\.[0-9]+' | head -1)
    if [ -z "$ver" ] && [ -f /usr/local/cuda/version.txt ]; then
        ver=$(grep -oP 'CUDA Version \K[0-9]+\.[0-9]+' /usr/local/cuda/version.txt 2>/dev/null | head -1)
    fi
    if [ -z "$ver" ] && [ -d /usr/local/cuda ]; then
        ver=$(ls -d /usr/local/cuda-[0-9]*.[0-9]* 2>/dev/null | sort -V | tail -1 | grep -oP 'cuda-\K[0-9]+\.[0-9]+')
    fi
    echo "${ver:-12.1}"
}

# Compute TORCH_INSTALL URL for current JetPack (no side effects). Sets TORCH_INSTALL if empty.
get_jetson_torch_install_url() {
    if [ -n "${TORCH_INSTALL:-}" ]; then
        return 0
    fi
    local first_line
    first_line=$(head -n 1 /etc/nv_tegra_release 2>/dev/null || true)
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
        60)  JP_PREFIX="v60dp"; WHEEL_NAME="torch-2.3.0a0+40ec155e58.nv24.03.13384722-cp310-cp310-linux_aarch64.whl" ;;
        61)  JP_PREFIX="v61";  WHEEL_NAME="torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl" ;;
        62)  JP_PREFIX="v62";  WHEEL_NAME="torch-2.6.0a0+ecf3bae40a.nv25.01-cp310-cp310-linux_aarch64.whl" ;;
        *)   JP_PREFIX="v61";  WHEEL_NAME="torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl" ;;
    esac
    TORCH_INSTALL="https://developer.download.nvidia.com/compute/redist/jp/${JP_PREFIX}/pytorch/${WHEEL_NAME}"
}

# Install Jetson PyTorch wheel into the currently activated venv (no apt/cusparselt).
# Call this from setup_python when on Jetson so torch is present before pip install -e .
ensure_jetson_torch_in_venv() {
    if [ ! -f /etc/nv_tegra_release ]; then
        return 0
    fi
    if python -c "import torch" 2>/dev/null; then
        echo "Jetson: PyTorch already in venv; using it (will not install from PyPI)."
        return 0
    fi
    get_jetson_torch_install_url
    echo "Jetson: installing PyTorch wheel so dependencies use it instead of PyPI..."
    pip install "numpy==1.26.1"
    pip install --no-cache-dir "$TORCH_INSTALL"
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

    # Check whether PyTorch is already importable in the project venv (or system python).
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

    local PY_VER="3.10"

    # 1. System packages required by PyTorch (per NVIDIA doc)
    echo "Installing PyTorch prerequisites (python3-pip, libopenblas-dev)..."
    sudo apt-get -y update
    sudo apt-get install -y python3-pip libopenblas-dev

    # 2. cusparselt for PyTorch 24.06+ (per NVIDIA doc; run in /tmp so it creates dirs there)
    local CUSPARSELT_SH="/tmp/install_cusparselt.sh"
    local CUDA_VER
    CUDA_VER=$(get_cuda_version)
    echo "Detected CUDA version: $CUDA_VER"
    wget -q -O "$CUSPARSELT_SH" "https://raw.githubusercontent.com/pytorch/pytorch/5c6af2b583709f6176898c017424dc9981023c28/.ci/docker/common/install_cusparselt.sh"
    (cd /tmp && sudo CUDA_VERSION=12.1 bash install_cusparselt.sh)

    # 3. Use project venv (create if missing)
    if [ ! -d "$VENV_DIR" ]; then
        echo "Creating project venv at $VENV_DIR (python${PY_VER})..."
        if ! command -v "python${PY_VER}" &>/dev/null; then
            echo "Error: python${PY_VER} not found. Install it (e.g. python3.10-venv) and re-run."
            exit 1
        fi
        "python${PY_VER}" -m venv "$VENV_DIR"
        # Make system TensorRT visible in venv (Jetson)
        local SITE_PACKAGES="$VENV_DIR/lib/python${PY_VER}/site-packages"
        local SYS_PYTHON_PATH="/usr/lib/python${PY_VER}/dist-packages"
        if [ -d "$SYS_PYTHON_PATH" ]; then
            echo "$SYS_PYTHON_PATH" > "$SITE_PACKAGES/jetson_system_packages.pth"
        fi
    fi

    # Ensure venv activate sets LD_LIBRARY_PATH for PyTorch CUDA (so torch.cuda.is_available() works)
    local ACTIVATE_SH="$VENV_DIR/bin/activate"
    if ! grep -q 'jetson.*LD_LIBRARY_PATH' "$ACTIVATE_SH" 2>/dev/null; then
        echo "Jetson: adding CUDA library path to venv activate script..."
        cat >> "$ACTIVATE_SH" << 'JETSON_ACTIVATE_EOF'

# Jetson: prepend CUDA/cuDNN paths so PyTorch in venv sees CUDA (detect installed version)
if [ -f /etc/nv_tegra_release ]; then
    _jetson_ld_path=""
    _cuda_ver=$(nvcc --version 2>/dev/null | grep -oP "release \\K[0-9]+\\.[0-9]+" | head -1)
    for _p in $( [ -n "$_cuda_ver" ] && echo "/usr/local/cuda-$_cuda_ver/lib64" ) /usr/local/cuda/lib64 /usr/lib/aarch64-linux-gnu /usr/lib/llvm-8/lib; do
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
