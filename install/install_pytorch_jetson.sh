#!/bin/bash
# Install PyTorch on Jetson per NVIDIA docs:
# https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html
#
# Prerequisites: JetPack installed. Optional: run scripts/setup_python.sh first to
# create the project venv; if the venv does not exist, this script creates it and
# installs only PyTorch (then run setup_python.sh to install the rest of the project).

set -e

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
    local PY_VER="3.10"

    # 1. System packages required by PyTorch (per NVIDIA doc)
    echo "Installing PyTorch prerequisites (python3-pip, libopenblas-dev)..."
    sudo apt-get -y update
    sudo apt-get install -y python3-pip libopenblas-dev

    # 2. cusparselt for PyTorch 24.06+ (per NVIDIA doc)
    local CUSPARSELT_SH="/tmp/install_cusparselt.sh"
    wget -q -O "$CUSPARSELT_SH" "https://raw.githubusercontent.com/pytorch/pytorch/5c6af2b583709f6176898c017424dc9981023c28/.ci/docker/common/install_cusparselt.sh"
    export CUDA_VERSION=12.1
    sudo bash "$CUSPARSELT_SH"

    # 3. PyTorch wheel URL: set TORCH_INSTALL to override. Otherwise we try to derive from JetPack.
    #    Compatibility matrix: https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html
    #    Example for JetPack 6.0 DP, Python 3.10:
    #    export TORCH_INSTALL=https://developer.download.nvidia.com/compute/redist/jp/v60dp/pytorch/torch-2.3.0a0+40ec155e58.nv24.03.13384722-cp310-cp310-linux_aarch64.whl
    if [ -z "${TORCH_INSTALL:-}" ]; then
        local JP_VERSION
        # Parse JetPack / L4T version from /etc/nv_tegra_release (e.g. "R36 (release), REVISION: 2.0" or "R36.2.0")
        local first_line
        first_line=$(head -n 1 /etc/nv_tegra_release 2>/dev/null || true)
        if [[ "$first_line" =~ R([0-9]+)\.([0-9]+) ]]; then
            local l4t_major="${BASH_REMATCH[1]}"
            local l4t_minor="${BASH_REMATCH[2]}"
            # L4T 36.2 -> JetPack 6.2 -> 62; 35.4 -> 5.4 -> 54
            JP_VERSION=$(( (l4t_major % 30) * 10 + l4t_minor ))
        elif [[ "$first_line" =~ R([0-9]+).*REVISION:\s*([0-9.]+) ]]; then
            local r="${BASH_REMATCH[1]}"
            local rev="${BASH_REMATCH[2]}"
            local rev_major="${rev%%.*}"
            JP_VERSION=$(( (r % 30) * 10 + rev_major ))
        else
            JP_VERSION="61"
        fi
        # Default wheel: JetPack 6.x Python 3.10 (cp310). Override TORCH_INSTALL for other JP/Python.
        # Compatibility matrix: https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html
        local WHEEL_NAME
        local JP_PREFIX
        case "$JP_VERSION" in
            60)  JP_PREFIX="v60dp"; WHEEL_NAME="torch-2.3.0a0+40ec155e58.nv24.03.13384722-cp310-cp310-linux_aarch64.whl" ;;
            61)  JP_PREFIX="v61";  WHEEL_NAME="torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl" ;;
            62)  JP_PREFIX="v62";  WHEEL_NAME="torch-2.6.0a0+ecf3bae40a.nv25.01-cp310-cp310-linux_aarch64.whl" ;;
            *)   JP_PREFIX="v61";  WHEEL_NAME="torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl" ;;
        esac
        TORCH_INSTALL="https://developer.download.nvidia.com/compute/redist/jp/${JP_PREFIX}/pytorch/${WHEEL_NAME}"
    fi

    echo "Using PyTorch wheel: $TORCH_INSTALL"

    # 4. Use project venv (create if missing)
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

    source "$VENV_DIR/bin/activate"

    if python -c "import torch" 2>/dev/null; then
        echo "PyTorch is already installed in $VENV_DIR; skipping."
        python -c "import torch; print('PyTorch', torch.__version__, 'CUDA', torch.cuda.is_available())"
        return 0
    fi

    # 5. Install PyTorch per NVIDIA doc (pip upgrade, numpy, wheel)
    echo "Installing PyTorch in venv..."
    pip install --upgrade pip
    pip install "numpy==1.26.1"
    pip install --no-cache-dir "$TORCH_INSTALL"

    echo "PyTorch for Jetson installed in $VENV_DIR"
    python -c "import torch; print('PyTorch', torch.__version__, 'CUDA', torch.cuda.is_available())"
}
