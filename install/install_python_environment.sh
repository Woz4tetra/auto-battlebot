#!/bin/bash

# Script to initialize a Python virtual environment and install project dependencies
# This script creates a venv in the project root and installs packages from pyproject.toml

# Function to check Python version
check_python_version() {
    local python_cmd=$1
    if ! command -v "$python_cmd" &> /dev/null; then
        return 1
    fi
    
    local version_output
    version_output=$($python_cmd --version 2>&1)
    local version
    version=$(echo "$version_output" | grep -oP '\d+\.\d+' | head -1)
    
    local major
    major=$(echo "$version" | cut -d. -f1)
    local minor
    minor=$(echo "$version" | cut -d. -f2)
    
    if [ "$major" -gt "$REQUIRED_MAJOR" ] || 
        ([ "$major" -eq "$REQUIRED_MAJOR" ] && [ "$minor" -ge "$REQUIRED_MINOR" ]); then
        echo "$python_cmd"
        return 0
    fi
    return 1
}

install_python_environment() {
    set -e  # Exit on error

    # Parse flags: --recreate / -y = yes, --no-recreate / -n = no (skip prompt when venv exists)
    local RECREATE_VENV=""
    while [ $# -gt 0 ]; do
        case "$1" in
            --recreate|-y|--recreate=yes) RECREATE_VENV=yes; shift ;;
            --no-recreate|-n|--recreate=no) RECREATE_VENV=no; shift ;;
            *) shift ;;
        esac
    done

    # Get the project root directory (parent of install/ directory)
    local SCRIPT_DIR
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local PROJECT_ROOT
    PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

    echo "Project root: $PROJECT_ROOT"

    # Virtual environment directory
    local VENV_DIR="$PROJECT_ROOT/venv"

    # Python version: 3.10 on Jetson (matches system + TensorRT), 3.11 elsewhere
    local REQUIRED_MAJOR=3
    local REQUIRED_MINOR
    if [ -f /etc/nv_tegra_release ]; then
        REQUIRED_MINOR=10
        echo "Jetson detected: using Python 3.10"
    else
        REQUIRED_MINOR=11
    fi

    # Find appropriate Python executable
    local PYTHON_CMD
    
    if PYTHON_CMD=$(check_python_version "python3"); then
        echo "Found suitable Python: $PYTHON_CMD"
    elif PYTHON_CMD=$(check_python_version "python$REQUIRED_MAJOR.$REQUIRED_MINOR"); then
        echo "Found suitable Python: $PYTHON_CMD"
    else
        echo "Error: Python $REQUIRED_MAJOR.$REQUIRED_MINOR or higher is required."
        echo "Please install Python $REQUIRED_MAJOR.$REQUIRED_MINOR or higher."
        return 1
    fi

    # Display Python version
    local PYTHON_VERSION
    PYTHON_VERSION=$($PYTHON_CMD --version)
    echo "Using $PYTHON_VERSION"

    # Create virtual environment if it doesn't exist
    if [ -d "$VENV_DIR" ]; then
        echo "Virtual environment already exists at $VENV_DIR"
        if [ "$RECREATE_VENV" = "yes" ]; then
            echo "Removing existing virtual environment (--recreate)."
            rm -rf "$VENV_DIR"
        elif [ "$RECREATE_VENV" = "no" ]; then
            echo "Using existing virtual environment (--no-recreate)."
        else
            read -p "Do you want to recreate it? (y/N): " -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                echo "Removing existing virtual environment..."
                rm -rf "$VENV_DIR"
            else
                echo "Using existing virtual environment."
            fi
        fi
    fi

    if [ ! -d "$VENV_DIR" ]; then
        echo "Creating virtual environment at $VENV_DIR..."
        $PYTHON_CMD -m venv "$VENV_DIR"
    fi

    # On Jetson: make system-installed TensorRT (apt) visible inside the venv
    if [ -f /etc/nv_tegra_release ]; then
        local SITE_PACKAGES="$VENV_DIR/lib/python$REQUIRED_MAJOR.$REQUIRED_MINOR/site-packages"
        local SYS_PYTHON_PATH="/usr/lib/python$REQUIRED_MAJOR.$REQUIRED_MINOR/dist-packages"
        if [ -d "$SYS_PYTHON_PATH" ]; then
            echo "Jetson: adding system Python path for TensorRT to venv..."
            echo "$SYS_PYTHON_PATH" > "$SITE_PACKAGES/jetson_system_packages.pth"
        else
            echo "Jetson: warning - $SYS_PYTHON_PATH not found; TensorRT may not be importable in venv."
        fi
    fi

    # Activate virtual environment
    echo "Activating virtual environment..."
    source "$VENV_DIR/bin/activate"

    # Upgrade pip, setuptools, and wheel
    echo "Upgrading pip, setuptools, and wheel..."
    pip install --upgrade pip setuptools wheel

    # On Jetson: install PyTorch wheel before pip install -e . so dependencies use it instead of pulling torch from PyPI
    if [ -f /etc/nv_tegra_release ]; then
        source "$SCRIPT_DIR/install_pytorch_jetson.sh"
        ensure_jetson_torch_in_venv
    fi

    # Check if pyproject.toml exists
    local PYPROJECT_FILE="$PROJECT_ROOT/pyproject.toml"
    if [ ! -f "$PYPROJECT_FILE" ]; then
        echo "Error: pyproject.toml not found at $PYPROJECT_FILE"
        return 1
    fi

    # Install project dependencies from pyproject.toml
    echo "Installing project dependencies from pyproject.toml..."
    pip install -e "$PROJECT_ROOT"

    echo ""
    echo "=========================================="
    echo "Virtual environment setup complete!"
    echo "=========================================="
    echo ""
    echo "To activate the virtual environment, run:"
    echo "  source $VENV_DIR/bin/activate"
    echo ""
    echo "Installed packages:"
    pip list
}
