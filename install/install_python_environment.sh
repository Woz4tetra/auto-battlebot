#!/bin/bash

# Script to initialize a Python virtual environment and install project dependencies
# This script creates a venv in the project root and installs packages from pyproject.toml

install_python_environment() {
    set -e  # Exit on error

    # Get the project root directory (parent of install/ directory)
    local SCRIPT_DIR
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local PROJECT_ROOT
    PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

    echo "Project root: $PROJECT_ROOT"

    # Virtual environment directory
    local VENV_DIR="$PROJECT_ROOT/venv"

    # Check if Python 3 is installed
    if ! command -v python3 &> /dev/null; then
        echo "Error: python3 is not installed. Please install Python 3."
        return 1
    fi

    # Display Python version
    local PYTHON_VERSION
    PYTHON_VERSION=$(python3 --version)
    echo "Using $PYTHON_VERSION"

    # Create virtual environment if it doesn't exist
    if [ -d "$VENV_DIR" ]; then
        echo "Virtual environment already exists at $VENV_DIR"
        read -p "Do you want to recreate it? (y/N): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            echo "Removing existing virtual environment..."
            rm -rf "$VENV_DIR"
        else
            echo "Using existing virtual environment."
        fi
    fi

    if [ ! -d "$VENV_DIR" ]; then
        echo "Creating virtual environment at $VENV_DIR..."
        python3 -m venv "$VENV_DIR"
    fi

    # Activate virtual environment
    echo "Activating virtual environment..."
    source "$VENV_DIR/bin/activate"

    # Upgrade pip, setuptools, and wheel
    echo "Upgrading pip, setuptools, and wheel..."
    pip install --upgrade pip setuptools wheel

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
