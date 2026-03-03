#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/synthetic_venv"

PYTHON_CMD=""
for candidate in python3.11 python3; do
    if command -v "$candidate" &>/dev/null; then
        PYTHON_CMD="$candidate"
        break
    fi
done

if [ -z "$PYTHON_CMD" ]; then
    echo "Error: Python 3.10+ is required."
    exit 1
fi

echo "Using $($PYTHON_CMD --version)"

if [ -d "$VENV_DIR" ]; then
    echo "Virtual environment already exists at $VENV_DIR"
    if [ "$1" = "--recreate" ] || [ "$1" = "-y" ]; then
        echo "Removing existing virtual environment (--recreate)."
        rm -rf "$VENV_DIR"
    else
        echo "Using existing venv. Pass --recreate to start fresh."
    fi
fi

if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment at $VENV_DIR..."
    $PYTHON_CMD -m venv "$VENV_DIR"
fi

source "$VENV_DIR/bin/activate"

pip install --upgrade pip setuptools wheel

echo "Installing project dependencies..."
pip install -e "$SCRIPT_DIR"

echo ""
echo "=========================================="
echo "Setup complete!"
echo "=========================================="
echo ""
echo "Activate with:"
echo "  source $VENV_DIR/bin/activate"
echo ""
echo "Optional extras:"
echo "  pip install -e '$SCRIPT_DIR[sam3d]'      # SAM 3D mesh conversion"
echo "  pip install -e '$SCRIPT_DIR[rembg]'      # Background removal for SAM 3D"
