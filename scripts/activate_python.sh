#!/bin/bash

# Convenience script to activate the Python virtual environment
# Usage: source ./activate.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
VENV_DIR="$PROJECT_ROOT/venv"

if [ ! -d "$VENV_DIR" ]; then
    echo "Error: Virtual environment not found at $VENV_DIR"
    echo "Run ./setup_python.sh first to create the virtual environment."
    return 1 2>/dev/null || exit 1
fi

source "$VENV_DIR/bin/activate"
echo "Virtual environment activated: $VENV_DIR"
