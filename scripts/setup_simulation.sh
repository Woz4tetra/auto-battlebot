#!/bin/bash

# Set up a dedicated Python virtual environment for the Genesis simulation server.
# The venv lives at simulation/venv, separate from the main project venv.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
SIM_DIR="$PROJECT_ROOT/simulation"
VENV_DIR="$SIM_DIR/venv"
REQUIREMENTS="$SIM_DIR/requirements.txt"

REQUIRED_MAJOR=3
REQUIRED_MINOR=10

# --- Find a suitable Python ---------------------------------------------------

find_python() {
    local cmd
    for cmd in python3 "python${REQUIRED_MAJOR}.${REQUIRED_MINOR}"; do
        if command -v "$cmd" &>/dev/null; then
            local ver
            ver=$($cmd -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
            local major=${ver%%.*}
            local minor=${ver##*.}
            if [ "$major" -gt "$REQUIRED_MAJOR" ] ||
               { [ "$major" -eq "$REQUIRED_MAJOR" ] && [ "$minor" -ge "$REQUIRED_MINOR" ]; }; then
                echo "$cmd"
                return 0
            fi
        fi
    done
    return 1
}

PYTHON_CMD=$(find_python) || {
    echo "Error: Python ${REQUIRED_MAJOR}.${REQUIRED_MINOR}+ is required."
    exit 1
}
echo "Using $($PYTHON_CMD --version)"

# --- Handle existing venv -----------------------------------------------------

RECREATE=""
while [ $# -gt 0 ]; do
    case "$1" in
        --recreate|-y) RECREATE=yes; shift ;;
        --no-recreate|-n) RECREATE=no; shift ;;
        *) shift ;;
    esac
done

if [ -d "$VENV_DIR" ]; then
    echo "Simulation venv already exists at $VENV_DIR"
    if [ "$RECREATE" = "yes" ]; then
        echo "Removing existing venv (--recreate)."
        rm -rf "$VENV_DIR"
    elif [ "$RECREATE" = "no" ]; then
        echo "Using existing venv (--no-recreate)."
    else
        read -p "Recreate it? (y/N): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$VENV_DIR"
        else
            echo "Using existing venv."
        fi
    fi
fi

# --- Create & populate --------------------------------------------------------

if [ ! -d "$VENV_DIR" ]; then
    echo "Creating simulation venv at $VENV_DIR ..."
    $PYTHON_CMD -m venv "$VENV_DIR"
fi

source "$VENV_DIR/bin/activate"

pip install --upgrade pip setuptools wheel

if [ ! -f "$REQUIREMENTS" ]; then
    echo "Error: $REQUIREMENTS not found."
    exit 1
fi

echo "Installing simulation dependencies from $REQUIREMENTS ..."
pip install -r "$REQUIREMENTS"

echo ""
echo "=========================================="
echo "Simulation venv setup complete!"
echo "=========================================="
echo ""
echo "Installed packages:"
pip list
