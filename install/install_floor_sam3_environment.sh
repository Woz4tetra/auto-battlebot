#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
VENV_DIR="$PROJECT_ROOT/venv"

SAM3_PATH=""
SAM3_GIT_URL=""
SKIP_BASE_SETUP=0

usage() {
    cat <<'EOF'
Usage:
  install/install_floor_sam3_environment.sh [options]

Options:
  --sam3-path <path>      Local SAM3 repository path to install with pip -e
  --sam3-git-url <url>    Git URL for SAM3 repository (cloned to training/floor/third_party/sam3)
  --skip-base-setup       Skip base venv setup script (assumes venv already exists)
  -h, --help              Show help

Notes:
  1) Installs floor pipeline dependencies from pyproject extra: .[floor]
  2) Installs SAM3 separately from local path or git URL
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --sam3-path)
            SAM3_PATH="${2:-}"
            shift 2
            ;;
        --sam3-git-url)
            SAM3_GIT_URL="${2:-}"
            shift 2
            ;;
        --skip-base-setup)
            SKIP_BASE_SETUP=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            usage
            exit 1
            ;;
    esac
done

if [ "$SKIP_BASE_SETUP" -eq 0 ]; then
    # Reuse existing project bootstrap so base dependencies stay consistent.
    # shellcheck source=/dev/null
    source "$SCRIPT_DIR/install_python_environment.sh"
    install_python_environment --no-recreate
fi

if [ ! -f "$VENV_DIR/bin/activate" ]; then
    echo "Error: virtual environment not found at $VENV_DIR"
    echo "Run install/install_python_environment.sh first, or omit --skip-base-setup."
    exit 1
fi

# shellcheck source=/dev/null
source "$VENV_DIR/bin/activate"

echo "Installing floor pipeline extras from pyproject..."
pip install --upgrade pip setuptools wheel
pip install -e "${PROJECT_ROOT}[floor]"

install_sam3_from_path() {
    local repo_path="$1"
    if [ ! -d "$repo_path" ]; then
        echo "Error: --sam3-path does not exist: $repo_path"
        exit 1
    fi
    echo "Installing SAM3 from local path: $repo_path"
    pip install -e "$repo_path"
    if [ -f "$repo_path/requirements.txt" ]; then
        pip install -r "$repo_path/requirements.txt"
    fi
}

install_sam3_from_git() {
    local git_url="$1"
    local clone_dir="$PROJECT_ROOT/training/floor/third_party/sam3"

    mkdir -p "$PROJECT_ROOT/training/floor/third_party"
    if [ -d "$clone_dir/.git" ]; then
        echo "Updating existing SAM3 checkout at $clone_dir"
        git -C "$clone_dir" pull --ff-only
    else
        echo "Cloning SAM3 from $git_url"
        git clone "$git_url" "$clone_dir"
    fi

    echo "Installing SAM3 from git checkout: $clone_dir"
    pip install -e "$clone_dir"
    if [ -f "$clone_dir/requirements.txt" ]; then
        pip install -r "$clone_dir/requirements.txt"
    fi
}

if [ -n "$SAM3_PATH" ]; then
    install_sam3_from_path "$SAM3_PATH"
elif [ -n "$SAM3_GIT_URL" ]; then
    install_sam3_from_git "$SAM3_GIT_URL"
else
    echo "Error: SAM3 source is required."
    echo "Provide either --sam3-path or --sam3-git-url."
    exit 1
fi

echo ""
echo "=========================================="
echo "Floor + SAM3 environment setup complete"
echo "=========================================="
echo "Virtualenv: $VENV_DIR"
echo "Activate with:"
echo "  source \"$VENV_DIR/bin/activate\""
