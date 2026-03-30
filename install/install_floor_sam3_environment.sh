#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
FLOOR_ROOT="$PROJECT_ROOT/training/floor"
VENV_DIR="$FLOOR_ROOT/.venv"
PYPROJECT_FILE="$FLOOR_ROOT/pyproject.toml"

SAM3_PATH=""
SAM3_GIT_URL=""
RECREATE_VENV=0

find_python() {
    if command -v python3.11 >/dev/null 2>&1; then
        echo "python3.11"
        return 0
    fi
    if command -v python3 >/dev/null 2>&1; then
        echo "python3"
        return 0
    fi
    return 1
}

usage() {
    cat <<'EOF'
Usage:
  install/install_floor_sam3_environment.sh [options]

Options:
  --sam3-path <path>      Local SAM3 repository path to install with pip -e
  --sam3-git-url <url>    Git URL for SAM3 repository (cloned to training/floor/third_party/sam3)
  --recreate              Recreate training/floor/.venv before install
  -h, --help              Show help

Notes:
  1) Uses isolated venv at training/floor/.venv
  2) Installs dependencies from training/floor/pyproject.toml
  3) Installs SAM3 separately from local path or git URL
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
        --recreate)
            RECREATE_VENV=1
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

if [ ! -f "$PYPROJECT_FILE" ]; then
    echo "Error: floor pyproject not found at $PYPROJECT_FILE"
    exit 1
fi

if [ "$RECREATE_VENV" -eq 1 ] && [ -d "$VENV_DIR" ]; then
    echo "Removing existing floor virtual environment at $VENV_DIR"
    rm -rf "$VENV_DIR"
fi

if [ ! -d "$VENV_DIR" ]; then
    PYTHON_CMD="$(find_python || true)"
    if [ -z "${PYTHON_CMD:-}" ]; then
        echo "Error: Could not find python3/python3.11"
        exit 1
    fi
    echo "Creating isolated floor environment with $PYTHON_CMD at $VENV_DIR"
    "$PYTHON_CMD" -m venv "$VENV_DIR"
fi

# shellcheck source=/dev/null
source "$VENV_DIR/bin/activate"

echo "Installing floor pipeline dependencies from $PYPROJECT_FILE"
pip install --upgrade pip setuptools wheel
pip install -e "$FLOOR_ROOT"

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

ensure_sam3_import_deps() {
    # Some SAM3 forks miss dependency metadata. Try importing and auto-install
    # missing top-level modules until import succeeds or attempts are exhausted.
    local max_attempts=8
    local attempt=1
    while [ "$attempt" -le "$max_attempts" ]; do
        local missing_mod
        missing_mod=$("$VENV_DIR/bin/python" - <<'PY'
import importlib
import traceback
try:
    importlib.import_module("sam3")
    print("OK")
except ModuleNotFoundError as e:
    print(e.name or "UNKNOWN")
except Exception:
    # sam3 may fail for runtime reasons unrelated to missing packages.
    print("OK")
PY
)
        if [ "$missing_mod" = "OK" ]; then
            return 0
        fi
        echo "SAM3 import missing dependency: $missing_mod"
        pip install "$missing_mod"
        attempt=$((attempt + 1))
    done
    echo "Warning: SAM3 import still has unresolved dependencies after $max_attempts attempts."
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

ensure_sam3_import_deps

echo ""
echo "=========================================="
echo "Floor + SAM3 environment setup complete"
echo "=========================================="
echo "Virtualenv: $VENV_DIR"
echo "Activate with:"
echo "  source \"$VENV_DIR/bin/activate\""
