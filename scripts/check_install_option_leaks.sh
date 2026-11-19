#!/bin/bash
root="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
leaks=$(grep -nE '^[[:space:]]*(set[[:space:]]+[-+]|trap[[:space:]])' \
    "$root"/install/*.sh "$root"/scripts/install_*.sh |
    grep -v '/install/fail_loudly.sh:' |
    grep -vE ':[0-9]+:[[:space:]]*set -e[[:space:]]*(#.*)?$' |
    grep -v '/install/install_opencv.sh:') || true
if [ -n "$leaks" ]; then
    echo "$leaks"
    echo "Move the option or trap into install/fail_loudly.sh, or scope it in a subshell."
    exit 1
fi
