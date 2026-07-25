#!/bin/bash
#
# Install the clang-format / clang-tidy / clangd version pinned in
# .llvm-version. Run this when scripts/lint reports the pinned tooling is
# missing; the full install_ubuntu_*.sh / install_jetson.sh scripts call it too.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

source "$PROJECT_ROOT/install/install_llvm_toolchain.sh"

install_llvm_toolchain "$PROJECT_ROOT"
