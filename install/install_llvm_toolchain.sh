#!/bin/bash

install_llvm_toolchain() {
    local project_root
    project_root=$(dirname "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")")

    local version_file="$project_root/.llvm-version"
    if [ ! -f "$version_file" ]; then
        echo "Error: $version_file not found"
        return 1
    fi

    local version
    version=$(tr -d '[:space:]' < "$version_file")

    if command -v "clang-format-$version" &>/dev/null \
        && command -v "clang-tidy-$version" &>/dev/null \
        && command -v "clangd-$version" &>/dev/null; then
        echo "LLVM $version tooling already installed: $("clang-format-$version" --version)"
        return 0
    fi

    if ! apt-cache show "clang-tidy-$version" >/dev/null 2>&1; then
        local codename
        codename=$(lsb_release -cs)
        echo "clang-tidy-$version is not in the distro repos; adding apt.llvm.org for $codename"
        wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key \
            | sudo tee /etc/apt/trusted.gpg.d/apt.llvm.org.asc >/dev/null
        echo "deb http://apt.llvm.org/$codename/ llvm-toolchain-$codename-$version main" \
            | sudo tee "/etc/apt/sources.list.d/llvm-toolchain-$version.list" >/dev/null
        sudo apt update
    fi

    echo "Installing LLVM $version tooling..."
    sudo apt install -y "clang-format-$version" "clang-tidy-$version" "clangd-$version"
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    set -e
    install_llvm_toolchain
fi
