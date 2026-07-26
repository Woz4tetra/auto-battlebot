#!/bin/bash

# Install the pinned LLVM tooling: clang-format, clang-tidy, clangd.
#
# The unversioned distro packages track whichever LLVM the release happens to
# ship -- 14 on Ubuntu 22.04 and JetPack, 18 on Ubuntu 24.04 -- and those
# versions disagree about formatting (clang-format 14 writes `override{}`, 18
# writes `override {}`). An unpinned toolchain therefore rewrites the tree
# differently depending on which machine ran scripts/lint. Pin every machine to
# the version in .llvm-version instead.
#
# Ubuntu 24.04 carries clang-*-18 in its own repos. Older releases do not, so
# apt.llvm.org is added for them (it publishes amd64 and arm64, covering both
# the dev boxes and the Jetson).
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

# Also runnable directly, so the toolchain can be repaired without redoing a
# full platform install (scripts/lint points here when the pin is missing).
if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    set -e
    install_llvm_toolchain
fi
