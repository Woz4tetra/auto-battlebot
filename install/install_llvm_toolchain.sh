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
    local project_root="${1:?project root required}"
    local version_file="$project_root/.llvm-version"

    if [ ! -f "$version_file" ]; then
        echo "Error: $version_file not found"
        exit 1
    fi

    local version
    version=$(tr -d '[:space:]' < "$version_file")

    local packages=("clang-format-$version" "clang-tidy-$version" "clangd-$version")

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

    local missing=()
    local pkg
    for pkg in "${packages[@]}"; do
        if ! dpkg -s "$pkg" >/dev/null 2>&1; then
            missing+=("$pkg")
        fi
    done

    if [ ${#missing[@]} -eq 0 ]; then
        echo "LLVM $version tooling already installed"
        return
    fi

    echo "Installing LLVM $version tooling: ${missing[*]}"
    sudo apt install -y "${missing[@]}"
}
