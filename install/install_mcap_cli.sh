#!/bin/bash
install_mcap_cli() {
    local install_dir="${HOME}/.local/bin"
    local binary_path="${install_dir}/mcap"

    if command -v mcap &>/dev/null; then
        echo "mcap CLI already installed: $(mcap version 2>/dev/null || echo 'unknown version')"
        return 0
    fi

    local arch
    arch="$(uname -m)"
    local asset
    case "$arch" in
        x86_64)  asset="mcap-linux-amd64" ;;
        aarch64) asset="mcap-linux-arm64" ;;
        *)
            echo "Unsupported architecture for mcap CLI: $arch"
            return 1
            ;;
    esac

    echo "Installing mcap CLI ($asset)..."
    # A fresh flash has no ~/.local/bin, and curl -o fails with "Failure writing output to
    # destination" rather than saying the directory is missing.
    mkdir -p "$install_dir"
    local url="https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2Fv0.0.62/${asset}"
    curl -fsSL "$url" -o "$binary_path"
    chmod +x "$binary_path"
    echo "mcap CLI installed to ${binary_path}"
    if ! command -v mcap &>/dev/null; then
        # Ubuntu's ~/.profile only adds ~/.local/bin to PATH when it exists at login, so on a
        # fresh flash the directory we just created is not on PATH until the next login.
        echo "Note: ${install_dir} is not on PATH yet. Log out and back in, or run:"
        echo "  export PATH=\"${install_dir}:\$PATH\""
    fi
}
