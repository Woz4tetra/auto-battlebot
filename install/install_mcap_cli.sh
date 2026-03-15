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
    local url="https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2Fv0.0.62/${asset}"
    curl -fsSL "$url" -o "$binary_path"
    chmod +x "$binary_path"
    echo "mcap CLI installed to ${binary_path}"
}
