#!/bin/bash
install_taplo() {
    if command -v taplo &>/dev/null; then
        echo "Taplo already installed: $(taplo --version 2>/dev/null || echo 'unknown version')"
        return 0
    fi

    local arch
    arch="$(uname -m)"

    local asset
    case "$arch" in
        x86_64) asset="taplo-linux-x86_64.gz" ;;
        aarch64) asset="taplo-linux-aarch64.gz" ;;
        *)
            echo "Unsupported architecture for Taplo: $arch"
            return 1
            ;;
    esac

    local url="https://github.com/tamasfe/taplo/releases/latest/download/${asset}"
    echo "Installing Taplo from ${url}..."
    curl -fsSL "$url" | gzip -d | sudo install -m 755 /dev/stdin /usr/local/bin/taplo
    echo "Taplo installed to /usr/local/bin/taplo"
}
