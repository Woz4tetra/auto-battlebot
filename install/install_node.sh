#!/bin/bash
# Install Node.js 22 from NodeSource, for building the web dashboard (scripts/build_web.sh).
#
# Ubuntu 24.04's apt ships Node 18 and current Vite needs 20.19 or newer. NodeSource publishes
# amd64 and arm64, so the dev boxes and the Jetson build the page the same way. A node already on
# PATH at 20.19 or newer (nvm, say) is left alone.

install_node() {
    local major=22
    if command -v node >/dev/null 2>&1; then
        local version
        version=$(node --version | sed 's/^v//')
        local have_major=${version%%.*}
        local rest=${version#*.}
        local have_minor=${rest%%.*}
        if [ "$have_major" -gt 20 ] || { [ "$have_major" -eq 20 ] && [ "$have_minor" -ge 19 ]; }; then
            echo "Node $version already installed"
            return 0
        fi
        echo "Node $version is too old for Vite; installing Node $major"
    fi

    sudo apt-get install -y ca-certificates curl gnupg
    sudo mkdir -p /etc/apt/keyrings
    curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key |
        sudo gpg --dearmor --yes -o /etc/apt/keyrings/nodesource.gpg
    echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_${major}.x nodistro main" |
        sudo tee /etc/apt/sources.list.d/nodesource.list >/dev/null
    sudo apt-get update
    sudo apt-get install -y nodejs
    echo "Installed Node $(node --version)"
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    set -e
    install_node
fi
