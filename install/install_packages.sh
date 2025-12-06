#!/bin/bash

# Check and install required packages from a file
install_packages() {
    local packages_file="${1:-packages.txt}"
    
    if [ ! -f "$packages_file" ]; then
        echo "Error: $packages_file not found"
        exit 1
    fi

    # Read packages from file (one per line, ignoring empty lines and comments)
    local required_packages
    required_packages=$(grep -v '^#' "$packages_file" | grep -v '^$' | tr '\n' ' ')

    if [ -z "$required_packages" ]; then
        echo "No packages specified in $packages_file"
    else
        echo "Checking required packages..."
        local missing_packages=""

        for pkg in $required_packages; do
            if ! dpkg -s "$pkg" >/dev/null 2>&1; then
                missing_packages="$missing_packages $pkg"
            fi
        done

        if [ -n "$missing_packages" ]; then
            echo "Installing missing packages:$missing_packages"
            sudo apt update
            sudo apt install -y $missing_packages
        else
            echo "All required packages already installed"
        fi
    fi
}
