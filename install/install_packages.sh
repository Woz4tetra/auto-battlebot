#!/bin/bash

# Check and install required packages from a file
install_packages() {
    local packages_file="${1:-packages.txt}"
    
    if [ ! -f "$packages_file" ]; then
        echo "Error: $packages_file not found"
        exit 1
    fi

    # Read packages from file (one per line, ignoring empty lines and comments)
    local package_entries
    package_entries=$(grep -v '^#' "$packages_file" | grep -v '^$')

    if [ -z "$package_entries" ]; then
        echo "No packages specified in $packages_file"
        return
    fi

    # First pass: add any repositories
    local repos_added=false
    while IFS= read -r entry; do
        if [[ "$entry" == *@* ]]; then
            local pkg="${entry%%@*}"
            local repo="${entry#*@}"
            
            # Check if repository is already added
            if ! grep -q "^deb .*${repo#ppa:}" /etc/apt/sources.list /etc/apt/sources.list.d/* 2>/dev/null; then
                echo "Adding repository: $repo"
                sudo apt-add-repository -y "$repo"
                repos_added=true
            else
                echo "Repository already added: $repo"
            fi
        fi
    done <<< "$package_entries"

    # Update apt cache if any repositories were added
    if [ "$repos_added" = true ]; then
        echo "Updating package lists..."
        sudo apt update
    fi

    # Second pass: check and install packages
    echo "Checking required packages..."
    local missing_packages=""

    while IFS= read -r entry; do
        # Extract package name (remove @repo if present)
        local pkg="${entry%%@*}"
        
        if ! dpkg -s "$pkg" >/dev/null 2>&1; then
            missing_packages="$missing_packages $pkg"
        fi
    done <<< "$package_entries"

    if [ -n "$missing_packages" ]; then
        echo "Installing missing packages:$missing_packages"
        if [ "$repos_added" = false ]; then
            sudo apt update
        fi
        sudo apt install -y $missing_packages
    else
        echo "All required packages already installed"
    fi
}
