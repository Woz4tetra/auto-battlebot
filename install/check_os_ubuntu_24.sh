#!/bin/bash

# Check if running on Ubuntu 24
check_os_ubuntu_24() {
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        if [ "$ID" != "ubuntu" ] || [ "$VERSION_ID" != "24.04" ]; then
            echo "Error: This script requires Ubuntu 24.04"
            echo "Current OS: $ID $VERSION_ID"
            exit 1
        fi
    else
        echo "Error: Cannot determine OS. /etc/os-release not found."
        exit 1
    fi
}
