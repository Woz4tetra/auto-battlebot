#!/bin/bash

# Check if running on Ubuntu 22
check_os_ubuntu_22() {
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        if [ "$ID" != "ubuntu" ] || [ "$VERSION_ID" != "22.04" ]; then
            echo "Error: This script requires Ubuntu 22.04"
            echo "Current OS: $ID $VERSION_ID"
            exit 1
        fi
    else
        echo "Error: Cannot determine OS. /etc/os-release not found."
        exit 1
    fi
}
