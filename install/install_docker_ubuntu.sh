#!/bin/bash

install_docker_ubuntu() {
    if command -v docker >/dev/null 2>&1; then
        echo "Docker is installed. Skipping docker installation"
        return 0
    fi

    # Update your system's package list
    sudo apt-get update

    # Download and run the Docker installation script
    curl https://get.docker.com | sh && sudo systemctl --now enable docker

    # Add your user to the 'docker' group to run commands without sudo
    sudo usermod -aG docker $USER
    newgrp docker # Apply group changes immediately

    docker run hello-world  # test docker installation
}
