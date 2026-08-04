#!/bin/bash

# docker/docker-compose.playback.yml uses the `include:` key, added in Compose v2.20.
# Older versions reject the whole file with the unhelpful message
# "(root) Additional property include is not allowed".
MIN_COMPOSE_VERSION="2.20"

# Verifies the Compose plugin is new enough and points at the fix when it is not.
#
# The usual cause of an old version is a stale manually-installed plugin shadowing the
# apt one: docker searches /usr/local/lib/docker/cli-plugins before
# /usr/libexec/docker/cli-plugins, so a leftover binary in the former wins even after
# docker-compose-plugin is upgraded. That is exactly what happened on megamind, which
# reported 2.11.2 while the apt package on disk was current.
check_docker_compose_version() {
    local version
    version=$(docker compose version --short 2>/dev/null | sed 's/^v//')

    if [ -z "$version" ]; then
        echo "Error: 'docker compose' is unavailable. Install the docker-compose-plugin package."
        return 1
    fi

    # sort -V puts the lower version first, so if the minimum sorts first we are fine.
    # This also handles the 5.x numbering shipped with recent Docker Engine releases.
    if [ "$(printf '%s\n%s\n' "$MIN_COMPOSE_VERSION" "$version" | sort -V | head -1)" != "$MIN_COMPOSE_VERSION" ]; then
        echo "Error: docker compose $version is too old; $MIN_COMPOSE_VERSION or newer is required."
        echo ""
        echo "Check for a stale plugin shadowing the packaged one:"
        echo "  ls -l /usr/local/lib/docker/cli-plugins/docker-compose"
        echo "  /usr/libexec/docker/cli-plugins/docker-compose version --short"
        echo ""
        echo "If the /usr/local/lib copy is older, move it aside:"
        echo "  sudo mv /usr/local/lib/docker/cli-plugins/docker-compose{,.disabled}"
        echo ""
        echo "Otherwise upgrade the package:"
        echo "  sudo apt-get update && sudo apt-get install --only-upgrade docker-compose-plugin"
        return 1
    fi

    echo "docker compose $version meets the $MIN_COMPOSE_VERSION minimum."
    return 0
}

install_docker_ubuntu() {
    if command -v docker >/dev/null 2>&1; then
        echo "Docker is installed. Skipping docker installation"
        check_docker_compose_version
        return $?
    fi

    # Update your system's package list
    sudo apt-get update

    # Download and run the Docker installation script
    curl https://get.docker.com | sh && sudo systemctl --now enable docker

    # Add your user to the 'docker' group to run commands without sudo
    sudo usermod -aG docker "${USER}"
    newgrp docker # Apply group changes immediately
    sudo setfacl --modify user:"${USER}":rw /var/run/docker.sock

    docker run hello-world  # test docker installation

    check_docker_compose_version
}
