#!/bin/bash

MIN_COMPOSE_VERSION="2.20"

version_at_least() {
    [ "$(printf '%s\n%s\n' "$2" "$1" | sort -V | head -1)" = "$2" ]
}

check_docker_compose_version() {
    local version
    version=$(docker compose version --short 2>/dev/null | sed 's/^v//')

    if [ -z "$version" ]; then
        echo "Error: 'docker compose' is unavailable. Install the docker-compose-plugin package."
        return 1
    fi

    if ! version_at_least "$version" "$MIN_COMPOSE_VERSION"; then
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

    sudo apt-get update

    curl https://get.docker.com | sh && sudo systemctl --now enable docker

    sudo usermod -aG docker "${USER}"
    newgrp docker
    sudo setfacl --modify user:"${USER}":rw /var/run/docker.sock

    docker run hello-world

    check_docker_compose_version
}
