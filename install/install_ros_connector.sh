#!/bin/bash
install_ros_connector() {
    BASE_DIR=$(realpath "$(dirname $0)")
    local docker_dir="${BASE_DIR}/../docker"

    docker build -f "${docker_dir}/ros-connector.Dockerfile" -t ros-connector:latest ${docker_dir}
}
