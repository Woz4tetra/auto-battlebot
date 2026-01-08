#!/bin/bash
BASE_DIR=$(realpath "$(dirname $0)")

docker compose \
    -f ${BASE_DIR}/../docker/docker-compose.roscore.yml \
    pull
