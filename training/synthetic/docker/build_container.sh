#!/bin/bash
BASE_DIR=$(realpath "$(dirname $0)")
cd ${BASE_DIR}/../../..
docker build -f training/synthetic/Dockerfile -t auto-battlebot-synthetic training/synthetic