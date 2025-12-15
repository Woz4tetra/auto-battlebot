#!/bin/bash

# Script to build and run auto_battlebot with debug flags enabled

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

${SCRIPT_DIR}/build.sh
${SCRIPT_DIR}/run.sh "$@"
