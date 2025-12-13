#!/bin/bash

# Script to run unit tests for auto_battlebot

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

source "$SCRIPT_DIR/install/build_cpp_project.sh"
build_cpp_project --test

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Running auto_battlebot unit tests...${NC}"
echo ""

# Check if build directory exists
if [ ! -d "$BUILD_DIR" ]; then
    echo -e "${RED}Build directory not found. Please run build_desktop.sh first.${NC}"
    exit 1
fi

# Check if test executable exists
TEST_EXECUTABLE="${BUILD_DIR}/auto_battlebot_test"
if [ ! -f "$TEST_EXECUTABLE" ]; then
    echo -e "${YELLOW}Test executable not found. Building with tests enabled...${NC}"
    cd "$BUILD_DIR"
    cmake .. -DBUILD_TESTING=ON -Wno-dev
    make auto_battlebot_test -j$(nproc)
    echo ""
fi

# Run the tests
echo -e "${YELLOW}Running data structures tests...${NC}"
echo "========================================"
cd "$BUILD_DIR"

if "$TEST_EXECUTABLE" "$@"; then
    echo ""
    echo -e "${GREEN}✓ All tests passed!${NC}"
    exit 0
else
    echo ""
    echo -e "${RED}✗ Some tests failed!${NC}"
    exit 1
fi
