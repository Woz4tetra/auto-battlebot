#!/bin/bash

# Script to build and run auto_battlebot with debug flags enabled

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="${PROJECT_ROOT}/build"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Building auto_battlebot with debug flags...${NC}"

# Build with debug flags
source "$PROJECT_ROOT/install/build_cpp_project.sh"
build_cpp_project --debug

echo ""
echo -e "${GREEN}✓ Build complete!${NC}"
echo -e "${YELLOW}Running auto_battlebot in debug mode...${NC}"
echo "========================================"
echo ""

# Run the executable with any additional arguments passed to this script
${BUILD_DIR}/auto_battlebot "$@"
