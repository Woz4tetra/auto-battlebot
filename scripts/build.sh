#!/bin/bash

# Script to build and run auto_battlebot with debug flags enabled

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Building auto_battlebot...${NC}"

# Build with debug flags
source "$PROJECT_ROOT/install/build_cpp_project.sh"
build_cpp_project

echo ""
echo -e "${GREEN}✓ Build complete!${NC}"
echo -e "${YELLOW}Running auto_battlebot...${NC}"
echo "========================================"
echo ""
