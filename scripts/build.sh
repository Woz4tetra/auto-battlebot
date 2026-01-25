#!/bin/bash

# Script to build auto_battlebot

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
echo "========================================"
echo ""
