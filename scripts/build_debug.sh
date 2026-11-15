#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}Building auto_battlebot with debug flags...${NC}"

source "$PROJECT_ROOT/install/build_cpp_project.sh"
build_cpp_project --test

echo ""
echo -e "${GREEN}✓ Debug build complete!${NC}"
echo "========================================"
echo ""
