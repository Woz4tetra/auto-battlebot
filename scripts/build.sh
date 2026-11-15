#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}Building auto_battlebot...${NC}"

source "$PROJECT_ROOT/install/build_cpp_project.sh"
build_cpp_project

# The web dashboard viz_relay serves. Optional: without Node the relay answers 503 with a note.
if command -v node >/dev/null 2>&1; then
    "$SCRIPT_DIR/build_web.sh"
else
    echo -e "${YELLOW}node not found; skipping the web dashboard (install/install_node.sh)${NC}"
fi

echo ""
echo -e "${GREEN}✓ Build complete!${NC}"
echo "========================================"
echo ""
