#!/bin/bash

# Build the Jetson lockup diagnostic probes and run them sequentially.
# Forwards extra args (e.g. --quick, --soak, --only <name>, --config <path>)
# to scripts/run_diagnostics.sh.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="${BUILD_DIR:-$PROJECT_ROOT/build}"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

DIAG_TARGETS=(
    diag_zed_grab
    diag_trt_loop
    diag_cuda_thermal
    diag_zed_trt_combined
    diag_tegrastats
    diag_ups_i2c
    diag_serial_opentx
    diag_mcap_disk
)

echo -e "${YELLOW}Configuring build with -DBUILD_DIAGNOSTICS=ON...${NC}"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_DIAGNOSTICS=ON

echo -e "${YELLOW}Building diagnostic probes...${NC}"
make -j"$(nproc)" "${DIAG_TARGETS[@]}"

echo -e "${GREEN}Build complete. Running diagnostics...${NC}"
echo "========================================"
echo ""

cd "$PROJECT_ROOT"
"$SCRIPT_DIR/run_diagnostics.sh" "$@"
