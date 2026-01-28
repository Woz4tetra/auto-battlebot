#!/bin/bash

# Script to run unit tests for auto_battlebot
#
# This script builds the project with testing enabled and runs the test suite.
# All arguments are passed directly to the Google Test executable.
#
# Options:
#   --valgrind                     Run tests under Valgrind memory analyzer
#   --help-gtest                   Show GTest options (instead of running tests)
#
# Common GTest arguments:
#   --gtest_filter=PATTERN         Run only tests matching the pattern
#                                  Examples: --gtest_filter=KeypointTest.*
#                                           --gtest_filter=*RobotDescription*
#                                           --gtest_filter=KeypointTest.DefaultConstruction
#   --gtest_list_tests             List all available tests without running them
#   --gtest_repeat=N               Run tests N times
#   --gtest_shuffle                Randomize test execution order
#   --gtest_brief=1                Brief output (only show failures)
#   --help                         Show all GTest options
#
# Examples:
#   ./scripts/build_and_test.sh                                    # Run all tests
#   ./scripts/build_and_test.sh --gtest_filter=KeypointTest.*      # Run only Keypoint tests
#   ./scripts/build_and_test.sh --gtest_list_tests                 # List all tests
#   ./scripts/build_and_test.sh --valgrind                         # Run all tests under Valgrind
#   ./scripts/build_and_test.sh --valgrind --gtest_filter=*Camera* # Run Camera tests under Valgrind

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="${PROJECT_ROOT}/build-test"
USE_VALGRIND=false

# Parse script-specific arguments
GTEST_ARGS=()
for arg in "$@"; do
    if [ "$arg" = "--valgrind" ]; then
        USE_VALGRIND=true
    else
        GTEST_ARGS+=("$arg")
    fi
done

source "$PROJECT_ROOT/install/build_cpp_project.sh"
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
    echo -e "${RED}Build directory not found. Build failed!.${NC}"
    exit 1
fi

# Check if test executable exists
TEST_EXECUTABLE="${BUILD_DIR}/auto_battlebot_test"
if [ ! -f "$TEST_EXECUTABLE" ]; then
    echo -e "${RED}Test executable not found. Build failed!${NC}"
    exit 1
fi

# Run the tests
if [ "$USE_VALGRIND" = true ]; then
    echo -e "${YELLOW}Running tests under Valgrind (memory analysis)...${NC}"
    echo "========================================"
else
    echo -e "${YELLOW}Running data structures tests...${NC}"
    echo "========================================"
fi
cd "$BUILD_DIR"

# Build the command
if [ "$USE_VALGRIND" = true ]; then
    TEST_CMD="valgrind --suppressions=${PROJECT_ROOT}/valgrind.supp --leak-check=full $TEST_EXECUTABLE"
else
    TEST_CMD="$TEST_EXECUTABLE"
fi

if $TEST_CMD "${GTEST_ARGS[@]}"; then
    echo ""
    echo -e "${GREEN}✓ All tests passed!${NC}"
    exit 0
else
    echo ""
    echo -e "${RED}✗ Some tests failed!${NC}"
    exit 1
fi
