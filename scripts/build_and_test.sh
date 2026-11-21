#!/bin/bash

set -e

usage() {
    cat <<'EOF'
Usage: scripts/build_and_test.sh [--valgrind | --asan | --tsan] [GTEST_ARGS...]

Build the project with testing enabled and run the auto_battlebot unit tests.
All arguments except --valgrind, --asan and --tsan are passed to the Google Test executable.

Options:
  --valgrind    Run tests under the Valgrind memory analyzer
  --asan        Build in build-asan/ with AddressSanitizer and run the tests there
  --tsan        Build in build-tsan/ with ThreadSanitizer and run the tests there
  -h, --help    Show this help; use --gtest_help to show all GTest options

Common GTest arguments:
  --gtest_filter=PATTERN   Run only tests matching the pattern
                           Examples: --gtest_filter=KeypointTest.*
                                     --gtest_filter=*RobotDescription*
                                     --gtest_filter=KeypointTest.DefaultConstruction
  --gtest_list_tests       List all available tests without running them
  --gtest_repeat=N         Run tests N times
  --gtest_shuffle          Randomize test execution order
  --gtest_brief=1          Brief output (only show failures)

Examples:
  ./scripts/build_and_test.sh                                     Run all tests
  ./scripts/build_and_test.sh --gtest_filter=KeypointTest.*       Run only Keypoint tests
  ./scripts/build_and_test.sh --gtest_list_tests                  List all tests
  ./scripts/build_and_test.sh --valgrind                          Run all tests under Valgrind
  ./scripts/build_and_test.sh --valgrind --gtest_filter=*Camera*  Run Camera tests under Valgrind
  ./scripts/build_and_test.sh --tsan --gtest_filter=*Encoder*     Run encoder tests under ThreadSanitizer
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="${PROJECT_ROOT}/build-test"
USE_VALGRIND=false
SANITIZER=none

GTEST_ARGS=()
for arg in "$@"; do
    if [ "$arg" = "-h" ] || [ "$arg" = "--help" ]; then
        usage
        exit 0
    elif [ "$arg" = "--valgrind" ]; then
        USE_VALGRIND=true
    elif [ "$arg" = "--asan" ]; then
        SANITIZER=address
    elif [ "$arg" = "--tsan" ]; then
        SANITIZER=thread
    else
        GTEST_ARGS+=("$arg")
    fi
done

if [ "$SANITIZER" != none ] && [ "$USE_VALGRIND" = true ]; then
    echo "--valgrind cannot be combined with --asan or --tsan" >&2
    exit 2
fi
if [ "$SANITIZER" = address ]; then
    BUILD_DIR="${PROJECT_ROOT}/build-asan"
elif [ "$SANITIZER" = thread ]; then
    BUILD_DIR="${PROJECT_ROOT}/build-tsan"
fi

source "$PROJECT_ROOT/install/build_cpp_project.sh"
AUTO_BATTLEBOT_BUILD_DIR="$BUILD_DIR" AUTO_BATTLEBOT_SANITIZER="$SANITIZER" build_cpp_project --test

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}Running auto_battlebot unit tests...${NC}"
echo ""

if [ ! -d "$BUILD_DIR" ]; then
    echo -e "${RED}Build directory not found. Build failed!.${NC}"
    exit 1
fi

TEST_EXECUTABLE="${BUILD_DIR}/auto_battlebot_test"
if [ ! -f "$TEST_EXECUTABLE" ]; then
    echo -e "${RED}Test executable not found. Build failed!${NC}"
    exit 1
fi

if [ "$USE_VALGRIND" = true ]; then
    echo -e "${YELLOW}Running tests under Valgrind (memory analysis)...${NC}"
    echo "========================================"
else
    echo -e "${YELLOW}Running data structures tests...${NC}"
    echo "========================================"
fi
cd "$BUILD_DIR"

if [ "$USE_VALGRIND" = true ]; then
    TEST_CMD="valgrind --suppressions=${PROJECT_ROOT}/valgrind.supp --leak-check=full $TEST_EXECUTABLE"
elif [ "$SANITIZER" = address ]; then
    export ASAN_OPTIONS="protect_shadow_gap=0:replace_intrin=0${ASAN_OPTIONS:+:$ASAN_OPTIONS}"
    TEST_CMD="$TEST_EXECUTABLE"
elif [ "$SANITIZER" = thread ]; then
    TEST_CMD="setarch -R $TEST_EXECUTABLE"
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
