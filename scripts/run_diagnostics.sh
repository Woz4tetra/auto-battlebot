#!/bin/bash
# Run the Jetson lockup diagnostic probes sequentially, tee output to logs/,
# and emit a final pass/fail/inconclusive table. Triggers diag_collect_jetson.py
# whenever a probe exits non-zero so a hang/error is captured with system state.
#
# Build prerequisite:
#   ./scripts/build.sh -- -DBUILD_DIAGNOSTICS=ON
#
# Usage:
#   ./scripts/run_diagnostics.sh [--quick|--soak] [--only <name>] [--config <path>]

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="${BUILD_DIR:-$PROJECT_ROOT/build}"
LOG_DIR="${LOG_DIR:-$PROJECT_ROOT/logs}"
COLLECT_PY="$SCRIPT_DIR/diag_collect_jetson.py"

mkdir -p "$LOG_DIR"

mode="default"
only=""
config=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --quick) mode="quick"; shift ;;
        --soak)  mode="soak";  shift ;;
        --only)  only="$2"; shift 2 ;;
        --config) config="$2"; shift 2 ;;
        -h|--help)
            grep -E '^# ' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *) echo "unknown arg: $1" >&2; exit 1 ;;
    esac
done

# Per-probe duration in seconds, indexed by mode.
case "$mode" in
    quick)   short=20;  medium=60;   long=120 ;;
    soak)    short=300; medium=1800; long=1800 ;;
    *)       short=60;  medium=300;  long=600 ;;
esac

# Run order matches the recommended bisection order in the plan.
# Format: name,duration_class,extra_args
PROBES=(
    "diag_cuda_thermal,short,--temp-threshold-c 80"
    "diag_tegrastats,short,"
    "diag_ups_i2c,short,"
    "diag_serial_opentx,short,"
    "diag_zed_grab,medium,"
    "diag_trt_loop,medium,"
    "diag_mcap_disk,medium,--out-dir /tmp/diag_mcap"
    "diag_zed_trt_combined,long,"
)

declare -a results

run_one() {
    local name="$1"
    local duration_s="$2"
    local extra="$3"

    local bin="$BUILD_DIR/$name"
    if [[ ! -x "$bin" ]]; then
        echo "[skip] $name: binary not found at $bin"
        results+=("$name,skip,binary_missing")
        return
    fi

    local stamp
    stamp="$(date -u +%Y%m%dT%H%M%SZ)"
    local log="$LOG_DIR/${name}_${stamp}.log"

    local cmd="$bin --duration-s $duration_s $extra"
    echo
    echo "=================================================="
    echo "RUN: $cmd"
    echo "LOG: $log"
    echo "=================================================="

    set +e
    # shellcheck disable=SC2086
    $cmd 2>&1 | tee "$log"
    local rc="${PIPESTATUS[0]}"
    set -e

    case "$rc" in
        0)  results+=("$name,pass,$log") ;;
        10) results+=("$name,HANG,$log");   collect_state "$name" "$log" ;;
        20) results+=("$name,errors,$log"); collect_state "$name" "$log" ;;
        30) results+=("$name,inconclusive,$log") ;;
        40) results+=("$name,setup_failure,$log") ;;
        *)  results+=("$name,exit_$rc,$log");  collect_state "$name" "$log" ;;
    esac
}

collect_state() {
    local name="$1"
    local log="$2"
    if [[ ! -x "$COLLECT_PY" ]]; then
        echo "[warn] $COLLECT_PY not executable, skipping system snapshot"
        return
    fi
    echo "--- collecting Jetson system state ---"
    python3 "$COLLECT_PY" --tag "$name" --append-to "$log" || true
}

# Map duration class to seconds.
duration_for_class() {
    case "$1" in
        short)  echo "$short" ;;
        medium) echo "$medium" ;;
        long)   echo "$long" ;;
        *)      echo "$short" ;;
    esac
}

cd "$PROJECT_ROOT"

for entry in "${PROBES[@]}"; do
    IFS=',' read -r name dur_class extra <<<"$entry"
    if [[ -n "$only" && "$only" != "$name" ]]; then
        continue
    fi
    duration_s="$(duration_for_class "$dur_class")"
    if [[ -n "$config" ]]; then
        extra="$extra --config $config"
    fi
    run_one "$name" "$duration_s" "$extra"
done

echo
echo "=================================================="
echo "DIAGNOSTIC SUMMARY"
echo "=================================================="
printf "%-26s %-16s %s\n" "PROBE" "RESULT" "LOG"
for line in "${results[@]}"; do
    IFS=',' read -r name result log <<<"$line"
    printf "%-26s %-16s %s\n" "$name" "$result" "$log"
done
