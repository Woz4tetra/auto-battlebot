#!/usr/bin/env bash
# Run the Stage 4 sweep set against the Mrs Buff Mk3 plant and summarize.
# Usage: playground/control_stage0/run_stage4_sweeps.sh <step-tag> [sweep ...]
set -euo pipefail
cd "$(dirname "$0")/../.."
TAG="${1:?usage: run_stage4_sweeps.sh <step-tag> [sweep ...]}"
shift
SWEEPS=("$@")
if [ ${#SWEEPS[@]} -eq 0 ]; then
  SWEEPS=(stage3_stop stage3_ram stage3_track)
fi
OUT="playground/control_stage0/sweep_out/$TAG"
mkdir -p "$OUT"
for s in "${SWEEPS[@]}"; do
  rm -rf "$OUT/$s"
  python playground/control_stage0/sim_sweep.py \
    --sweep "playground/control_stage0/sweeps/$s.toml" \
    --sim-config simulation/sim_mrs_buff_mk3.toml \
    --out "$OUT/$s" > "$OUT/$s.out" 2>&1
  fails=$(grep -c 'no MCAP produced' "$OUT/$s.out" || true)
  [ "$fails" = "0" ] || echo "!! $s: $fails runs produced no MCAP (see $OUT/$s.out)"
done
python playground/control_stage0/summarize_sweeps.py "$OUT" --tag "$TAG"
