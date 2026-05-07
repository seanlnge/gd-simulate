#!/usr/bin/env bash
set -euo pipefail

# Run from anywhere: this script jumps to the crate root.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# Optional first arg or GD_LEVELS_SAVE env can override the local-levels file.
SAVE_PATH="${1:-${GD_LEVELS_SAVE:-$HOME/AppData/Local/GeometryDash/CCLocalLevels.dat}}"
TICK_LOG_OUT="${2:-examples/opti/opti_ticks.log}"

cd "$ROOT_DIR"

# Visualize "opti nerfdate" using the example click recording.
cargo run -- \
  --save "$SAVE_PATH" \
  --level "opti nerfdate" \
  --clicks-bin "examples/opti/opti_click_pattern.bin" \
  --visualize \
  --tick-log-out "$TICK_LOG_OUT"
