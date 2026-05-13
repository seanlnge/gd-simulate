#!/usr/bin/env sh
set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
ROOT="$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)"
VISUALIZER="$ROOT/visualizer"
SKIP_INSTALL="${SKIP_INSTALL:-0}"

cd "$ROOT"
echo "Building gd-real-sim Rust CLI (debug, for Tauri native visualizer launches)..."
cargo build

echo "Building gd-real-sim Rust CLI (release)..."
cargo build --release

cd "$VISUALIZER"
if [ "$SKIP_INSTALL" != "1" ] && [ ! -d "node_modules" ]; then
    echo "Installing visualizer npm dependencies..."
    npm install
fi

echo "Building Tauri visualizer bundle..."
npm run tauri -- build

echo
echo "Build complete."
echo "Rust CLI: $ROOT/target/release/gd-real-sim"
echo "Tauri app: $VISUALIZER/src-tauri/target/release/gd-real-sim-visualizer"
