#!/usr/bin/env sh
set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
ROOT="$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)"
VISUALIZER="$ROOT/visualizer"
SKIP_INSTALL="${SKIP_INSTALL:-0}"

cd "$VISUALIZER"
if [ "$SKIP_INSTALL" != "1" ] && [ ! -d "node_modules" ]; then
    echo "Installing visualizer npm dependencies..."
    npm install
fi

echo "Starting Tauri development app..."
exec npm run tauri -- dev
