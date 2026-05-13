#!/usr/bin/env sh
set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
ROOT="$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)"
APP="tauri"

if [ "${1:-}" = "--app" ] || [ "${1:-}" = "-a" ]; then
    APP="${2:-tauri}"
    shift 2
fi

case "$APP" in
    rust)
        EXE="$ROOT/target/release/gd-real-sim"
        ;;
    tauri)
        EXE="$ROOT/visualizer/src-tauri/target/release/gd-real-sim-visualizer"
        ;;
    *)
        echo "unknown app '$APP' (expected 'tauri' or 'rust')" >&2
        exit 2
        ;;
esac

if [ ! -x "$EXE" ] && [ ! -f "$EXE" ]; then
    echo "built executable not found: $EXE" >&2
    echo "run scripts/build.sh first" >&2
    exit 1
fi

echo "Running $EXE"
exec "$EXE" "$@"
