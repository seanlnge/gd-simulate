# gd-real-sim

`gd-real-sim` is a parity-focused Geometry Dash simulator for replaying click tapes against real level data. It parses levelstrings/save data, simulates at 240 Hz, reports completion/death outcomes, optionally emits traces/logs, and includes a built-in visualizer for inspecting path + hitboxes.

## What This Project Does

- Simulates GD gameplay physics with a deterministic fixed-step loop.
- Supports click tapes from inline bits, text files, or `SP240BIN` recordings.
- Produces structured outcomes (`completed`, `died`, `timeout`) with trace frames.
- Provides a visualizer for trajectory, object hitboxes, scrubber, and input/velocity bars.

## Support Status And Limitations

High-level status is tracked in `support_matrix.json`.

- **Working today**
  - 240 Hz simulation loop.
  - Core mode/portal/pad/orb/collision pipelines.
  - End-trigger completion detection.
  - One-player dual portal behavior.
  - Visualizer with camera controls and hitbox overlays.
- **Known limitations**
  - Many mechanics are still partial parity (mode edge-cases, slope nuances, trigger interactions).
  - Platformer mode is unsupported.
  - Force blocks are unsupported.
  - Two-player split controls are unsupported.
  - Trigger families that mutate level state (move/toggle/rotate/spawn/collision/etc.) are not fully simulated.

For exact per-mechanic state (`supported` / `partial` / `unsupported` / `no_op`), read `support_matrix.json`.

## Repository Layout

- `src/lib.rs`: crate exports.
- `src/main.rs`: CLI, output/log wiring, and visualizer.
- `src/sim/mod.rs`: public simulation API + orchestration + tests.
- `src/sim/`: simulation internals (included by `mod.rs`):
  - `support_checks.rs`
  - `mode_physics.rs`
  - `portals.rs`
  - `pads.rs`
  - `orbs.rs`
  - `ground_bounds.rs`
  - `collision_resolution.rs`
  - `geometry_helpers.rs`
- `examples/`: utility examples and assets.
- `bitstring/`: click-capture tools that produce `SP240BIN` files for `--clicks-bin`.
- `tests/real_port_contract.rs`: integration contract tests.
- `support_matrix.json`: mechanic support/limitations document.

## Build And Run

From `gd-real-sim/`:

```bash
cargo fmt
cargo check
cargo test -q
```

Show CLI help:

```bash
cargo run -- --help
```

## Bitstring Folder

`bitstring/` contains two capture paths:

- `get_bitstring.cpp` + `get_bitstring.exe`: process-attached capture from `GeometryDash.exe`; starts when player X movement begins.
- `get_bitstring.py`: audio-triggered capture via speaker loopback (or manual countdown mode).
- `requirements.txt`: Python deps for `get_bitstring.py`.

Both output the same binary format: `SP240BIN` (used by `--clicks-bin`).

## Bitstring Commands

You're gonna need to use cheat engine or some other memory scanning tool to at least get the address of the player x coordinate. You can also use the .py that captures audio to start the click pattern tracking as well, but then you may (likely will) have to play around with --tick-offset for gd-real-sim to align the sim and the click pattern.

Build C++ capture tool (if you need to rebuild the exe):

```bash
cl /std:c++17 /EHsc /O2 ^
  bitstring\get_bitstring.cpp DashBot-3.0\HAPIH.cpp ^
  /Fe:bitstring\get_bitstring.exe ^
  /I DashBot-3.0
```

Record with the C++ capture tool (default 240 Hz, 60s):

```bash
bitstring\get_bitstring.exe examples\opti\opti_click_pattern.bin
```

Record with custom duration/hz and trigger tuning:

```bash
bitstring\get_bitstring.exe examples\opti\opti_click_pattern.bin ^
  --duration 90 ^
  --hz 240 ^
  --x-trigger-max 500 ^
  --min-step 0.5 ^
  --max-step 7.0
```

Record with direct memory addresses (if pointer chain drifts):

```bash
bitstring\get_bitstring.exe examples\opti\opti_click_pattern.bin ^
  --x-addr 0x1C3545D688C ^
  --y-addr 0x1C328E5F3E0
```

Python recorder setup:

```bash
pip install -r bitstring/requirements.txt
```

Python recorder (audio-triggered):

```bash
python bitstring/get_bitstring.py --output examples/opti/opti_click_pattern.bin --duration 90
```

Python recorder without audio trigger:

```bash
python bitstring/get_bitstring.py --output examples/opti/opti_click_pattern.bin --duration 90 --no-audio-trigger
```

## Simulator + Visualizer Commands

Simulate with a local level save + named level + click bin:

```bash
cargo run -- \
  --save "/path/to/CCLocalLevels.dat" \
  --level "My Level Name" \
  --clicks-bin "/path/to/clicks.bin"
```

List levels inside a save file:

```bash
cargo run -- --save "/path/to/CCLocalLevels.dat" --list-levels
```

Simulate with inline levelstring + inline click bits:

```bash
cargo run -- \
  --levelstring "..." \
  --clicks "001100101..."
```

Write compressed trace JSON and per-tick text log:

```bash
cargo run -- \
  --save "/path/to/CCLocalLevels.dat" \
  --level "My Level Name" \
  --clicks-bin "/path/to/clicks.bin" \
  --trace-out "trace.json.gz" \
  --tick-log-out "ticks.log"
```

Visualize a level with click tape:

```bash
cargo run -- \
  --save "/path/to/CCLocalLevels.dat" \
  --level "My Level Name" \
  --clicks-bin "/path/to/clicks.bin" \
  --visualize
```

Visualize with tick offset (alias: `--tick-offset`) and log output:

```bash
cargo run -- \
  --save "/path/to/CCLocalLevels.dat" \
  --level "My Level Name" \
  --clicks-bin "/path/to/clicks.bin" \
  --start-tick-offset -10 \
  --tick-log-out "ticks.log" \
  --visualize
```

Visualize when the source click tape was captured at non-240 Hz:

```bash
cargo run -- \
  --save "/path/to/CCLocalLevels.dat" \
  --level "My Level Name" \
  --clicks-bin "/path/to/clicks.bin" \
  --clicks-bin-hz 60 \
  --visualize
```

Visualize with max tick cap and unsupported-policy report mode:

```bash
cargo run -- \
  --save "/path/to/CCLocalLevels.dat" \
  --level "My Level Name" \
  --clicks-bin "/path/to/clicks.bin" \
  --max-ticks 12000 \
  --unsupported-policy report \
  --visualize
```

## Visualizer Controls

Enable visualizer with `--visualize`. Controls:

- `Left/Right` or `A/D`: horizontal scroll
- `+/-` or mouse wheel: zoom
- `F`: toggle follow-cube
- `Home/End`: jump to first/last tick
- Top-right S: settings button, open to toggle
  - M: Toggle non-rotated hitbox visibility (hitbox that interacts with non-rotated objects)
  - R: Toggle rotated hitbox visibility (hitbox that interacts with rotated objects)
  - I: Toggle inner hitbox visibility (hitbox that determines fatal ceiling hits + side hits)
  - T: Trace toggled hitboxes over each game tick
  - C: Toggle click pattern through time viewer (top bar at bottom)
  - V: Toggle y-velocity through time viewer (bottom bar at bottom)
- Drag top: seek by tick
- Right-mouse drag: pan camera
- `Esc`: close

## `examples/opti` Usage

`examples/opti/` contains:

- `opti_objects.txt`: object slice reference for the Opti setup.
- `opti_click_pattern.bin`: click recording used for replay.
- `run_visualizer.sh`: convenience runner.

Run:

```bash
bash examples/opti/run_visualizer.sh "/path/to/CCLocalLevels.dat"
```

Optional second arg overrides tick-log output path.
