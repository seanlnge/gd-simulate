# Live Visualizer Settings And Attempts Plan

## Goal
Make live play in the native visualizer behave like the normal visualizer for settings, start the camera at world x=0, and surface recent live attempts in the Tauri app with one-click saving to the existing bitstring library.

## Design
- Reuse the existing native `VizState`, `SettingsToggle`, `draw_settings_ui`, and bar rendering paths for live play.
- Change live Escape behavior only: first Escape opens the settings panel, Escape while the panel is open exits the native window. Replay visualizer keeps its current behavior unless shared helper changes are needed.
- Add an explicit Exit control to the native settings panel, represented as a `SettingsToggle::Exit` row for live mode only.
- Keep simulation input gating in `LiveSimulationSession`: clicks start being recorded/processed when x reaches/crosses 0. Camera initialization changes should not change physics timing.
- Persist live attempts as JSON records from the native process into a per-level history file under the visualizer directory. The Tauri app lists the newest 20 attempts and saves an attempt bitstring via the existing `upsert_bitstring` command.

## Checklist
- [x] Add focused Rust tests for live attempt metrics and x=0 progress calculation.
- [x] Add attempt history contracts and Tauri commands to list live attempts.
- [x] Pass a level id/history path when launching live native visualizer from Tauri.
- [x] Record each live attempt's bitstring, percentage, processed click ticks, outcome, and timestamp from the native live loop.
- [x] Reuse native settings UI in live play, including Escape-to-settings and an Exit row.
- [x] Initialize/clamp the live camera so the left side starts at x=0 while preserving first-input gating.
- [x] Update React API/types and `LevelView.tsx` to show the latest 20 attempts with percent, processed clicks, and Save Bitstring actions.
- [x] Verify with `cargo test`, Tauri crate tests/build, frontend build, and lints for edited files.

## Review
- `cargo test` passes in the core crate.
- `cargo test` passes in `visualizer/src-tauri`.
- `npm run build` passes in `visualizer`.
- Edited files have no IDE linter diagnostics.
