# Lessons

## 2026-05-12: Preserve Established CLI Entry Points

- Adding a second Rust binary makes plain `cargo run -- ...` ambiguous unless `default-run` is set.
- When adding helper binaries, either update `Cargo.toml` with `default-run = "gd-real-sim"` or verify existing scripts/manual commands that rely on the package default.

## 2026-05-12: Align Bitstring Timing To Recorder Semantics

- The SP240BIN recorder documents sample 0 as the first moving attempt tick near player x = -1, not the tick where the simulator player's center crosses x = 0.
- Do not gate replay/live click consumption on player-center x >= 0. Camera start and input tape timing are separate concerns.
