# gd-real-sim handoff

## What's working end-to-end

- `CCLocalLevels.dat` ingestion (Windows XOR/base64/zlib) + `--levelstring`/`--levelstring-file`
- 240 Hz physics tick with `gdp` speed/gravity constants per speed tier
- Cube/ship/ball/ufo/wave/robot/spider/swing physics (partial; see below)
- Mode, gravity, size, and speed portals (touch-gated, once per entry)
- Yellow, pink, red, and blue (gravity) pads with `gdp`-style impulses
- Solid collision with `snap_up_threshold` (10 / 5 / 6 from `gdp`)
- Hazard collision with the `0.3×` lethal hitbox check from `gdp::collidedWithObjectInternal`
- Slope helper (`slope_y_pos`) faithfully ported from `GameObject::slopeYPos`
- End trigger (id 3607) detection via X-sweep
- CLI JSON output + gzipped per-tick trace
- Explicit `UnsupportedFeature` errors for platformer, mirror/dual/teleport portals, orbs, and every non-visual trigger

Run: `cargo test` → 18 green.

## Where we deliberately stopped (and what to port next)

| Area | Current state | What's missing | Reference |
|---|---|---|---|
| Robot variable-height jump | Fixed impulse + held-rising acceleration | Proper jump-cut frame count from `gdp` | `gdp/PlayerObject_updateJump.cpp` lines ~270–350 |
| Ball rotation | Flip only | Ball-rotation feel from `PlayerObject_runBallRotation.cpp` | same |
| Spider teleport | Gravity flip only | Instant teleport to nearest solid (requires block raycast) | `PlayerObject_updateJump.cpp` spider branch |
| Stair snap | Not implemented | `checkSnapJumpToObject` with speed-tier stair-size tables | `PlayerObject_checkSnapJumpToObject.cpp` |
| Moving platforms | Not implemented | Velocity inheritance from `gdp::collidedWithObjectInternal` "moving platform block" section | same |
| Slopes in sim | Treated as uphill-right only | Slope direction/rotation, hazard slopes, slope velocity carry | `PlayerObject_collidedWithSlopeInternal.cpp` |
| Orbs | All unsupported | Yellow/pink/red/blue/green/black orbs with `gdp::boostPlayer` constants | `PlayerObject_boostPlayer.cpp` |
| Triggers with sim effect | Only end trigger | Move, toggle, spawn, gravity, timewarp triggers via gdclone X-sweep | `gdclone/src/level/trigger/*.rs` |
| Dual mode | Unsupported (detected) | Second player entity + mirrored input | not in current `gdp/` snippets |
| Timewarp | Unsupported | `m_timewarp` / `m_deltaRemainder` from `GJBaseGameLayer::update` | `GJBaseGameLayer_update.cpp` |

## What additional resources would most unblock progress

**In priority order:**

1. **The `gdp/PlayerObject/` files I haven't needed yet are probably sufficient for the remaining player work** (`PlayerObject_runBallRotation.cpp`, `PlayerObject_updateSlopeRotation.cpp`, `PlayerObject_updateShipRotation.cpp` are already there). Nothing else needed for single-player modes.
2. **Trigger/effect manager decomp** — `EffectGameObject::triggerObject`, `GJEffectManager::prepareMoveActions`, `GJEffectManager::processMoveActionsStep`, `GJEffectManager::preCollisionCheck`. Without these, move/toggle/spawn triggers stay unsupported. gdclone has the trigger *dispatcher* but its X-sweep semantics need to be reconciled with GD's actual per-tick execution order.
3. **Orb decomp** — `PlayerObject::ringJump` or equivalent. Without this orbs just error. The `gdp` files don't currently include it, and hand-inferring the impulses from community wiki is risky.
4. **Optional: a short recorded trace** (levelstring + click bitstring + GD-actual outcome) for *one* 2.2 level. Right now I can prove internal self-consistency with `gdp`+`gdclone` but not that my timestep integration lines up with real GD frame-for-frame. A single verified sample would catch accumulated error.

## Known parity risks even in "completed" areas

- I apply gravity per-tick at `DT * 60` scale, then use `DT * 60 * player_speed * HORIZONTAL_SLOW` for X. This matches gdclone's `Δx = vx * dt * speed` but I haven't bit-for-bit verified that `gdp`'s `physicsSecond = delta * 60 / stepCount` produces the same X per step at 60 FPS input. Minor drift is possible.
- The spider/robot press-start detection uses "pressed this tick AND !was_jump_buffered". `gdp` uses `m_jumpBuffered` combined with `m_wasJumpBuffered` via the `gameEventTriggered(21, …) / (22, …)` path, which also involves edge cases for `m_stateRingJump`. Most levels won't notice.
- Speed portal semantics apply once on first intersection. Real GD uses touch-triggered portals with X-priority ordering. If two speed portals overlap spatially, behavior may differ from GD.
- Mini size is hardcoded to `vehicle_size = 0.6` and `player_half = 7.5`. GD uses `0.6` for cube/ball and different scales per vehicle (`0.85` for mini ship per `gdp::updateJump` line ~130).

## Files I'd recommend reading if you pick this up

- `src/sim.rs` — core simulation loop, physics dispatch, collision, portals/pads.
- `src/level.rs` — object ID classification table (this is the place to extend hazard/orb/trigger IDs).
- `src/clock.rs` — `SpeedProfile::for_player_speed` and `physics_steps`; any new speed tier goes here.
- `src/collision.rs` — `slope_y_pos`; extend here for slope-direction branching.
- `support_matrix.json` — single source of truth for what's declared supported vs partial vs unsupported.
- `tests/real_port_contract.rs` — contract tests; every new physics behavior should land with a test here.

## How to verify

```
cargo test
cargo run -- --levelstring "..." --clicks "0101..."
cargo run -- --save path/to/CCLocalLevels.dat --level "Level Name" --clicks-file path/to/clicks.txt --trace-out trace.json.gz
```
