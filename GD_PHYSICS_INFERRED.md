# Inferred Geometry Dash Physics from `gd-real-sim`

This document captures all gameplay physics that can be inferred directly from the current `gd-real-sim` implementation.

- Primary sources: `src/sim.rs`, `src/collision.rs`, `src/clock.rs`, `src/level.rs`, `src/input.rs`, `tests/real_port_contract.rs`, `support_matrix.json`.
- Interpretation rule: behavior is documented as implemented, even where comments indicate future parity work.

## Simulation Clock and Integration

- Fixed tick: `DT = 1.0 / 240.0`.
- Horizontal integration per tick: `x += vx * DT * 60 * player_speed`.
- Vertical integration per tick: `y += vy * DT * 60 * VERTICAL_SLOW`.
- `VERTICAL_SLOW = 0.9`.
- Default run length: `max_ticks = 240 * 60 * 10`.
- Click tape is one bit per physics tick (`0` not held, `1` held), sampled by `ClickTape::is_pressed(tick)`.

## Core Player State Model

The simulator tracks:

- Position and velocity: `x`, `y`, `vx`, `vy`.
- Mode: `Cube`, `Ship`, `Ball`, `Ufo`, `Wave`, `Robot`, `Spider`, `Swing`.
- Gravity orientation: `gravity_sign` (`-1` normal/down, `+1` flipped/up).
- Size/speed profile: `mini`, `vehicle_size`, `player_speed`, `speed_multiplier`, `gravity`, `y_start`.
- Contact/state flags: `on_ground`, `was_jump_buffered`, `jump_buffered`, `state_ring_jump`, `on_slope`, `is_accelerating`.
- Slope state: `slope_exit_vy`, `slope_exit_vx`, `slope_contact_cooldown`, `slope_object`, `slope_is_current_top`, `slope_prev_radius`, `rotation`.
- Snap state: `snapped_object`, `snap_distance`.

Derived helpers:

- `flip_mod = -gravity_sign`.
- Player half-size: `15.0` normal, `7.5` mini.

Initial spawn state:

- `x = 0`, `y = 105`, `mode = Cube`, `gravity_sign = -1`, `player_speed = 0.9`, `on_ground = true`.

## Speed Profiles

`SpeedProfile::for_player_speed(speed)` sets `y_start`, `gravity`, and `speed_multiplier`.

- `speed > 1.0`: `y_start=11.420032`, `gravity=0.957199`, `speed_multiplier=5.870002`.
- `speed > 0.85`: `y_start=11.1800318`, `gravity=0.958199024`, `speed_multiplier=5.77000189`.
- `speed == 0.7`: `y_start=10.620032`, `gravity=0.940199`, `speed_multiplier=5.980002`.
- `speed == 1.3 || speed == 1.6`: `y_start=11.230032`, `gravity=0.961199`, `speed_multiplier=6.000002`.
- Else fallback: same as normal (`0.9`) profile.

## Mode Physics

## Cube

- Ground + hold jump: `vy = y_start * jump_scale * flip_mod`, where `jump_scale = 1.0` (normal) or `0.8` (mini).
- While still on slope, jump can receive `+ slope_exit_vy * 0.25`, capped to `1.4x` base jump along gravity-opposed direction.
- Grounded and not jumping: `vy = 0`; `is_accelerating` cleared.
- Airborne gravity update:
  - `vy += gravity * -flip_mod * SUBSTEP_TO_FRAME * VERTICAL_SLOW`.
  - Clamp with gravity orientation (`>= -15` in normal gravity, `<= 15` in flipped gravity).

## Ship

- Uses `used_gravity = 0.9582`.
- Flyer player-size scalar: `1.0` normal, `0.85` mini.
- Complex GDP-style hold/release branch (`v51`, `v52`, `m_isAccelerating` parity shape).
- Current implementation uses `falling_bugged = false` (`v52 = 0.4` path).
- Non-accelerating flyer clamp:
  - Normal gravity: `vy` clamped to `[-6.4/size, 8.0/size]`.
  - Flipped gravity: mirrored variant with top `6.4/size`.

## Ball

- Ground press start flips gravity and multiplies `vy *= 0.6`.
- Otherwise: `vy += 0.9582 * -flip_mod * SUBSTEP_TO_FRAME`.

## Ufo

- Press start: `vy = 7.0 * flip_mod` (normal size) or `8.0 * flip_mod` (mini).
- Then gravity integration as flyer-style constant term: `+ 0.9582 * -flip_mod * SUBSTEP_TO_FRAME`.

## Wave

- Continuous override: `vy = speed_multiplier * factor * sign * flip_mod`.
- `factor = 1.0` normal or `2.0` mini.
- `sign = +1` while held, `-1` when released.

## Robot

- Ground press start: `vy = y_start * 0.5 * flip_mod`.
- While held and already moving against gravity: extra `+ gravity * 0.5 * flip_mod * SUBSTEP_TO_FRAME`.
- Otherwise standard gravity: `+ gravity * -flip_mod * SUBSTEP_TO_FRAME`.

## Spider

- Ground press start: flip gravity, set `vy = 0`, leave ground.
- Otherwise standard gravity branch: `+ gravity * -flip_mod * SUBSTEP_TO_FRAME`.

## Swing

- `accel = gravity * 0.4 * SUBSTEP_TO_FRAME`.
- Held: `vy += accel * flip_mod`.
- Released: `vy -= accel * flip_mod`.
- Additional clamp: `vy` in `[-8/size_scale, +8/size_scale]`, `size_scale=1.0` (normal) or `0.8` (mini-size path).

## Bounds and Death Conditions

- Implicit floor top: `y = 90`.
  - Normal cube (half 15) rests at center `y = 105`.
- Implicit ceiling death line: `y >= 2505`.
- Out-of-bounds floor death: `y < -1200`.
- Ceiling death can be blocked for cube when intersecting h-square object `1859`.

## Collision Model

Collision resolves in two passes:

1. Slopes.
2. Solids/hazards.

General features:

- Nearby-object broad phase: only objects with `abs(object.x - player.x) < 120`.
- Base overlap is AABB (`intersects_box_player`) with half-size chosen by context.
- Snap threshold by mode:
  - Flyers (`Ship`, `Ufo`, `Wave`, `Swing`): `6`.
  - Mini non-flyers: `5`.
  - Default: `10`.

### Solids

- Uses MTV-style axis choice:
  - If vertical overlap is shallower: land/head logic.
  - Else: stair-snap attempt or side-hit death.
- Landing sets `on_ground = true`, `vy = 0`.
- Head bonk may kill cube using inner lethal hitbox (see below).
- Horizontal contact can stair-snap when top surface is within reach and state allows it (descending/on ground/recent slope context).

### Hazard Lethality

- Non-circular hazards: AABB overlap against player outer half-size.
- Circular hazards: closest point from player outer square to circle center against radius.
- Cube inner lethal half-size constant: `3.75` (`OPENGD_CUBE_INNER_HALF`) for side/ceiling lethal checks on solids.
- Non-cube side lethality uses `player_half * 0.3`.

## Slopes

Slope behavior is one of the most developed areas in the sim.

- Slope objects come from `HitboxData::Slope`.
- Entry selection uses non-rotated outer hitbox overlap and additional first-contact "exit-rect" gate.
- Contact uses transformed local-to-world geometry and a near-hypotenuse check.
- Radius on slope uses GDP-style intrinsic-angle form: `player_half / cos(local_slope_angle)`.
- Slope orientation (`slope_floor_top`) is inferred from transformed triangle geometry.
- Supports chained slope transitions with cooldown window.

Key constants/behaviors:

- `slope_contact_cooldown = 48` ticks while on slope (~0.2s at 240 Hz).
- Uphill tolerance helper (`float_g`) contributes to attach threshold (`4.0` when uphill and was on slope, `1.0` on entry uphill, else `0.0`).
- Cube slope rotation interpolation:
  - Toward slope target: `SLOPE_LERP = 0.4`.
  - Return to flat on ground: `RETURN_LERP = 0.25`.
  - Snap-to-zero deadzone: `abs(rotation) < 0.05`.

Slope exit velocity:

- Computed from transformed slope grade and world rect aspect.
- Multiplier: `min(1.12 / slope_angle, 1.54)`.
- Base term scales by `rect_height * player_speed * speed_multiplier / rect_width`.
- Additional sign uses gravity and uphill/downhill direction.
- For `Ship/Ufo/Wave/Ball`, slope velocity is scaled by `0.75`.
- On detach, if slope-exit `vy` is stronger along gravity-opposed direction than current `vy`, it is applied once, then cleared.

## Portal Physics

Portals are touch-activated and usually one-shot per player/object pair.

Mode portals (`mode_for_portal`):

- `12` Cube
- `13` Ship
- `47` Ball
- `111` Ufo
- `660` Wave
- `745` Robot
- `1331` Spider
- `1933`, `2862` Swing

Special mode change behavior:

- Entering Ship from non-Ship halves carried `vy` and clears ground contact.

Speed portals (`player_speed_for_portal`):

- `200 -> 0.7`
- `201 -> 0.9`
- `202 -> 1.1`
- `203 -> 1.3`
- `1334 -> 1.6`

When triggered, speed portal reassigns `player_speed`, `speed_multiplier`, `gravity`, `y_start`, and `vx`.

Gravity portals:

- `10` sets normal gravity (`-1`), `11` sets flipped gravity (`+1`).
- If actual gravity flip occurs, `vy` is halved.

Size portals:

- `101` mini (`mini=true`, `vehicle_size=0.6`).
- `99` normal (`mini=false`, `vehicle_size=1.0`).

Mirror portals:

- No-op for physics.

Dual portals:

- `286` activates dual partner.
- `287` deactivates partner.
- Partner shares same input and is stepped through same physics loop with mirrored gravity sign.

Teleport portals:

- Entry `747` teleports to nearest `749` in X.
- Current behavior changes `y` only; gravity and velocity are preserved.

## Pads

Activation:

- Checks pad-specific OpenGD-like activation rects (`opengd_pad_activation_rect`) when available.
- Blue pads can activate pre-collision.
- At most one pad impulse is applied per tick in main pad pass.

Pad force direction:

- Applied on gravity axis (`vy = magnitude` in current implementation).
- Rotation affects activation/facing logic, not impulse direction.

Pad magnitudes (`G = flip_mod`):

- Yellow (`35`):
  - Cube/Robot/Ship/Ufo: `16G`
  - Ball/Spider/Swing: `9.6G`
  - Wave: `0`
- Purple (`140`):
  - Cube/Robot: `12G`
  - Ship: `5.6G`
  - Ufo: `6.4G`
  - Ball/Spider: `6.72G`
  - Swing: `6.24G`
  - Wave: `0`
- Red (`1332`):
  - Cube/Robot: `20G`
  - Ship: `10.08G`
  - Ufo: `9.6G`
  - Ball/Spider/Swing: `12G`
  - Wave: `0`
- Blue gravity (`67`):
  - Cube/Robot/Ship/Ufo: `6.4G` (before flip)
  - Ball/Spider/Swing: `3.84G` (before flip)
  - Then flips gravity.

Gravity pad facing gate:

- Uses `rotation` and signed `scale_y` to infer world-facing direction.

## Orbs

Activation:

- Fires on press-start OR queued hold (`state_ring_jump`) while overlapping orb.
- Consumes orb once per player/object.

Orb behaviors:

- Yellow (`36`):
  - Cube uses `y_start * flip_mod`.
  - Ball/Spider `0.7x` cube yellow base.
  - Robot `0.9x`.
  - Swing `0.6x`.
  - Ship/Ufo `8G`.
- Pink (`141`):
  - Cube/Robot `12G`.
  - Other modes use current proportional factors.
- Red (`1333`):
  - Cube/Robot `1.38x` yellow-cube base.
  - Ship `1.0x` yellow-cube base.
  - Other mode-specific factors.
- Gravity jump (`84`, `1022`):
  - Flip gravity first, then yellow-orb impulse.
- Green (`1594`):
  - Applies mode-specific inverse pulse, then flips gravity.
- Black (`1330`):
  - Cube/Ball/Robot/Spider: `-15G`.
  - Ship/Swing: `-14G`.
  - Ufo: `-11.2G`.
- Dash/spider/toggle special IDs:
  - `1704` dash orb: currently minimal/partial behavior (no dedicated dash state machine).
  - `1751` spider orb: flips gravity and zeroes `vy`.
  - `3004` toggle orb: currently physics no-op.

Orb and pad boosts set `is_accelerating = true`.

## Snap-Jump / Stair Thresholds

`snap_jump_thresholds(player_speed, vehicle_size)`:

- Speed `0.9`: `(1.0, 120 or 90 mini, 150, 90)`
- Speed `0.7`: `(1.0, 90, 120, 60)`
- Speed `1.1`: `(2.0, 150 or 90 mini, 195, 120)`
- Speed `1.3`: `(2.0, 90, 225, 135)`
- Else:
  - big: `(2.0, 180, 225, 135)`
  - mini: `(1.0, 120, 150, 90)`

This logic is used by `check_snap_jump_to_object`.

## Level Coordinate and Transform Rules

- Raw level object Y (`key 3`) is loaded as `y + 90.0`.
- Rotation uses negated level angle (`rotation = -key6`) to match engine transform direction.
- Box hitboxes use transformed corner bounds (`opengd_box_transform`) from local hitbox data + signed scales + rotation.
- Circle center uses object center convention.

## Trigger and Unsupported Policy (As Implemented)

- Platformer (`kA17 == 1`) hard errors as unsupported.
- Many trigger IDs are accepted as no-op with warning (rather than hard error), except supported set used by simulator logic.
- End trigger (`3607`) is functionally supported for completion.
- Visual-only triggers can be accepted without gameplay effect.

## Completion Detection

Run completes if either:

- Player crosses object `3607`, or
- Player crosses `last_gameplay_block_x + 60`.

`last_gameplay_block_x` considers `Solid`, `Slope`, and `Hazard` objects.

## Known Partial/Unsupported Areas

Based on `support_matrix.json` plus code comments:

- Most modes are marked partial.
- Timewarp remainder behavior unsupported.
- Two-player split control unsupported (dual currently one-input model).
- Many triggers unsupported or visual-only partial.
- Slope, circle hitboxes, and rotation-aware hitbox parity are partial.
- Breakables and moving platform velocity inheritance unsupported.
- Dash/spider/toggle orb full state machines not fully modeled.
- Teleport portal pairing and portal hysteresis/priority remain partial.

## Notes on Canon vs Current Behavior

- This file documents actual current simulator behavior, not an idealized GDP/OpenGD target.
- Several comments in code describe intended parity still in progress; where relevant, tests and implementation currently take precedence in this summary.
