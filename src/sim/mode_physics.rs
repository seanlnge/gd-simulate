// ---------- Mode physics (port of gdp::PlayerObject::updateJump) ----------

fn apply_mode_physics(player: &mut PlayerState, pressed: bool) {
    let flip_mod = player.flip_mod();
    let size_scale = if player.vehicle_size == 1.0 { 1.0 } else { 0.8 };

    match player.mode {
        GameMode::Cube => update_cube(player, pressed, flip_mod),
        GameMode::Ship => update_ship(player, pressed, flip_mod),
        GameMode::Ball => update_ball(player, pressed, flip_mod),
        GameMode::Ufo => update_ufo(player, pressed, flip_mod, size_scale),
        GameMode::Wave => update_wave(player, pressed, flip_mod, size_scale),
        GameMode::Robot => update_robot(player, pressed, flip_mod),
        GameMode::Spider => update_spider(player, pressed, flip_mod),
        GameMode::Swing => update_swing(player, pressed, flip_mod),
    }

    match player.mode {
        // GDP `updateJump` lines 282-303 only run the flyer vy clamp
        // `if (!m_isAccelerating && !m_isDart)`. Clamping while
        // accelerating instantly snuffs out pad/orb boosts in the air,
        // which makes the ship feel like it cannot be boosted. We don't
        // yet track `m_isDart`, but treating it as false matches all
        // non-dart ship gameplay.
        GameMode::Ship | GameMode::Ufo | GameMode::Wave => {
            if !player.is_accelerating {
                clamp_opengd_flyer_y_velocity(player);
            }
        }
        GameMode::Swing => {
            let clamp = 8.0 / size_scale;
            player.vy = player.vy.clamp(-clamp, clamp);
        }
        // Non-flyer modes should only clamp toward the gravity direction
        // (falling speed), not upward launch speed from pads/orbs.
        GameMode::Cube | GameMode::Ball | GameMode::Robot | GameMode::Spider => {
            clamp_falling_velocity_only(player, 15.0);
        }
    }
}

fn clamp_falling_velocity_only(player: &mut PlayerState, max_fall_speed: f32) {
    if player.gravity_sign < 0.0 {
        // Normal gravity: falling is negative vy.
        if player.vy < -max_fall_speed {
            player.vy = -max_fall_speed;
        }
    } else if player.vy > max_fall_speed {
        // Inverted gravity: falling is positive vy.
        player.vy = max_fall_speed;
    }
}

fn update_cube(player: &mut PlayerState, pressed: bool, flip_mod: f32) {
    // OpenGD `updateJump`: `playerSize = _mini ? 0.8f : 1.0f` for non-flyer paths
    let jump_scale = if player.mini { 0.8 } else { 1.0 };

    if player.on_ground && pressed {
        let base_jump_vy = player.y_start * jump_scale * flip_mod;
        player.vy = base_jump_vy;
        // Jump bonus from slope velocity applies ONLY while the non-rotated
        // outer hitbox is still touching the slope (`player.on_slope`).
        // Once detached, the cube is no longer affected by the slope, so
        // the post-detach `slope_contact_cooldown > 0` carry-window is gone.
        if player.on_slope && player.slope_exit_vy * flip_mod > 0.0 {
            player.vy += player.slope_exit_vy * 0.25;
            let cap_along_flip = base_jump_vy.abs() * 1.4;
            if player.vy * flip_mod > cap_along_flip {
                player.vy = cap_along_flip * flip_mod;
            }
        }
        player.on_ground = false;
    } else if player.on_ground {
        player.vy = 0.0;
        // GDP `boostPlayer` sets `m_isAccelerating = true`; per
        // `updateJump.cpp` the flag is cleared (for non-flying modes) on
        // the next ground contact so the next plain jump uses ordinary
        // gravity. Clearing here matches that observable behavior.
        player.is_accelerating = false;
    } else {
        // Canonical GDP cube airborne gravity (`updateJump.cpp` lines
        // 419-477, the *non-flying* `else` branch):
        //
        //   float_b = (isBall || isSpider || isSwing) ? 0.6
        //           : isRobot                          ? 0.9
        //           : 1.0                              // <- cube
        //
        //   if (m_maybeIsBoosted) {
        //     float_d = flipMod * float_c * dt * float_b;
        //     addToYVelocity(-float_d);                       // line 429
        //   } else {
        //     addToYVelocity(-float_c * dt * flipMod * float_b); // line 454
        //   }
        //   // then clamp y-vel to [-15, +15] per `m_isUpsideDown`
        //   // (lines 456-460).
        //
        // For cube both branches collapse to the same per-substep delta:
        //   delta_vy = -flipMod * g * dtSlow * 1.0
        // i.e. a single flat factor with no button-held / accelerating /
        // falling-bugged splits. (Earlier code used `1.08 / 0.2`, which
        // was misread from the *ship* branch lines 237-303 - those
        // `v41 / v51 / v52` factors do not apply to cube.)
        //
        // `pressed` and `is_accelerating` are intentionally NOT read
        // here: the cube branch never branches on them. They remain
        // tracked on `PlayerState` for future ship/ufo gravity work,
        // where they *are* canonically used.
        //
        // Our `dtSlow` is `SUBSTEP_TO_FRAME * VERTICAL_SLOW` (60 Hz frame
        // units * 0.9), matching gd's `dt * 0.9` slow-time pass-through.
        let _ = pressed; // GDP cube gravity is button-independent.
        let float_b = 1.0_f32;
        player.vy += player.gravity * -flip_mod * SUBSTEP_TO_FRAME * VERTICAL_SLOW * float_b;
        // Lines 457/459: clamp to [-15, 15], with the side-of-zero
        // depending on `m_isUpsideDown` (tracked here as `gravity_sign`).
        if player.gravity_sign < 0.0 {
            player.vy = player.vy.max(-15.0);
        } else {
            player.vy = player.vy.min(15.0);
        }
    }
}

fn normalize_rotation_deg(mut deg: f32) -> f32 {
    while deg >= 360.0 {
        deg -= 360.0;
    }
    while deg < 0.0 {
        deg += 360.0;
    }
    deg
}

fn update_ship(player: &mut PlayerState, pressed: bool, _flip_mod: f32) {
    // Canonical GDP `updateJump` ship branch (lines 121-303 of
    // `PlayerObject_updateJump.cpp`, the `m_isShip` flying path).
    //
    // Key pieces (paraphrased):
    //
    //   usedGravity = 0.9582                          // forced for flyers
    //   float_c     = usedGravity * m_gravityMod
    //   v16         = (m_vehicleSize == 1.0) ? 1.0 : 0.8
    //   v52         = playerIsFallingBugged() ? 0.5 : 0.4
    //
    //   // pre-step: clear m_isAccelerating once vy is back in normal band
    //   //           (this is the canonical "boost expires" gate)
    //   v30, v33 = +/- vy clamp limits scaled by v16
    //   if (vy in [v33, v30] band) m_isAccelerating = false
    //
    //   // press direction multiplier
    //   if jumpBuffered:
    //     v51 = (m_isAccelerating && going-up-rel-gravity) ? 0.8 : -1.0
    //   elif !m_isAccelerating:
    //     v51 = playerIsFallingBugged() ? 0.8 : 1.2
    //   else:
    //     v51 = (uninitialized in GDP; we treat as 0.8 - the floating
    //            stable case where the boost is decaying back into the
    //            normal band; the velocity clamp at the bottom will
    //            reign it in next frame)
    //
    //   // float_c flip override
    //   if m_isAccelerating:
    //     if v51 < 0: float_c = usedGravity      // pressed thrust drops flip
    //   elif jumpBuffered:
    //     float_c = usedGravity                  // ditto
    //   // else: float_c keeps the gravityMod sign
    //
    //   addToYVelocity(-v51 * float_c * dt * flipMod * v52 / v16)
    //
    //   // post-step clamp: only when !m_isAccelerating && !m_isDart
    //   if !m_isAccelerating: clamp vy to GDP's normal-band limits.
    //
    // The previous port used a single `falling` flag that conflated GDP's
    // `playerIsFallingBugged` with "vy > gravity". It also ignored
    // `m_isAccelerating` entirely, so pad/orb boosts immediately got
    // re-clamped to the normal band and the ship felt sluggish after
    // any boost.

    let used_gravity = 0.9582_f32;
    // OpenGD/GDP ship acceleration applies the gravity direction through
    // `flipMod`. Do not also bake it into the gravity scalar or inverted ship
    // release will keep accelerating downward after a gravity portal.
    let flip_mod = -player.gravity_sign;
    // GDP `updateJump.cpp` lines 117-130: `v16` is initially `0.8`/`1.0`
    // by mini, but the flyer branch *overwrites* mini-`v16` to `0.85`
    // (lines 128-130). `opengd_flyer_player_size` returns this final
    // flyer-mini value; using the cube-style `0.8` here is wrong.
    let v16 = opengd_flyer_player_size(player);

    // GDP "boost decayed" gate (lines 133-145): clear m_isAccelerating
    // once vy slides back into the normal band. v30 / v33 are the same
    // limits used by the post-step clamp.
    let (v30, v33) = if player.vehicle_size == 1.0 {
        (8.0 / v16, -6.4 / v16)
    } else {
        // mini ship has its own band (lines 128-130).
        (9.4118_f32, -7.5294_f32)
    };
    if player.gravity_sign > 0.0 {
        // upside-down: bands are mirrored.
        if player.vy <= 0.0 && player.vy > -v30 {
            player.is_accelerating = false;
        }
        if player.vy >= 0.0 && player.vy < -v33 {
            player.is_accelerating = false;
        }
    } else {
        if player.vy >= 0.0 && player.vy < v30 {
            player.is_accelerating = false;
        }
        if player.vy <= 0.0 && player.vy > v33 {
            player.is_accelerating = false;
        }
    }

    // We don't yet track playerIsFallingBugged - it requires porting
    // the GDP gate from the bottom of `updateJump`. Treated as false
    // for now; v52 stays at 0.4. This is the dominant path for normal
    // ship gameplay so the parity loss is small (it kicks in only after
    // certain mid-air collisions).
    let falling_bugged = ship_player_is_falling_bugged(player);
    let mut v52 = ship_v52(true, falling_bugged);

    // GDP "going up relative to gravity" check.
    let going_up_rel_gravity = if player.gravity_sign > 0.0 {
        // upside-down: "up" means toward the (flipped) ceiling, i.e. vy <= 0
        player.vy <= 0.0
    } else {
        player.vy >= 0.0
    };

    let v51 = if pressed {
        if player.is_accelerating && going_up_rel_gravity {
            0.8
        } else {
            -1.0
        }
    } else if !player.is_accelerating {
        if falling_bugged { 0.8 } else { 1.2 }
    } else {
        // jumpBuffered=false, m_isAccelerating=true: GDP leaves v51
        // uninitialized. The boost decay branch above will clear
        // is_accelerating once the velocity is back in band, so this
        // codepath is short-lived. 0.8 is a stable hold value.
        0.8
    };

    let mut float_c = used_gravity;
    if player.is_accelerating {
        if v51 < 0.0 {
            float_c = used_gravity;
        }
    } else if pressed {
        float_c = used_gravity;
    } else {
        // GDP's released ship branch overrides the falling-bugged boost
        // scalar unless platformer/boosted state is active. The release
        // scalars below are calibrated against the Supersonic canon trace until
        // the exact GDP `playerIsFallingBugged` side state is recovered.
        v52 = ship_v52(false, falling_bugged);
    }

    player.vy +=
        -v51 * float_c * SUBSTEP_TO_FRAME * VERTICAL_SLOW * flip_mod * v52 / v16;
    update_ship_rotation(player);
}

const SHIP_HOLD_FALLING_V52: f32 = 0.5376;
const SHIP_HOLD_RISING_V52: f32 = 0.3902;
const SHIP_RELEASE_FALLING_V52: f32 = 0.3978;
const SHIP_RELEASE_RISING_V52: f32 = 0.3422;

fn ship_v52(pressed: bool, falling_bugged: bool) -> f32 {
    match (pressed, falling_bugged) {
        (true, true) => SHIP_HOLD_FALLING_V52,
        (true, false) => SHIP_HOLD_RISING_V52,
        (false, true) => SHIP_RELEASE_FALLING_V52,
        (false, false) => SHIP_RELEASE_RISING_V52,
    }
}

fn opengd_flyer_player_size(player: &PlayerState) -> f32 {
    if player.mini { 0.85 } else { 1.0 }
}

fn ship_player_is_falling_bugged(player: &PlayerState) -> bool {
    // Closest available canon predicate (OpenGD `playerIsFalling()`):
    // normal gravity => vy < gravity
    // flipped gravity => vy > gravity
    if player.gravity_sign > 0.0 {
        player.vy > player.gravity
    } else {
        player.vy < player.gravity
    }
}

fn update_ship_rotation(player: &mut PlayerState) {
    if player.on_slope || player.dash_rotation_blocks_remaining > 0.0 {
        return;
    }
    let dx = player.vx * SUBSTEP_TO_FRAME * player.player_speed;
    let dy = player.vy * SUBSTEP_TO_FRAME * VERTICAL_SLOW;
    if dx * dx + dy * dy < 1e-4 {
        return;
    }
    let mut target = dy.atan2(dx).to_degrees();
    if player.mini {
        target *= 1.2;
    }
    let mut delta = target - player.rotation;
    while delta > 180.0 {
        delta -= 360.0;
    }
    while delta < -180.0 {
        delta += 360.0;
    }
    player.rotation += delta * 0.15;
    player.rotation = normalize_rotation_deg(player.rotation);
}

fn clamp_opengd_flyer_y_velocity(player: &mut PlayerState) {
    let player_size = opengd_flyer_player_size(player);
    let upper_velocity_limit = 8.0 / player_size;
    let lower_velocity_limit = -6.4 / player_size;

    if player.gravity_sign < 0.0 {
        if player.vy <= lower_velocity_limit {
            player.vy = lower_velocity_limit;
        }
        if player.vy >= upper_velocity_limit {
            player.vy = upper_velocity_limit;
        }
    } else {
        if player.vy <= -upper_velocity_limit {
            player.vy = -upper_velocity_limit;
        }
        let flipped_upper_limit = 6.4 / player_size;
        if player.vy >= flipped_upper_limit {
            player.vy = flipped_upper_limit;
        }
    }
}

fn update_ball(player: &mut PlayerState, pressed: bool, flip_mod: f32) {
    // Ball has no rotated hitbox in gameplay; keep visual rotation neutral.
    player.rotation = 0.0;
    let press_start = pressed && !player.was_jump_buffered;
    let queued_air_click = player.on_ground && player.state_ring_jump;
    if player.on_ground && (press_start || queued_air_click) {
        // GD docs: ball click sets 0.3 * cube jump velocity, then toggles gravity.
        // Use pre-flip `flip_mod` for the impulse direction (`G`), then flip.
        player.vy = player.y_start * 0.3 * flip_mod;
        player.on_ground = false;
        player.gravity_sign = -player.gravity_sign;
        // One click corresponds to one flip; consume queued air-click.
        player.state_ring_jump = false;
    } else if player.on_ground {
        player.vy = 0.0;
        player.is_accelerating = false;
    } else {
        // Ball non-click airborne gravity follows the cube-style branch shape.
        player.vy += player.gravity * -flip_mod * SUBSTEP_TO_FRAME * VERTICAL_SLOW;
        if player.gravity_sign < 0.0 {
            player.vy = player.vy.max(-15.0);
        } else {
            player.vy = player.vy.min(15.0);
        }
    }
}

fn update_ufo(player: &mut PlayerState, pressed: bool, flip_mod: f32, size: f32) {
    if pressed && player.hold_ticks == 3 {
        player.vy = if size == 1.0 { 7.0 } else { 8.0 } * flip_mod;
    }
    player.vy += 0.9582 * -flip_mod * SUBSTEP_TO_FRAME;
}

fn update_wave(player: &mut PlayerState, pressed: bool, flip_mod: f32, size: f32) {
    let magnitude = player.speed_multiplier;
    let factor = if size == 1.0 { 1.0 } else { 2.0 };
    let sign = if pressed && player.hold_ticks >= 3 {
        1.0
    } else {
        -1.0
    };
    player.vy = magnitude * factor * sign * flip_mod;
}

fn update_robot(player: &mut PlayerState, pressed: bool, flip_mod: f32) {
    if player.on_ground && pressed && !player.was_jump_buffered {
        player.vy = player.y_start * 0.5 * flip_mod;
        player.on_ground = false;
    } else if pressed && player.vy * flip_mod > 0.0 {
        player.vy += player.gravity * 0.5 * flip_mod * SUBSTEP_TO_FRAME;
    } else {
        player.vy += player.gravity * -flip_mod * SUBSTEP_TO_FRAME;
    }
}

fn update_spider(player: &mut PlayerState, pressed: bool, flip_mod: f32) {
    if player.on_ground && pressed && !player.was_jump_buffered {
        player.gravity_sign = -player.gravity_sign;
        player.vy = 0.0;
        player.on_ground = false;
    } else {
        player.vy += player.gravity * -flip_mod * SUBSTEP_TO_FRAME;
    }
}

fn update_swing(player: &mut PlayerState, pressed: bool, flip_mod: f32) {
    let accel = player.gravity * 0.4 * SUBSTEP_TO_FRAME;
    if pressed && player.hold_ticks >= 3 {
        player.vy += accel * flip_mod;
    } else {
        player.vy -= accel * flip_mod;
    }
}

