// ---------- Orbs ----------

fn apply_orbs(
    level: &Level,
    player: &mut PlayerState,
    touched: &mut HashSet<(usize, u8)>,
    which_player: u8,
    pressed: bool,
) {
    // OpenGD ring jump uses `_queuedHold && m_bIsHolding`: a fresh press fires
    // immediately, and a press that started while airborne can fire when the
    // player enters a ring later in that same hold.
    let press_start = pressed && !player.was_jump_buffered;
    let queued_air_press = pressed && player.state_ring_jump;

    for (idx, object) in level.objects.iter().enumerate() {
        if object.kind != ObjectKind::Orb {
            continue;
        }
        let key = (idx, which_player);
        if touched.contains(&key) {
            continue;
        }
        if !intersects_player(object, *player) {
            continue;
        }

        let is_green_gravity_jump_family = matches!(object.object_id, 1022 | 1594);
        let can_activate = if is_green_gravity_jump_family {
            // Green/gravity-jump orb family should trigger whenever held while touching.
            pressed
        } else {
            press_start || queued_air_press
        };
        if !can_activate {
            continue;
        }

        let activated = match object.object_id {
            36 => {
                apply_yellow_orb(player);
                true
            }
            84 => {
                apply_blue_orb(player);
                true
            }
            141 => {
                apply_pink_orb(player);
                true
            }
            1333 => {
                apply_red_orb(player);
                true
            }
            1022 => {
                apply_green_orb(player);
                true
            }
            1330 => {
                apply_black_orb(player);
                true
            }
            1594 => {
                apply_green_orb(player);
                true
            }
            1704 => apply_green_dash_orb(player, object.rotation),
            1751 => apply_pink_dash_orb(player, object.rotation),
            3004 => true, // toggle orb: physics no-op for now
            _ => false,
        };
        if !activated {
            continue;
        }
        touched.insert(key);
        player.state_ring_jump = false; // consume queued hold on orb fire
        player.on_ground = false;
        // GDP `boostPlayer` (called by all orbs through their respective
        // velocity setters) sets `m_isAccelerating = true`. This persists
        // until ground contact and reduces airborne gravity, giving orbs
        // their characteristic longer arc compared to a plain jump.
        player.is_accelerating = true;
    }
}

/// Orb tables from GD Docs. "Cube yellow orb" = `y_start` (cube jump velocity
/// at current speed tier). "G" = flip_mod.
fn cube_yellow_velocity(player: &PlayerState) -> f32 {
    player.y_start * player.flip_mod()
}

fn apply_yellow_orb(player: &mut PlayerState) {
    let g = player.flip_mod();
    let cube_base = cube_yellow_velocity(player) * mini_cube_pulse_scale(player);
    player.vy = match player.mode {
        GameMode::Cube | GameMode::Ball | GameMode::Robot | GameMode::Spider | GameMode::Swing => {
            match player.mode {
                GameMode::Cube => cube_base,
                GameMode::Ball | GameMode::Spider => cube_base * 0.7,
                GameMode::Robot => cube_base * 0.9,
                GameMode::Swing => cube_base * 0.6,
                _ => cube_base,
            }
        }
        GameMode::Ship | GameMode::Ufo => 8.0 * g,
        GameMode::Wave => player.vy, // docs: no effect
    };
}

fn apply_pink_orb(player: &mut PlayerState) {
    let base = cube_yellow_velocity(player);
    let cube_scale = mini_cube_pulse_scale(player);
    player.vy = match player.mode {
        GameMode::Cube => base * 0.72 * cube_scale,
        GameMode::Robot => base * 0.72,
        GameMode::Ship => base * 0.37,
        GameMode::Ball | GameMode::Spider => base * 0.7 * 0.77,
        GameMode::Ufo => base * 0.42,
        GameMode::Swing => base * 0.6 * 0.72,
        GameMode::Wave => player.vy,
    };
}

fn apply_red_orb(player: &mut PlayerState) {
    let base = cube_yellow_velocity(player);
    let cube_scale = mini_cube_pulse_scale(player);
    player.vy = match player.mode {
        GameMode::Cube => base * 1.38 * cube_scale,
        GameMode::Robot => base * 1.28,
        GameMode::Ship => base, // same as cube yellow
        GameMode::Ball | GameMode::Spider => base * 0.7 * 1.34,
        GameMode::Ufo => base * 1.02,
        GameMode::Swing => base * 0.6 * 1.38,
        GameMode::Wave => player.vy,
    };
}

fn apply_blue_orb(player: &mut PlayerState) {
    let base = cube_yellow_velocity(player);
    let cube_scale = mini_cube_pulse_scale(player);
    player.vy = match player.mode {
        GameMode::Cube => base * 0.4 * cube_scale,
        GameMode::Ship => base * 0.4,
        GameMode::Ball | GameMode::Spider => base * 0.4 * 0.7,
        GameMode::Ufo => base * 0.4,
        GameMode::Wave => player.vy,
        GameMode::Robot => base * 0.9 * 0.4,
        GameMode::Swing => base * 0.6 * 0.4,
    };
    player.gravity_sign = -player.gravity_sign;
}


fn apply_green_orb(player: &mut PlayerState) {
    // Green orb: flip gravity, then apply yellow-orb impulse table in the new gravity.
    player.gravity_sign = -player.gravity_sign;
    apply_yellow_orb(player);
}

fn is_h_block(object: &LevelObject) -> bool {
    object.object_id == 1859
}

fn normalize_signed_deg(deg: f32) -> f32 {
    let mut out = normalize_rotation_deg(deg);
    if out > 180.0 {
        out -= 360.0;
    }
    out
}

fn clamp_dash_angle_deg(rotation_deg: f32) -> f32 {
    let mut theta = normalize_signed_deg(rotation_deg);
    if theta >= 90.0 || theta <= -90.0 {
        theta = normalize_signed_deg(180.0 - theta);
    }
    theta.clamp(-70.0, 70.0)
}

fn start_dash_orb(player: &mut PlayerState, orb_rotation_deg: f32, toggle_gravity: bool) {
    let clamped = clamp_dash_angle_deg(orb_rotation_deg);
    player.dash_angle_deg = clamped;
    player.rotation = normalize_rotation_deg(clamped);
    player.dash_rotation_blocks_remaining = DASH_SPIN_DISTANCE_BLOCKS * WORLD_UNITS_PER_BLOCK;
    player.vy = 0.0;
    if toggle_gravity {
        player.gravity_sign = -player.gravity_sign;
    }
}

fn apply_green_dash_orb(player: &mut PlayerState, orb_rotation_deg: f32) -> bool {
    start_dash_orb(player, orb_rotation_deg, false);
    true
}

fn apply_pink_dash_orb(player: &mut PlayerState, orb_rotation_deg: f32) -> bool {
    start_dash_orb(player, orb_rotation_deg, true);
    true
}

fn apply_black_orb(player: &mut PlayerState) {
    let g = player.flip_mod();
    let cube_scale = mini_cube_pulse_scale(player);
    // Black orb: yvel = -15G for most modes; ship = -14G (then decays); UFO = -11.2G.
    player.vy = match player.mode {
        GameMode::Cube => -15.0 * g * cube_scale,
        GameMode::Ball | GameMode::Robot | GameMode::Spider => -15.0 * g,
        GameMode::Ship | GameMode::Swing => -14.0 * g,
        GameMode::Ufo => -11.2 * g,
        GameMode::Wave => player.vy,
    };
}

