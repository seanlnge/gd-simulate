// ---------- Pads ----------

fn apply_pads(
    level: &Level,
    player: &mut PlayerState,
    touched: &mut HashSet<usize>,
    which_player: u8,
) {
    let mut applied_this_tick = false;
    for (idx, object) in level.objects.iter().enumerate() {
        if applied_this_tick {
            break;
        }
        let key = idx * 2 + which_player as usize;
        if object.kind != ObjectKind::Pad || touched.contains(&key) {
            continue;
        }
        if !intersects_pad_activation(object, *player) {
            continue;
        }
        let flip_mod = player.flip_mod();
        let rot = object.rotation;
        let sy = object.scale_y;
        let activated = match object.object_id {
            35 => {
                apply_yellow_pad(player, flip_mod, rot, sy);
                true
            }
            67 => {
                if gravity_pad_can_activate(player.gravity_sign, rot, sy) {
                    apply_blue_pad(player, rot, sy);
                    true
                } else {
                    false
                }
            }
            140 => {
                apply_purple_pad(player, flip_mod, rot, sy);
                true
            }
            1332 => {
                apply_red_pad(player, flip_mod, rot, sy);
                true
            }
            _ => false,
        };
        if !activated {
            continue;
        }
        touched.insert(key);
        player.on_ground = false;
        applied_this_tick = true;
    }
}

fn apply_blue_pads_pre_collision(
    level: &Level,
    player: &mut PlayerState,
    touched: &mut HashSet<usize>,
    which_player: u8,
) {
    for (object_index, object) in nearby_objects(level, player.x) {
        if object.kind != ObjectKind::Pad || object.object_id != 67 {
            continue;
        }
        let key = object_index * 2 + which_player as usize;
        if touched.contains(&key) || !intersects_pad_activation(object, *player) {
            continue;
        }
        if !gravity_pad_can_activate(player.gravity_sign, object.rotation, object.scale_y) {
            continue;
        }
        touched.insert(key);
        apply_blue_pad(player, object.rotation, object.scale_y);
        player.on_ground = false;
    }
}

/// OpenGD's `propellPlayer` applies pad force along the player's gravity axis;
/// object rotation/flip changes the hitbox/animation, not the impulse direction.
fn apply_pad_vector(player: &mut PlayerState, magnitude: f32, _rot_deg: f32, _scale_y: f32) {
    if magnitude == 0.0 {
        return;
    }
    player.vy = magnitude;
    // GDP `boostPlayer` sets `m_isAccelerating = true` whenever a pad/orb
    // imparts an impulse. This persists until the player next touches
    // ground (cleared in `update_cube` and similar mode-update helpers).
    // Cube gravity branches on this flag - an accelerating, falling
    // (not-jump-buffered) cube falls with `factor=-0.6` instead of -1.08,
    // so post-pad arcs travel notably farther than naturally-jumped arcs.
    player.is_accelerating = true;
}

fn mini_cube_pulse_scale(player: &PlayerState) -> f32 {
    if player.mode == GameMode::Cube && player.mini {
        0.8
    } else {
        1.0
    }
}

/// Pad impulse tables from GD Docs (boomlings.dev/reference/player_physics).
/// "G" in the table = `flip_mod` (gravity-direction scalar) for an unrotated pad.
fn apply_yellow_pad(player: &mut PlayerState, flip_mod: f32, rot_deg: f32, scale_y: f32) {
    let g = flip_mod;
    let cube_scale = mini_cube_pulse_scale(player);
    let mag = match player.mode {
        GameMode::Cube => 16.0 * g * cube_scale,
        GameMode::Robot => 16.0 * g,
        GameMode::Ship | GameMode::Ufo => {
            player.pending_yvel_next_tick = 8.0 * g;
            16.0 * g
        }
        GameMode::Ball | GameMode::Spider | GameMode::Swing => 9.6 * g,
        GameMode::Wave => 0.0,
    };
    apply_pad_vector(player, mag, rot_deg, scale_y);
}

fn apply_purple_pad(player: &mut PlayerState, flip_mod: f32, rot_deg: f32, scale_y: f32) {
    let g = flip_mod;
    let cube_scale = mini_cube_pulse_scale(player);
    let mag = match player.mode {
        GameMode::Cube => 10.4 * g * cube_scale,
        GameMode::Robot => 10.4 * g,
        GameMode::Ship => 5.6 * g,
        GameMode::Ufo => 6.4 * g,
        GameMode::Ball | GameMode::Spider => 6.72 * g,
        GameMode::Swing => 6.24 * g,
        GameMode::Wave => 0.0,
    };
    apply_pad_vector(player, mag, rot_deg, scale_y);
}

fn apply_red_pad(player: &mut PlayerState, flip_mod: f32, rot_deg: f32, scale_y: f32) {
    let g = flip_mod;
    let cube_scale = mini_cube_pulse_scale(player);
    let mag = match player.mode {
        GameMode::Cube => 20.0 * g * cube_scale,
        GameMode::Robot => 20.0 * g,
        GameMode::Ship => 10.08 * g,
        GameMode::Ufo => 9.6 * g,
        GameMode::Ball | GameMode::Spider | GameMode::Swing => 12.0 * g,
        GameMode::Wave => 0.0,
    };
    apply_pad_vector(player, mag, rot_deg, scale_y);
}

fn apply_blue_pad(player: &mut PlayerState, rot_deg: f32, scale_y: f32) {
    let flip_mod_before = player.flip_mod();
    let cube_scale = mini_cube_pulse_scale(player);
    let mag = match player.mode {
        GameMode::Cube => 6.4 * flip_mod_before * cube_scale,
        GameMode::Robot | GameMode::Ship | GameMode::Ufo => 6.4 * flip_mod_before,
        GameMode::Ball | GameMode::Spider | GameMode::Swing => 3.84 * flip_mod_before,
        GameMode::Wave => 0.0,
    };
    apply_pad_vector(player, mag, rot_deg, scale_y);
    player.gravity_sign = -player.gravity_sign;
}

fn gravity_pad_can_activate(gravity_sign: f32, rotation_deg: f32, scale_y: f32) -> bool {
    // Use effective pad facing, including signed Y scale.
    // Positive world Y means the pad faces more upward than downward.
    let theta = rotation_deg.to_radians();
    let mut facing_world_y = scale_y.signum() * theta.cos();
    // Sideways pads (e.g. 90deg) should not be rejected by floating-point
    // noise (`cos(90deg)` in f32 is a tiny signed epsilon).
    if facing_world_y.abs() < 1e-4 {
        facing_world_y = 0.0;
    }
    if gravity_sign < 0.0 {
        facing_world_y >= 0.0
    } else {
        facing_world_y <= 0.0
    }
}

fn intersects_pad_activation(object: &LevelObject, player: PlayerState) -> bool {
    let player_half = player.player_half();
    // Pad activation follows the same transformed object hitbox drawn by the
    // visualizer. Do not use the old tall legacy activation rectangles here:
    // they visibly trigger pads before player and pad hitboxes touch.
    if let Some(transform) = opengd_box_transform(object) {
        return intersects_oriented_quad_player(transform.corners, player, player_half);
    }
    let Some(rect) = object_rect(object) else {
        return false;
    };
    intersects_pad_player(rect, player, player_half, object.rotation)
}

fn intersects_pad_player(rect: Rect, player: PlayerState, player_half: f32, object_rotation: f32) -> bool {
    // Pads should trigger from the cube's rotated outer hitbox while airborne,
    // not the inner centered "blue box" used for some block lethality checks.
    // Non-rotated pad volumes use the cube's non-rotated outer box. Rotated
    // pad volumes use the rotated-outer proxy (circumscribed radius).
    if player.mode == GameMode::Cube && !player.on_ground {
        let theta = object_rotation.to_radians();
        let pad_axis_aligned = theta.sin().abs() < 1e-4;
        if pad_axis_aligned {
            return intersects_box_player(rect, player, player_half);
        }
        let radius = player_half * std::f32::consts::SQRT_2;
        let closest_x = player.x.clamp(
            rect.center[0] - rect.half_extents[0],
            rect.center[0] + rect.half_extents[0],
        );
        let closest_y = player.y.clamp(
            rect.center[1] - rect.half_extents[1],
            rect.center[1] + rect.half_extents[1],
        );
        let dx = player.x - closest_x;
        let dy = player.y - closest_y;
        return dx * dx + dy * dy <= radius * radius;
    }
    intersects_box_player(rect, player, player_half)
}

