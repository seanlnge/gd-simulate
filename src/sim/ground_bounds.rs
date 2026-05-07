fn refresh_ground_probe(level: &Level, player: &mut PlayerState) {
    if player.on_ground || !is_falling_toward_gravity(player) {
        return;
    }
    if grounded_by_probe(level, *player) {
        player.on_ground = true;
    }
}

fn is_falling_toward_gravity(player: &PlayerState) -> bool {
    // `flip_mod` points opposite gravity; falling means velocity points with gravity.
    player.vy * player.flip_mod() <= 0.0
}

fn grounded_by_probe(level: &Level, player: PlayerState) -> bool {
    let half = player.player_half();
    let max_gap = GROUND_PROBE_DISTANCE + GROUND_PROBE_HEIGHT + 0.001;
    let gravity_down = player.gravity_sign < 0.0;

    if gravity_down {
        let floor_gap = player.y - half - IMPLICIT_FLOOR_Y;
        if floor_gap >= -0.001 && floor_gap <= max_gap {
            return true;
        }
    }

    for (_, object) in nearby_objects(level, player.x) {
        if is_h_block(object) {
            continue;
        }
        if !matches!(object.kind, ObjectKind::Solid | ObjectKind::Slope) {
            continue;
        }
        let Some(rect) = object_rect(object) else {
            continue;
        };
        if (player.x - rect.center[0]).abs() > half + rect.half_extents[0] {
            continue;
        }

        if gravity_down {
            let support_y = rect.center[1] + rect.half_extents[1];
            let gap = player.y - half - support_y;
            if gap >= -0.001 && gap <= max_gap {
                return true;
            }
        } else {
            let support_y = rect.center[1] - rect.half_extents[1];
            let gap = support_y - (player.y + half);
            if gap >= -0.001 && gap <= max_gap {
                return true;
            }
        }
    }
    false
}

// ---------- Collision ----------

/// Implicit ground / ceiling that GD enforces even without explicit blocks.
/// The captured Pop baseline has floor top at y=90, so a 30px cube rests with
/// center y=105. The upper death ceiling sits 80 grid blocks above that center:
/// `30 * 80 + 105 = 2505`.
const IMPLICIT_FLOOR_Y: f32 = 90.0;
const IMPLICIT_CEILING_DEATH_Y: f32 = 2505.0;
const OPENGD_CUBE_INNER_HALF: f32 = 4.5;

fn ball_window_bounds(level: &Level, player: &PlayerState) -> Option<(f32, f32)> {
    if player.mode != GameMode::Ball {
        return None;
    }
    let portal = level
        .objects
        .iter()
        .filter(|obj| obj.kind == ObjectKind::ModePortal && obj.object_id == 47 && obj.x <= player.x)
        .max_by(|a, b| a.x.partial_cmp(&b.x).unwrap_or(std::cmp::Ordering::Equal))?;
    // Ball portal corridor: 8 blocks tall (240u), centered around the
    // highest 30u interval at the portal's Y.
    let f = (portal.y / WORLD_UNITS_PER_BLOCK).floor() * WORLD_UNITS_PER_BLOCK;
    Some((f - 120.0, f + 120.0))
}

fn apply_implicit_bounds(_level: &Level, player: &mut PlayerState) -> bool {
    let mut grounded = false;
    let player_half = player.player_half();
    let ball_window = ball_window_bounds(_level, player);
    let floor_top = ball_window.map(|(floor, _)| floor).unwrap_or(IMPLICIT_FLOOR_Y);
    let ceiling_y = ball_window.map(|(_, ceiling)| ceiling);
    let gravity_down = player.gravity_sign < 0.0;
    if gravity_down {
        // Floor catches the player when falling.
        if player.y - player_half <= floor_top && player.vy <= 0.0 {
            player.y = floor_top + player_half;
            player.vy = 0.0;
            player.on_ground = true;
            grounded = true;
        }
        if let Some(ceiling) = ceiling_y {
            // Ball mode corridor uses a clamped ceiling instead of death.
            if player.y + player_half >= ceiling && player.vy > 0.0 {
                player.y = ceiling - player_half;
                player.vy = 0.0;
            }
        }
    } else {
        if let Some(ceiling) = ceiling_y {
            // Flipped ball can "ground" on the window's upper surface.
            if player.y + player_half >= ceiling && player.vy >= 0.0 {
                player.y = ceiling - player_half;
                player.vy = 0.0;
                player.on_ground = true;
                grounded = true;
            }
        }
        if player.y - player_half <= floor_top && player.vy < 0.0 {
            player.y = floor_top + player_half;
            player.vy = 0.0;
        }
    }
    grounded
}

