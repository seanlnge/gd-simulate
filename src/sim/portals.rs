use crate::consts::{
    PLAYER_SPEED_0_5X, PLAYER_SPEED_1X, PLAYER_SPEED_2X, PLAYER_SPEED_3X, PLAYER_SPEED_4X,
};

// ---------- Portals ----------

#[allow(clippy::too_many_arguments)]
fn apply_portals(
    level: &Level,
    player: &mut PlayerState,
    touched: &mut HashSet<usize>,
    teleport_exits: &[&LevelObject],
    touched_teleports: &mut HashSet<usize>,
    dual_activate: &mut bool,
    dual_deactivate: &mut bool,
    which_player: u8,
) {
    for (idx, object) in level.objects.iter().enumerate() {
        let key = idx * 2 + which_player as usize; // per-player touched index
        let already = touched.contains(&key);
        let is_portal = matches!(
            object.kind,
            ObjectKind::ModePortal
                | ObjectKind::SpeedPortal
                | ObjectKind::GravityPortal
                | ObjectKind::SizePortal
                | ObjectKind::MirrorPortal
                | ObjectKind::DualPortal
                | ObjectKind::TeleportPortal
        );
        if !is_portal || already {
            continue;
        }
        if !intersects_player(object, *player) {
            continue;
        }
        touched.insert(key);
        match object.kind {
            ObjectKind::ModePortal => {
                let next_mode = mode_for_portal(object.object_id);
                if next_mode == GameMode::Ship && player.mode != GameMode::Ship {
                    player.vy /= 2.0;
                    player.on_ground = false;
                }
                player.mode = next_mode;
            }
            ObjectKind::SpeedPortal => {
                let ps = player_speed_for_portal(object.object_id);
                let profile = SpeedProfile::for_player_speed(ps);
                player.player_speed = ps;
                player.speed_multiplier = profile.speed_multiplier;
                player.gravity = profile.gravity;
                player.y_start = profile.y_start;
                player.vx = profile.speed_multiplier;
            }
            ObjectKind::GravityPortal => {
                let next_gravity_sign = if object.object_id == 10 { -1.0 } else { 1.0 };
                if (player.gravity_sign - next_gravity_sign).abs() > f32::EPSILON {
                    player.vy /= 2.0;
                }
                player.gravity_sign = next_gravity_sign;
            }
            ObjectKind::SizePortal => {
                player.mini = object.object_id == 101;
                player.vehicle_size = if player.mini { 0.6 } else { 1.0 };
            }
            ObjectKind::MirrorPortal => {
                // Mirror portals have no gameplay effect in GD 2.2 — visual only.
            }
            ObjectKind::DualPortal => {
                // 286 = orange (activate dual), 287 = blue (deactivate dual).
                if object.object_id == 286 {
                    *dual_activate = true;
                } else if object.object_id == 287 {
                    *dual_deactivate = true;
                }
            }
            ObjectKind::TeleportPortal if object.object_id == 747 => {
                // Blue teleport portal: vertical teleport to paired orange (749).
                // Pair by nearest exit in X (teleport portals are usually placed
                // close together). Gravity unchanged, velocity preserved.
                if let Some(exit) = nearest_exit(object, teleport_exits) {
                    if !touched_teleports.contains(&idx) {
                        touched_teleports.insert(idx);
                        player.y = exit.y;
                    }
                }
            }
            _ => {}
        }
    }
}

fn nearest_exit<'a>(entry: &LevelObject, exits: &'a [&'a LevelObject]) -> Option<&'a LevelObject> {
    exits
        .iter()
        .min_by(|a, b| {
            (a.x - entry.x)
                .abs()
                .partial_cmp(&(b.x - entry.x).abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .copied()
}

fn mode_for_portal(object_id: u32) -> GameMode {
    match object_id {
        // GD's portal IDs: id 12 is the *cube* portal (portal_03 sprite,
        // blue) and id 13 is the *ship* portal (portal_04 sprite, pink).
        // We had these backwards, which is why vierre's first portal
        // (id=13 at x=255) was not flipping the player into ship mode -
        // we were treating it as a cube portal (no-op) and the cube
        // continued into the spike at (315, 306) and died as a cube.
        12 => GameMode::Cube,
        13 => GameMode::Ship,
        47 => GameMode::Ball,
        111 => GameMode::Ufo,
        660 => GameMode::Wave,
        745 => GameMode::Robot,
        1331 => GameMode::Spider,
        1933 | 2862 => GameMode::Swing,
        _ => GameMode::Cube,
    }
}

fn player_speed_for_portal(object_id: u32) -> f32 {
    match object_id {
        200 => PLAYER_SPEED_0_5X,
        201 => PLAYER_SPEED_1X,
        202 => PLAYER_SPEED_2X,
        203 => PLAYER_SPEED_3X,
        1334 => PLAYER_SPEED_4X,
        _ => PLAYER_SPEED_1X,
    }
}

