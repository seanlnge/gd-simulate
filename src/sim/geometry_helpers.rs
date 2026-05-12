fn object_rect(object: &LevelObject) -> Option<Rect> {
    let hb = match object.hitbox {
        Some(hb) => hb,
        None => return synthetic_activation_rect(object),
    };
    let (cx, cy, hx, hy) = match hb {
        HitboxData::Box { .. } => {
            return opengd_box_transform(object).map(|transform| transform.bounds);
        }
        HitboxData::Slope { half_extents } => (
            object.x,
            object.y,
            half_extents[0] * object.scale_x.abs(),
            half_extents[1] * object.scale_y.abs(),
        ),
        HitboxData::Circle { radius } => {
            let [cx, cy] = circle_hazard_center(object);
            (cx, cy, radius, radius)
        }
    };
    // Apply rotation: cocos2d AABB of rotated local box (level `rotation` is
    // already negated key `6` to match gdclone).
    let (rhx, rhy) = if object.rotation == 0.0 {
        (hx, hy)
    } else {
        let theta = object.rotation.to_radians();
        let (s, c) = (theta.sin().abs(), theta.cos().abs());
        (hx * c + hy * s, hx * s + hy * c)
    };
    Some(Rect {
        center: [cx, cy],
        half_extents: [rhx, rhy],
    })
}

fn ceiling_death_blocked_by_h_square(level: &Level, player: PlayerState) -> bool {
    for (_, object) in nearby_objects(level, player.x) {
        if object.object_id != 1859 {
            continue;
        }
        let Some(rect) = object_rect(object) else {
            continue;
        };
        if intersects_box_player(rect, player, player.player_half()) {
            return true;
        }
    }
    false
}

/// Some gameplay objects (e.g. teleport portals 747/749) have no hitbox in
/// `gdclone/assets/data/object.json`, but they still need a touch volume for
/// activation. For activation-only objects we fabricate a portal-sized box
/// centered on the object; for anything else we return None so they behave
/// as purely visual.
fn synthetic_activation_rect(object: &LevelObject) -> Option<Rect> {
    match object.kind {
        ObjectKind::TeleportPortal
        | ObjectKind::ModePortal
        | ObjectKind::SpeedPortal
        | ObjectKind::GravityPortal
        | ObjectKind::SizePortal
        | ObjectKind::MirrorPortal
        | ObjectKind::DualPortal
        | ObjectKind::Orb
        | ObjectKind::Pad => Some(Rect {
            center: [object.x, object.y],
            half_extents: [15.0, 45.0],
        }),
        _ => None,
    }
}

fn intersects_player(object: &LevelObject, player: PlayerState) -> bool {
    if is_portal_kind(object.kind) {
        if let Some(corners) = portal_activation_quad(object) {
            return intersects_oriented_quad_player(corners, player, player.player_half());
        }
        return false;
    }
    let Some(rect) = object_rect(object) else {
        return false;
    };
    intersects_box_player(rect, player, player.player_half())
}

fn is_portal_kind(kind: ObjectKind) -> bool {
    matches!(
        kind,
        ObjectKind::ModePortal
            | ObjectKind::SpeedPortal
            | ObjectKind::GravityPortal
            | ObjectKind::SizePortal
            | ObjectKind::MirrorPortal
            | ObjectKind::DualPortal
            | ObjectKind::TeleportPortal
    )
}

fn portal_activation_quad(object: &LevelObject) -> Option<[(f32, f32); 4]> {
    if let Some(transform) = opengd_box_transform(object) {
        return Some(transform.corners);
    }
    // Fallback for activation-only portal objects that do not have a box
    // hitbox in object data.
    if is_portal_kind(object.kind) {
        let hx = 15.0_f32;
        let hy = 45.0_f32;
        return Some([
            slope_local_to_world_point(
                object.x,
                object.y,
                object.rotation,
                object.scale_x,
                object.scale_y,
                -hx,
                -hy,
            ),
            slope_local_to_world_point(
                object.x,
                object.y,
                object.rotation,
                object.scale_x,
                object.scale_y,
                hx,
                -hy,
            ),
            slope_local_to_world_point(
                object.x,
                object.y,
                object.rotation,
                object.scale_x,
                object.scale_y,
                hx,
                hy,
            ),
            slope_local_to_world_point(
                object.x,
                object.y,
                object.rotation,
                object.scale_x,
                object.scale_y,
                -hx,
                hy,
            ),
        ]);
    }
    None
}

fn intersects_oriented_quad_player(
    quad: [(f32, f32); 4],
    player: PlayerState,
    player_half: f32,
) -> bool {
    let player_corners = [
        (player.x - player_half, player.y - player_half),
        (player.x + player_half, player.y - player_half),
        (player.x + player_half, player.y + player_half),
        (player.x - player_half, player.y + player_half),
    ];

    let mut axes = Vec::with_capacity(4);
    axes.push((1.0_f32, 0.0_f32));
    axes.push((0.0_f32, 1.0_f32));
    for i in 0..2 {
        let (x1, y1) = quad[i];
        let (x2, y2) = quad[(i + 1) % 4];
        let edge_x = x2 - x1;
        let edge_y = y2 - y1;
        let axis = (-edge_y, edge_x);
        let len_sq = axis.0 * axis.0 + axis.1 * axis.1;
        if len_sq > 1e-8 {
            let inv_len = len_sq.sqrt().recip();
            axes.push((axis.0 * inv_len, axis.1 * inv_len));
        }
    }

    for axis in axes {
        let (q_min, q_max) = project_points(axis, &quad);
        let (p_min, p_max) = project_points(axis, &player_corners);
        if q_max < p_min || p_max < q_min {
            return false;
        }
    }
    true
}

fn project_points(axis: (f32, f32), points: &[(f32, f32); 4]) -> (f32, f32) {
    let mut min = f32::INFINITY;
    let mut max = f32::NEG_INFINITY;
    for &(x, y) in points {
        let p = x * axis.0 + y * axis.1;
        min = min.min(p);
        max = max.max(p);
    }
    (min, max)
}

fn intersects_box_player(rect: Rect, player: PlayerState, player_half: f32) -> bool {
    (player.x - rect.center[0]).abs() <= player_half + rect.half_extents[0]
        && (player.y - rect.center[1]).abs() <= player_half + rect.half_extents[1]
}

fn lethal_hazard_intersects(
    object: &LevelObject,
    rect: Rect,
    player: PlayerState,
    player_half: f32,
) -> bool {
    match object.hitbox {
        Some(HitboxData::Circle { radius }) => {
            let [cx, cy] = circle_hazard_center(object);
            let closest_x = cx.clamp(player.x - player_half, player.x + player_half);
            let closest_y = cy.clamp(player.y - player_half, player.y + player_half);
            let dx = cx - closest_x;
            let dy = cy - closest_y;
            dx * dx + dy * dy <= radius.powi(2)
        }
        _ => intersects_box_player(rect, player, player_half),
    }
}

fn circle_hazard_center(object: &LevelObject) -> [f32; 2] {
    // Our level loader stores `object.x, object.y` as the visual center
    // (the cell-center convention used by `slope_local_to_world_point`).
    // OpenGD's literal `+ Vec2(15, 15)` only applies inside its own
    // engine because `m_obj->getPosition()` returns the cell's
    // bottom-left; we don't have that double-translation, so circle
    // hazards are checked at the object's stored position directly.
    [object.x, object.y]
}

fn side_hit_is_lethal(
    rect: Rect,
    player: PlayerState,
    _player_half: f32,
    on_slope_this_tick: bool,
    was_on_slope: bool,
    slope_contact_cooldown: u8,
) -> bool {
    // GDP reaches the lethal tail only after slope/snap resolution. When the
    // cube is still being carried by a slope transition, the contact is a
    // collision candidate, not an immediate side death.
    if slope_contact_cooldown > 0 || player.on_ground && (on_slope_this_tick || was_on_slope) {
        return false;
    }
    let inner_half = cube_lethal_inner_half(&player);
    if player.mode == GameMode::Ship {
        return intersects_rotated_player_inner(rect, player, inner_half);
    }
    intersects_box_player(rect, player, inner_half)
}

fn intersects_rotated_player_inner(rect: Rect, player: PlayerState, inner_half: f32) -> bool {
    let theta = player.rotation.to_radians();
    let (s, c) = (theta.sin(), theta.cos());
    let local = [
        (-inner_half, -inner_half),
        (inner_half, -inner_half),
        (inner_half, inner_half),
        (-inner_half, inner_half),
    ];
    let mut player_corners = [(0.0_f32, 0.0_f32); 4];
    for (i, (x, y)) in local.into_iter().enumerate() {
        player_corners[i] = (player.x + x * c - y * s, player.y + x * s + y * c);
    }
    let rect_corners = [
        (
            rect.center[0] - rect.half_extents[0],
            rect.center[1] - rect.half_extents[1],
        ),
        (
            rect.center[0] + rect.half_extents[0],
            rect.center[1] - rect.half_extents[1],
        ),
        (
            rect.center[0] + rect.half_extents[0],
            rect.center[1] + rect.half_extents[1],
        ),
        (
            rect.center[0] - rect.half_extents[0],
            rect.center[1] + rect.half_extents[1],
        ),
    ];

    let mut axes = Vec::with_capacity(4);
    axes.push((1.0_f32, 0.0_f32));
    axes.push((0.0_f32, 1.0_f32));
    for i in 0..2 {
        let (x1, y1) = player_corners[i];
        let (x2, y2) = player_corners[(i + 1) % 4];
        let axis = (-(y2 - y1), x2 - x1);
        let len_sq = axis.0 * axis.0 + axis.1 * axis.1;
        if len_sq > 1e-8 {
            let inv = len_sq.sqrt().recip();
            axes.push((axis.0 * inv, axis.1 * inv));
        }
    }
    for axis in axes {
        let (rmin, rmax) = project_points(axis, &rect_corners);
        let (pmin, pmax) = project_points(axis, &player_corners);
        if rmax < pmin || pmax < rmin {
            return false;
        }
    }
    true
}

fn cube_lethal_inner_half(player: &PlayerState) -> f32 {
    if player.mode == GameMode::Cube {
        if player.mini {
            5.0
        } else {
            OPENGD_CUBE_INNER_HALF
        }
    } else {
        player.player_half() * 0.3
    }
}

fn check_snap_jump_to_object_for_level(
    level: &Level,
    player: &mut PlayerState,
    object_index: usize,
) {
    let Some(current) = level.objects.get(object_index) else {
        return;
    };
    let previous = player
        .snapped_object
        .and_then(|previous_index| level.objects.get(previous_index));
    if let Some(previous) = previous {
        check_snap_jump_to_object(player, object_index, previous, current);
    } else {
        player.snapped_object = Some(object_index);
        player.snap_distance = player.x - current.x;
    }
}

fn check_snap_jump_to_object(
    player: &mut PlayerState,
    object_index: usize,
    previous: &LevelObject,
    current: &LevelObject,
) {
    let pos_x = player.x;
    if player.snapped_object != Some(object_index) && previous.kind == ObjectKind::Solid {
        let (threshold, little_stair, down_stair, big_stair) =
            snap_jump_thresholds(player.player_speed, player.vehicle_size);
        let block_length = player.flip_mod() * 30.0;
        let diff_x = current.x - previous.x;
        let diff_y = current.y - previous.y;
        let is_snap_stair = ((diff_x - little_stair).abs() <= threshold
            && (diff_y - block_length).abs() <= threshold)
            || ((diff_x - down_stair).abs() <= threshold
                && (diff_y + block_length).abs() <= threshold)
            || ((diff_x - big_stair).abs() <= threshold
                && (diff_y - block_length * 2.0).abs() <= threshold);
        if is_snap_stair {
            let target_x = current.x + player.snap_distance;
            if (target_x - pos_x).abs() > threshold {
                player.x = if target_x <= pos_x {
                    pos_x - threshold
                } else {
                    pos_x + threshold
                };
            } else {
                player.x = target_x;
            }
        }
    }
    player.snapped_object = Some(object_index);
    player.snap_distance = pos_x - current.x;
}

fn snap_jump_thresholds(player_speed: f32, vehicle_size: f32) -> (f32, f32, f32, f32) {
    if (player_speed - 0.9).abs() < 0.001 {
        (
            1.0,
            if vehicle_size == 1.0 { 120.0 } else { 90.0 },
            150.0,
            90.0,
        )
    } else if (player_speed - 0.7).abs() < 0.001 {
        (1.0, 90.0, 120.0, 60.0)
    } else if (player_speed - 1.1).abs() < 0.001 {
        (
            2.0,
            if vehicle_size == 1.0 { 150.0 } else { 90.0 },
            195.0,
            120.0,
        )
    } else if (player_speed - 1.3).abs() < 0.001 {
        (2.0, 90.0, 225.0, 135.0)
    } else if vehicle_size == 1.0 {
        (2.0, 180.0, 225.0, 135.0)
    } else {
        (1.0, 120.0, 150.0, 90.0)
    }
}

fn nearby_objects(level: &Level, player_x: f32) -> impl Iterator<Item = (usize, &LevelObject)> {
    level
        .objects
        .iter()
        .enumerate()
        .filter(move |(_, object)| (object.x - player_x).abs() < 120.0)
}

