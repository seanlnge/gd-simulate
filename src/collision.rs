use crate::{level::LevelObject, object_data::HitboxData};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rect {
    pub center: [f32; 2],
    pub half_extents: [f32; 2],
}

impl Rect {
    pub fn min_x(self) -> f32 {
        self.center[0] - self.half_extents[0]
    }

    pub fn max_x(self) -> f32 {
        self.center[0] + self.half_extents[0]
    }

    pub fn min_y(self) -> f32 {
        self.center[1] - self.half_extents[1]
    }

    pub fn max_y(self) -> f32 {
        self.center[1] + self.half_extents[1]
    }
}

pub fn intersects(a: Rect, b: Rect) -> bool {
    (a.center[0] - b.center[0]).abs() <= a.half_extents[0] + b.half_extents[0]
        && (a.center[1] - b.center[1]).abs() <= a.half_extents[1] + b.half_extents[1]
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OpengdBoxTransform {
    pub corners: [(f32, f32); 4],
    pub bounds: Rect,
}

/// `GameObject::setOuterBounds` semantics for `_pHitboxes` box rects.
///
/// Builds a local rect from the hitbox `{offset, half_extents}` (so the
/// rect is centered on `offset` in object-local space), applies rotation
/// and signed scale to all four corners, then translates by the object's
/// world position `(object.x, object.y)`.
///
/// Note: an earlier port of this routine added a `+ Vec2(15, 15)` shift
/// to match OpenGD's `setOuterBounds` (where `m_obj->getPosition()`
/// returns the bottom-left of a 30x30 sprite cell so `+15, +15`
/// translates that to the cell's center). Our level loader already
/// stores `object.x, object.y` as the visual *center* (matching
/// `slope_local_to_world_point` which never adds `+15`), so the extra
/// `+15` was a double-translation. Removing it makes box, circle, and
/// slope coordinates use the same convention.
pub fn opengd_box_transform(object: &LevelObject) -> Option<OpengdBoxTransform> {
    let HitboxData::Box {
        offset,
        half_extents,
    } = object.hitbox?
    else {
        return None;
    };

    let local_x = offset[0] - half_extents[0];
    let local_y = offset[1] - half_extents[1];
    let width = half_extents[0] * 2.0;
    let height = half_extents[1] * 2.0;
    let theta = object.rotation.to_radians();
    let (sin, cos) = (theta.sin(), theta.cos());
    let mut corners = [(0.0_f32, 0.0_f32); 4];

    for (index, (px, py)) in [
        (local_x, local_y),
        (local_x + width, local_y),
        (local_x + width, local_y + height),
        (local_x, local_y + height),
    ]
    .into_iter()
    .enumerate()
    {
        let sx = px * object.scale_x;
        let sy = py * object.scale_y;
        corners[index] = (
            object.x + sx * cos - sy * sin,
            object.y + sx * sin + sy * cos,
        );
    }

    Some(OpengdBoxTransform {
        corners,
        bounds: rect_from_corners(corners),
    })
}

fn rect_from_corners(corners: [(f32, f32); 4]) -> Rect {
    let mut min_x = f32::INFINITY;
    let mut max_x = f32::NEG_INFINITY;
    let mut min_y = f32::INFINITY;
    let mut max_y = f32::NEG_INFINITY;

    for (x, y) in corners {
        min_x = min_x.min(x);
        max_x = max_x.max(x);
        min_y = min_y.min(y);
        max_y = max_y.max(y);
    }

    Rect {
        center: [(min_x + max_x) * 0.5, (min_y + max_y) * 0.5],
        half_extents: [(max_x - min_x) * 0.5, (max_y - min_y) * 0.5],
    }
}

/// Port of GameObject::slopeYPos for rectangular slope bounds.
pub fn slope_y_pos(rect: Rect, player_x: f32, uphill: bool, hazard: bool, floor_top: bool) -> f32 {
    let slope_left = rect.min_x();
    let slope_right = rect.max_x();
    let slope_bottom = rect.min_y();
    let slope_top = rect.max_y();
    let slope_ratio = (rect.max_y() - rect.min_y()) / (rect.max_x() - rect.min_x());

    let mut result = if slope_left < player_x {
        let distance_from_right = player_x - slope_right;
        if uphill {
            slope_top + distance_from_right * slope_ratio
        } else {
            slope_bottom - distance_from_right * slope_ratio
        }
    } else {
        let distance_from_left = slope_left - player_x;
        if !uphill {
            slope_top + distance_from_left * slope_ratio
        } else {
            slope_bottom - distance_from_left * slope_ratio
        }
    };

    if hazard {
        result += if floor_top { -4.0 } else { 4.0 };
    }

    result
}

/// True if `(px,py)` is near the walkable hypotenuse of gdclone's slope
/// triangle (local space, center `0`). The X axis must lie within the
/// triangle's span. The perpendicular distance threshold is generous (10
/// local units) because after the GDP snap (`y = surface + player_half /
/// cos(angle)`) the cube's bottom corner sits perpendicular distance
/// `player_half * (1 - cos(angle))` above the hypotenuse — up to ~8.3 for a
/// 1x2 (63.4°) slope. The X-bound + AABB-overlap caller-side gate prevents
/// false positives for cubes inside the bounding box but on the wrong side
/// of the hypotenuse (snap_delta vs. snap_threshold rejects those).
pub fn near_slope_hypotenuse(px: f32, py: f32, hx: f32, hy: f32) -> bool {
    if hx < 1e-3 {
        return false;
    }
    let hyp = (hx * hx + hy * hy).sqrt();
    // Line through (-hx,-hy)->(hx,hy): hy*X - hx*Y = 0
    let dist = (hy * px - hx * py).abs() / hyp;
    if dist > 10.0 {
        return false;
    }
    px >= -hx - 1.0 && px <= hx + 1.0
}

/// World Y for player center when snapped to a rotated slope surface.
pub fn rotated_slope_player_world_y(
    cx: f32,
    cy: f32,
    rot_deg: f32,
    scale_x: f32,
    scale_y: f32,
    hx: f32,
    hy: f32,
    player_x: f32,
    _player_y: f32,
    player_half: f32,
    gravity_down: bool,
) -> Option<f32> {
    if scale_x.abs() < 1e-4 || scale_y.abs() < 1e-4 {
        return None;
    }

    let a = slope_local_to_world_point(cx, cy, rot_deg, scale_x, scale_y, -hx, -hy);
    let b = slope_local_to_world_point(cx, cy, rot_deg, scale_x, scale_y, hx, hy);
    let dx = b.0 - a.0;
    let dy = b.1 - a.1;
    if dx.abs() < 1e-4 {
        return None;
    }

    let min_x = a.0.min(b.0);
    let max_x = a.0.max(b.0);
    if player_x < min_x || player_x > max_x {
        return None;
    }

    let t = ((player_x - a.0) / dx).clamp(0.0, 1.0);
    let y_surf = a.1 + t * dy;
    // GDP `playerRadOnSlope = playerRadius / cos(getSlopeAngle())`.
    // `getSlopeAngle()` is the slope's *intrinsic* angle from its local
    // half-extents (atan2(hy, hx)), independent of world rotation/flip.
    // Using the world-rotated cos was wrong: a 1x2 slope rotated -90deg
    // would compute rad = 33.5 instead of 16.8, putting the snap target
    // far above the cube and outside the snap-attach threshold.
    let local_angle = hy.atan2(hx);
    let cos_angle = local_angle.cos().abs().max(1e-4);
    let rad = player_half / cos_angle;
    Some(if gravity_down {
        y_surf + rad
    } else {
        y_surf - rad
    })
}

pub fn transformed_slope_world_grade(
    cx: f32,
    cy: f32,
    rot_deg: f32,
    scale_x: f32,
    scale_y: f32,
    hx: f32,
    hy: f32,
) -> Option<f32> {
    if scale_x.abs() < 1e-4 || scale_y.abs() < 1e-4 {
        return None;
    }
    let a = slope_local_to_world_point(cx, cy, rot_deg, scale_x, scale_y, -hx, -hy);
    let b = slope_local_to_world_point(cx, cy, rot_deg, scale_x, scale_y, hx, hy);
    let dx = b.0 - a.0;
    if dx.abs() < 1e-4 {
        return None;
    }
    Some((b.1 - a.1) / dx)
}

pub fn slope_local_to_world_point(
    cx: f32,
    cy: f32,
    rot_deg: f32,
    scale_x: f32,
    scale_y: f32,
    x: f32,
    y: f32,
) -> (f32, f32) {
    let theta = rot_deg.to_radians();
    let sx = x * scale_x;
    let sy = y * scale_y;
    (
        cx + sx * theta.cos() - sy * theta.sin(),
        cy + sx * theta.sin() + sy * theta.cos(),
    )
}

pub fn slope_world_to_local_point(
    cx: f32,
    cy: f32,
    rot_deg: f32,
    scale_x: f32,
    scale_y: f32,
    wx: f32,
    wy: f32,
) -> Option<(f32, f32)> {
    if scale_x.abs() < 1e-4 || scale_y.abs() < 1e-4 {
        return None;
    }
    let theta = rot_deg.to_radians();
    let dx = wx - cx;
    let dy = wy - cy;
    // Inverse rotation, then inverse signed scale.
    let rx = dx * theta.cos() + dy * theta.sin();
    let ry = -dx * theta.sin() + dy * theta.cos();
    Some((rx / scale_x, ry / scale_y))
}

/// `gdp::collidedWithSlopeInternal` exit-speed magnitude, split into local
/// tangent components along the hypotenuse (up-ramp = toward `(hx,hy)`).
pub fn slope_exit_velocity_local(
    hx: f32,
    hy: f32,
    player_speed: f32,
    speed_multiplier: f32,
) -> (f32, f32) {
    let w = hx * 2.0;
    let h = hy * 2.0;
    if w <= 0.0 {
        return (0.0, 0.0);
    }
    let slope_angle = hy.atan2(hx);
    let mult = (1.12_f32 / slope_angle).min(1.54);
    let base = (h * player_speed * speed_multiplier) / w;
    let mag = mult * base;
    let hyp = (hx * hx + hy * hy).sqrt().max(1e-4);
    let tx = hx / hyp;
    let ty = hy / hyp;
    (tx * mag, ty * mag)
}

pub fn rotate_vec_local_to_world(vx_loc: f32, vy_loc: f32, rot_deg: f32) -> (f32, f32) {
    let theta = rot_deg.to_radians();
    let vx_w = vx_loc * theta.cos() - vy_loc * theta.sin();
    let vy_w = vx_loc * theta.sin() + vy_loc * theta.cos();
    (vx_w, vy_w)
}

/// Pad / orb boost direction: local +Y is the default “straight” boost,
/// rotated by engine degrees (negated RobTop key `6`, matching gdclone).
#[inline]
pub fn boost_vector_world(magnitude: f32, rot_deg: f32) -> (f32, f32) {
    let theta = rot_deg.to_radians();
    (magnitude * theta.sin(), magnitude * theta.cos())
}
