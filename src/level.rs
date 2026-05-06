use std::collections::HashMap;

use serde::Serialize;

use crate::{
    SimError, SimResult,
    object_data::{HitboxData, ObjectDatabase},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ObjectKind {
    Solid,
    Hazard,
    Slope,
    ModePortal,
    SpeedPortal,
    GravityPortal,
    SizePortal,
    MirrorPortal,
    DualPortal,
    TeleportPortal,
    Pad,
    Orb,
    Trigger,
    Decoration,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct LevelObject {
    pub object_id: u32,
    pub x: f32,
    pub y: f32,
    /// Engine rotation in degrees = **negated** RobTop key `6` (cocos2d/gdclone).
    pub rotation: f32,
    /// Legacy uniform scale from key `32` (kept for diagnostics).
    pub scale: f32,
    /// Signed transform scales after applying key `32`, key `128/129`,
    /// and flip flags key `4/5`.
    pub scale_x: f32,
    pub scale_y: f32,
    pub groups: Vec<u32>,
    pub kind: ObjectKind,
    #[serde(skip_serializing)]
    pub hitbox: Option<HitboxData>,
    pub raw: HashMap<String, String>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Level {
    pub header: HashMap<String, String>,
    pub objects: Vec<LevelObject>,
}

impl Level {
    pub fn parse(levelstring: &str, db: &ObjectDatabase) -> SimResult<Self> {
        let mut segments = levelstring.split(';').filter(|segment| !segment.is_empty());
        let header = segments
            .next()
            .map(parse_pairs)
            .transpose()?
            .unwrap_or_default();

        let mut objects = Vec::new();
        for segment in segments {
            let raw = parse_pairs(segment)?;
            let object_id = raw
                .get("1")
                .ok_or_else(|| SimError::LevelParse("object missing property 1/id".to_owned()))?
                .parse::<u32>()
                .map_err(|error| SimError::LevelParse(error.to_string()))?;
            let default = db.get(object_id);
            let hitbox = opengd_fallback_hitbox(object_id)
                .or_else(|| default.as_ref().and_then(|view| view.hitbox));
            let kind = classify_object(object_id, hitbox);
            let uniform_scale = number_prop(&raw, "32", 1.0)?;
            let mut scale_x = number_prop(&raw, "128", uniform_scale)?;
            let mut scale_y = number_prop(&raw, "129", uniform_scale)?;
            if bool_prop(&raw, "4")? {
                scale_x *= -1.0;
            }
            if bool_prop(&raw, "5")? {
                scale_y *= -1.0;
            }
            objects.push(LevelObject {
                object_id,
                x: number_prop(&raw, "2", 0.0)?,
                // OpenGD applies a `+90` world-space offset when loading
                // level object Y coordinates (`PlayLayer::loadLevel`, key 3).
                // Real GD trace places the cube at y=105 with the floor
                // at y=90, so a slope/box at raw key 3 = 15 (level y=15)
                // sits with bottom at y=90 = floor, matching cube bottom.
                // No further `+15` adjustment is applied here.
                y: number_prop(&raw, "3", 0.0)? + 90.0,
                // RobTop key `6` is stored positive in levelstrings; cocos2d/gdclone
                // use the negated angle for transforms (`angle = -degrees`).
                rotation: -number_prop(&raw, "6", 0.0)?,
                scale: uniform_scale,
                scale_x,
                scale_y,
                groups: groups_prop(raw.get("57")),
                kind,
                hitbox,
                raw,
            });
        }

        Ok(Self { header, objects })
    }
}

pub fn classify_object(object_id: u32, hitbox: Option<HitboxData>) -> ObjectKind {
    match object_id {
        // Mode portals: cube 12, ship 13, ball 47, ufo 111, wave 660, robot 745,
        // spider 1331, swing 1933/2862. Values cross-checked against GD 2.2.
        // (ID 12 = portal_03 = blue cube portal; ID 13 = portal_04 = pink ship portal.)
        12 | 13 | 47 | 111 | 660 | 745 | 1331 | 1933 | 2862 => ObjectKind::ModePortal,
        // Speed portals: authoritative from gdclone::insert_trigger_data.
        // 200 = 0.5x, 201 = 1x, 202 = 2x, 203 = 3x, 1334 = 4x.
        200 | 201 | 202 | 203 | 1334 => ObjectKind::SpeedPortal,
        // Gravity portals: 10 flips down, 11 flips up (blue/yellow arrow).
        10 | 11 => ObjectKind::GravityPortal,
        // Size portals: 101 = mini, 99 = big.
        99 | 101 => ObjectKind::SizePortal,
        // Mirror/dual/teleport: detected but unsupported — fail loudly.
        45 | 46 => ObjectKind::MirrorPortal,
        286 | 287 => ObjectKind::DualPortal,
        747 | 749 => ObjectKind::TeleportPortal,
        // Pads: 35 yellow, 67 blue/gravity, 140 purple, 1332 red.
        35 | 67 | 140 | 1332 => ObjectKind::Pad,
        // Orbs: 36 yellow, 84 gravity ring, 141 pink, 1022 gravity-jump,
        // 1330 black, 1333 red, 1594 green, 1704 dash, 1751 spider, 3004 toggle.
        36 | 84 | 141 | 1022 | 1330 | 1333 | 1594 | 1704 | 1751 | 3004 => ObjectKind::Orb,
        // Triggers authoritative from gdclone::insert_trigger_data.
        29 | 30 | 105 | 221 | 717 | 718 | 743 | 744 | 899 => ObjectKind::Trigger,
        901 | 1006 | 1007 | 1049 | 1268 | 1346 | 1347 | 1520 | 1611 | 1616 | 1811 | 1815 | 1817 => {
            ObjectKind::Trigger
        }
        3607 => ObjectKind::Trigger,
        // Core 2.2 hazard IDs (spikes, saws, flames, large spikes). Sawblade
        // outer rings (1705/1706/1707) are visually concentric with their
        // 678/679/680 hazard cores but extend slightly further; they kill on
        // contact too, so they're hazards, not solids the player can stand on.
        8 | 39 | 103 | 177 | 178 | 179 | 184 | 185 | 186 | 216 | 217 | 218 | 219 | 220 | 392
        | 421 | 446 | 447 | 458 | 459 | 460 | 461 | 577 | 678 | 679 | 680 | 1705 | 1706 | 1707
        | 1715 | 1717 | 1720 | 1722 => ObjectKind::Hazard,
        _ => match hitbox {
            Some(HitboxData::Slope { .. }) => ObjectKind::Slope,
            Some(_) => ObjectKind::Solid,
            None => ObjectKind::Decoration,
        },
    }
}

fn opengd_fallback_hitbox(object_id: u32) -> Option<HitboxData> {
    // OpenGD LongData.cpp has `_pHitboxes` for some gameplay solids that are
    // missing from gdclone's object.json hitbox table.
    match object_id {
        // `lightsquare_01_01_001` (`LongData.cpp`: `{30, 30, -15, -15}`).
        207 => Some(HitboxData::Box {
            offset: [0.0, 0.0],
            half_extents: [15.0, 15.0],
        }),
        // `blockOutline_02_001` (`LongData.cpp`: `{1.5, 30, -15, -0.75}`).
        468 => Some(HitboxData::Box {
            offset: [0.0, 0.0],
            half_extents: [15.0, 0.75],
        }),
        // `h-square` (ceiling-safe block in Pop)
        1859 => Some(HitboxData::Box {
            offset: [0.0, 0.0],
            half_extents: [15.0, 15.0],
        }),
        // `s-block` (stops dash/normal movement in Pop)
        1829 => Some(HitboxData::Box {
            offset: [0.0, 0.0],
            half_extents: [15.0, 15.0],
        }),
        _ => None,
    }
}

fn parse_pairs(segment: &str) -> SimResult<HashMap<String, String>> {
    let tokens = segment.split(',').collect::<Vec<_>>();
    if tokens.len() % 2 != 0 {
        return Err(SimError::LevelParse(format!(
            "odd number of key/value tokens in segment {segment:?}"
        )));
    }

    let mut pairs = HashMap::with_capacity(tokens.len() / 2);
    for pair in tokens.chunks_exact(2) {
        pairs.insert(pair[0].to_owned(), pair[1].to_owned());
    }
    Ok(pairs)
}

fn number_prop(raw: &HashMap<String, String>, key: &str, fallback: f32) -> SimResult<f32> {
    raw.get(key)
        .map(|value| {
            value
                .parse::<f32>()
                .map_err(|error| SimError::LevelParse(error.to_string()))
        })
        .unwrap_or(Ok(fallback))
}

fn bool_prop(raw: &HashMap<String, String>, key: &str) -> SimResult<bool> {
    Ok(raw
        .get(key)
        .map(|value| matches!(value.as_str(), "1" | "true" | "True"))
        .unwrap_or(false))
}

fn groups_prop(value: Option<&String>) -> Vec<u32> {
    value
        .map(|value| {
            value
                .split('.')
                .filter_map(|group| group.parse::<u32>().ok())
                .collect()
        })
        .unwrap_or_default()
}
