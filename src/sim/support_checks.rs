// ---------- Unsupported-feature gating ----------

fn reject_unsupported(level: &Level) -> SimResult<()> {
    // `kA2` in GD save format is the level's *starting gamemode*, not
    // the platformer flag. Platformer is `kA17`. Reading `kA2 == "1"`
    // as "platformer" was wrong; for any level that started in ship
    // mode (kA2,1) it would also have rejected the load, but in the
    // current code path we never even hit this with vierre because
    // its kA2 is 0 (cube start) - the bug it was masking is that
    // `apply_starting_header_state` was never reading kA2 to set the
    // initial gamemode at all.
    if let Some(platformer_flag) = level.header.get("kA17") {
        if platformer_flag == "1" {
            return Err(SimError::UnsupportedFeature {
                feature: "platformer mode".to_owned(),
                object_id: 0,
            });
        }
    }
    // Note: GD has a 2-player split-controls flag in the level header but the
    // exact `kA*` index is not documented in any of our reference repos
    // (gdclone/gdp/G.js). We do not gate on it for now; if a level with split
    // controls is run, the partner physics will simply track the same input as
    // the primary. Treat that as a parity risk to revisit when we have a known
    // 2-player save to test against.
    let mut warned: std::collections::HashSet<u32> = std::collections::HashSet::new();
    for object in &level.objects {
        if is_force_block(object.object_id) {
            return Err(SimError::UnsupportedFeature {
                feature: "force blocks".to_owned(),
                object_id: object.object_id,
            });
        }
        match object.kind {
            // Mirror portals: no gameplay effect, accepted silently.
            ObjectKind::MirrorPortal => {}
            ObjectKind::Trigger if !is_supported_trigger(object.object_id) => {
                // Triggers that mutate the level (move/toggle/rotate/etc.)
                // are not yet ported. Previously this hard-errored, which
                // makes it impossible to play `pop.bin` against any modern
                // re-saved Pop level (which now contains a stray `move`
                // trigger far off the cube's path). Treat them as no-ops
                // and warn once per id so we still surface what's missing
                // without blocking the run.
                if warned.insert(object.object_id) {
                    eprintln!(
                        "warning: unsupported {} (id {}) at ({:.1}, {:.1}) - ignored, physics parity may drift if this trigger is touched",
                        trigger_name(object.object_id),
                        object.object_id,
                        object.x,
                        object.y,
                    );
                }
            }
            _ => {}
        }
    }
    Ok(())
}

fn is_force_block(object_id: u32) -> bool {
    // GD 2.2 force block IDs observed in level data. Until the value,
    // ForceID, range, and relative properties are mapped, fail explicitly.
    matches!(object_id, 2069 | 3645)
}

fn is_supported_trigger(object_id: u32) -> bool {
    // End trigger is the only trigger with a simulation-relevant effect we
    // currently implement. Visual-only triggers (color, alpha, shake, pulse)
    // don't affect death prediction so we accept them silently.
    matches!(
        object_id,
        3607 | 1007 | 1006 | 1520 | 29 | 30 | 105 | 221 | 717 | 718 | 743 | 744 | 899
    )
}

fn trigger_name(object_id: u32) -> &'static str {
    match object_id {
        901 => "move trigger",
        1049 => "toggle trigger",
        1268 => "spawn trigger",
        1346 => "rotate trigger",
        1347 => "follow trigger",
        1611 => "count trigger",
        1616 => "stop trigger",
        1811 => "instant-count trigger",
        1815 => "collision trigger",
        1817 => "pickup trigger",
        _ => "trigger",
    }
}

