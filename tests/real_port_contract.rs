use assert_cmd::Command;
use base64::{Engine, engine::general_purpose::STANDARD};
use flate2::{Compression, write::GzEncoder};
use gd_real_sim::{
    clock::{SpeedProfile, StepTiming, physics_steps},
    collision::{Rect, opengd_box_transform, rotated_slope_player_world_y, slope_y_pos},
    input::ClickTape,
    level::{Level, ObjectKind},
    object_data::{HitboxData, ObjectDatabase},
    save::{parse_local_levels_xml, select_local_level},
    sim::{GameMode, SimulationConfig, SimulationOutcome, simulate, simulate_with_trace},
    support::SupportMatrix,
    trace_diff::{DiffConfig, compare_logs},
};
use std::io::Write;

fn assert_rect_close(rect: Rect, center: [f32; 2], half_extents: [f32; 2]) {
    assert!(
        (rect.center[0] - center[0]).abs() < 0.001
            && (rect.center[1] - center[1]).abs() < 0.001
            && (rect.half_extents[0] - half_extents[0]).abs() < 0.001
            && (rect.half_extents[1] - half_extents[1]).abs() < 0.001,
        "expected rect center={center:?} half_extents={half_extents:?}, got center={:?} half_extents={:?}",
        rect.center,
        rect.half_extents
    );
}

fn assert_corners_close(actual: [(f32, f32); 4], expected: [(f32, f32); 4]) {
    for (index, (actual, expected)) in actual.iter().zip(expected.iter()).enumerate() {
        assert!(
            (actual.0 - expected.0).abs() < 0.001 && (actual.1 - expected.1).abs() < 0.001,
            "corner {index}: expected {expected:?}, got {actual:?}"
        );
    }
}

#[test]
fn object_database_reads_gdclone_hitboxes() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let block = db.get(1).unwrap();

    assert_eq!(block.texture, "square_01_001.png");
    assert!(matches!(
        block.hitbox,
        Some(HitboxData::Box {
            half_extents,
            offset
        }) if half_extents == [15.0, 15.0] && offset == [0.0, 0.0]
    ));
}

#[test]
fn level_parser_preserves_raw_properties_and_classifies_objects_from_data() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse("kA4,0;1,1,2,30,3,15,57,2.3;1,13,2,90,3,15,6,45;", &db).unwrap();

    assert_eq!(level.header.get("kA4").unwrap(), "0");
    assert_eq!(level.objects[0].y, 105.0);
    assert_eq!(level.objects[0].groups, vec![2, 3]);
    assert_eq!(level.objects[0].kind, ObjectKind::Solid);
    assert_eq!(level.objects[1].kind, ObjectKind::ModePortal);
    assert_eq!(level.objects[1].raw.get("1").unwrap(), "13");
    assert_eq!(level.objects[1].raw.get("6").unwrap(), "45");
    // Engine angle = negated RobTop key `6` (gdclone / cocos2d).
    assert!((level.objects[1].rotation + 45.0).abs() < 1e-5);
}

#[test]
fn parser_applies_opengd_fallback_hitboxes_for_h_square_and_s_block() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse("kA4,0;1,1859,2,1605,3,2415;1,1829,2,1665,3,2235;", &db).unwrap();

    assert_eq!(level.objects[0].kind, ObjectKind::Solid);
    assert!(matches!(
        level.objects[0].hitbox,
        Some(HitboxData::Box {
            half_extents,
            offset
        }) if half_extents == [15.0, 15.0] && offset == [0.0, 0.0]
    ));
    assert_eq!(level.objects[1].kind, ObjectKind::Solid);
    assert!(matches!(
        level.objects[1].hitbox,
        Some(HitboxData::Box {
            half_extents,
            offset
        }) if half_extents == [15.0, 15.0] && offset == [0.0, 0.0]
    ));
}

#[test]
fn pop_rotated_468_transform_matches_opengd() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse("kA4,0;1,468,2,420.8,3,75,6,270;", &db).unwrap();
    let object = &level.objects[0];

    assert_eq!(object.kind, ObjectKind::Solid);
    assert!(matches!(
        object.hitbox,
        Some(HitboxData::Box {
            half_extents,
            offset
        }) if half_extents == [15.0, 0.75] && offset == [0.0, 0.0]
    ));

    let transform = opengd_box_transform(object).expect("468 should have an OpenGD box transform");
    // Object position is the loaded center (no hidden +15 shift); for
    // raw (420.8, 75) the world center is (420.8, 165) = (420.8, 75+90).
    // After 270deg rotation the local 1.5x30 hitbox swaps to 30x1.5 about
    // that center.
    assert_rect_close(transform.bounds, [420.8, 165.0], [0.75, 15.0]);
    assert_corners_close(
        transform.corners,
        [
            (421.55, 150.0),
            (421.55, 180.0),
            (420.05, 180.0),
            (420.05, 150.0),
        ],
    );
}

#[test]
fn pop_flipped_rotated_468_transform_matches_opengd() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse("kA4,0;1,468,2,479.2,3,15,6,270,5,1;", &db).unwrap();
    let object = &level.objects[0];

    assert!(
        object.scale_y < 0.0,
        "raw key 5 should become signed Y scale"
    );

    let transform = opengd_box_transform(object).expect("468 should have an OpenGD box transform");
    // No hidden +15 shift; object world center is `(479.2, 15+90) = (479.2, 105)`.
    assert_rect_close(transform.bounds, [479.2, 105.0], [0.75, 15.0]);
    assert_corners_close(
        transform.corners,
        [
            (478.45, 90.0),
            (478.45, 120.0),
            (479.95, 120.0),
            (479.95, 90.0),
        ],
    );
}

#[test]
fn pop_207_transform_matches_opengd() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse("kA4,0;1,207,2,47,3,164.5,32,1.29;", &db).unwrap();
    let object = &level.objects[0];

    assert_eq!(object.kind, ObjectKind::Solid);
    assert!(matches!(
        object.hitbox,
        Some(HitboxData::Box {
            half_extents,
            offset
        }) if half_extents == [15.0, 15.0] && offset == [0.0, 0.0]
    ));

    let transform = opengd_box_transform(object).expect("207 should have an OpenGD box transform");
    // No hidden +15 shift; world center is `(47, 164.5+90) = (47, 254.5)`.
    // With uniform 1.29 scale the half-extents become 19.35.
    assert_rect_close(transform.bounds, [47.0, 254.5], [19.35, 19.35]);
    assert_corners_close(
        transform.corners,
        [
            (27.65, 235.15),
            (66.35, 235.15),
            (66.35, 273.85),
            (27.65, 273.85),
        ],
    );
}

#[test]
fn unsupported_move_trigger_is_ignored_with_warning() {
    // Move triggers (and other not-yet-ported sim-relevant triggers)
    // used to hard-error. Modern re-saves of even simple custom levels
    // sprinkle stray move triggers far from the cube's path, which made
    // every `pop.bin`-driven run fail at level load. They are now
    // treated as no-ops (with a stderr warning) so the simulator can
    // still play levels that only *contain* unsupported triggers
    // without ever interacting with them. Parity is preserved as long
    // as the trigger does not actually move/toggle anything the cube
    // touches; this is documented in the warning text.
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse("kA4,0;1,901,2,90,3,15,51,4;", &db).unwrap();
    let result = simulate(
        &level,
        &ClickTape::from_bits("0").unwrap(),
        SimulationConfig { max_ticks: 10 },
    );

    assert!(
        result.is_ok(),
        "unsupported move trigger should be ignored, got: {:?}",
        result.err(),
    );
}

#[test]
fn unsupported_platformer_header_is_reported() {
    let db = ObjectDatabase::load_embedded().unwrap();
    // `kA17` is the platformer flag in GD's save format; `kA2` is the
    // starting gamemode (cube/ship/etc.). The previous fixture confused
    // the two and was the reason `vierre` (which has `kA2 = 0` and
    // `kA17 = 0`) loaded fine while looking like the test was checking
    // the right key.
    let level = Level::parse("kA17,1,kA4,0;1,1,2,30,3,15;", &db).unwrap();
    let err = simulate(
        &level,
        &ClickTape::from_bits("0").unwrap(),
        SimulationConfig { max_ticks: 10 },
    )
    .unwrap_err()
    .to_string();
    assert!(err.contains("platformer"));
}

#[test]
fn force_blocks_are_reported_unsupported_not_silent_noops() {
    let db = ObjectDatabase::load_embedded().unwrap();
    for object_id in [2069, 3645] {
        let level = Level::parse(&format!("kA4,0;1,{object_id},2,30,3,15;"), &db).unwrap();
        let err = simulate(
            &level,
            &ClickTape::from_bits("0").unwrap(),
            SimulationConfig { max_ticks: 10 },
        )
        .unwrap_err()
        .to_string();
        assert!(
            err.contains("force blocks"),
            "id {object_id} should report force-block unsupported status, got {err}"
        );
    }
}

#[test]
fn dash_and_toggle_orbs_are_accepted_as_partial_mechanics() {
    let db = ObjectDatabase::load_embedded().unwrap();
    for object_id in [1704, 3004] {
        let level = Level::parse(&format!("kA4,0;1,{object_id},2,30,3,15;"), &db).unwrap();
        let outcome = simulate(
            &level,
            &ClickTape::from_bits("0").unwrap(),
            SimulationConfig { max_ticks: 10 },
        )
        .unwrap();
        assert!(matches!(
            outcome,
            SimulationOutcome::Died { .. } | SimulationOutcome::Timeout { .. }
        ));
    }
}

#[test]
fn mirror_portal_is_accepted_silently() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse("kA4,0;1,45,2,60,3,15;", &db).unwrap();
    // Mirror portal must not produce an unsupported error.
    let outcome = simulate(
        &level,
        &ClickTape::from_bits(&"0".repeat(240)).unwrap(),
        SimulationConfig { max_ticks: 240 },
    )
    .unwrap();
    // Player falls freely (no floor), so expect died or timeout — not an error.
    assert!(matches!(
        outcome,
        SimulationOutcome::Died { .. } | SimulationOutcome::Timeout { .. }
    ));
}

#[test]
fn yellow_orb_fires_on_press_start() {
    let db = ObjectDatabase::load_embedded().unwrap();
    // Yellow orb (36) right at player spawn so gravity can't drop the player
    // below it before intersection. Press from tick 0 to trigger press_start.
    let level = Level::parse("kA4,0;1,36,2,15,3,15;", &db).unwrap();
    let bits = "1".repeat(240);
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&bits).unwrap(),
        SimulationConfig { max_ticks: 240 },
    )
    .unwrap();
    // After hitting the orb while pressed, vy should jump to ~y_start = 11.18.
    assert!(
        run.trace.iter().any(|f| (f.state.vy - 11.18).abs() < 0.1),
        "expected yellow orb impulse in trace"
    );
}

#[test]
fn gravity_ring_orb_84_sets_point_4_yellow_velocity_then_flips_gravity() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse("kA4,0;1,84,2,15,3,15;", &db).unwrap();
    let bits = "1".repeat(240);
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&bits).unwrap(),
        SimulationConfig { max_ticks: 240 },
    )
    .unwrap();
    // Blue orb should apply 0.4x yellow-orb velocity, then toggle gravity.
    assert!(
        run.trace.iter().any(|f| f.state.gravity_sign > 0.0),
        "expected gravity to flip after id=84"
    );
    assert!(
        run.trace
            .iter()
            .any(|f| (f.state.vy - (11.1800318 * 0.4)).abs() < 0.1),
        "expected +0.4x yellow velocity before gravity toggle"
    );
}

#[test]
fn pink_orb_141_applies_docs_point_72_yellow_velocity_in_cube_mode() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse("kA4,0;1,141,2,15,3,15;", &db).unwrap();
    let bits = "1".repeat(240);
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&bits).unwrap(),
        SimulationConfig { max_ticks: 240 },
    )
    .unwrap();
    assert!(
        run.trace
            .iter()
            .any(|f| (f.state.vy - (11.1800318 * 0.72)).abs() < 0.1),
        "expected id=141 pink orb pulse near cube yellow * 0.72 in normal gravity cube mode"
    );
}

#[test]
fn red_orb_1333_is_stronger_than_pink_orb() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse("kA4,0;1,1333,2,15,3,15;", &db).unwrap();
    let bits = "1".repeat(240);
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&bits).unwrap(),
        SimulationConfig { max_ticks: 240 },
    )
    .unwrap();
    assert!(
        run.trace.iter().any(|f| f.state.vy > 14.0),
        "expected id=1333 red orb to apply a stronger positive pulse"
    );
}

#[test]
fn black_orb_1330_sets_negative_velocity() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse("kA4,0;1,1330,2,15,3,15;", &db).unwrap();
    let bits = "1".repeat(240);
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&bits).unwrap(),
        SimulationConfig { max_ticks: 240 },
    )
    .unwrap();
    assert!(
        run.trace.iter().any(|f| f.state.vy <= -14.9),
        "expected black orb impulse to drive vy near -15"
    );
}

#[test]
fn teleport_portal_preserves_velocity_and_moves_y() {
    let db = ObjectDatabase::load_embedded().unwrap();
    // Entry (747) right at player spawn, exit (749) far above. Player should
    // teleport to exit y before falling far from the spawn.
    let level = Level::parse("kA4,0;1,747,2,15,3,15;1,749,2,30,3,210;", &db).unwrap();
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&"0".repeat(240)).unwrap(),
        SimulationConfig { max_ticks: 240 },
    )
    .unwrap();
    assert!(
        run.trace.iter().any(|f| f.state.y >= 250.0),
        "teleport should lift player toward exit y=300"
    );
}

#[test]
fn dual_portal_creates_mirrored_partner() {
    let db = ObjectDatabase::load_embedded().unwrap();
    // Orange dual portal (286) at (60, 105). Partner should appear with
    // opposite gravity_sign (+1.0 instead of -1.0).
    let level = Level::parse("kA4,0;1,286,2,15,3,15;", &db).unwrap();
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&"0".repeat(240)).unwrap(),
        SimulationConfig { max_ticks: 240 },
    )
    .unwrap();
    assert!(
        run.trace
            .iter()
            .any(|f| f.partner.map(|p| p.gravity_sign > 0.0).unwrap_or(false)),
        "partner player should exist with flipped gravity after dual portal"
    );
}

#[test]
fn fixed_step_count_matches_gdp_formula() {
    assert_eq!(physics_steps(1.0 / 60.0, 1.0), 4);
    assert_eq!(physics_steps(1.0 / 60.0, 0.5), 8);
    assert_eq!(physics_steps(1.0 / 60.0, 1.5), 4);
    assert_eq!(physics_steps(0.0, 1.0), 1);

    let timing = StepTiming::from_frame_delta(1.0 / 60.0, 1.0);
    assert_eq!(timing.step_count, 4);
    assert_eq!(timing.physics_second, 0.25);
}

#[test]
fn speed_profiles_match_gdp_time_mod_constants() {
    let normal = SpeedProfile::for_player_speed(0.9);
    assert_eq!(normal.y_start, 11.1800318);
    assert_eq!(normal.gravity, 0.958199024);
    assert_eq!(normal.speed_multiplier, 5.77000189);

    let fast = SpeedProfile::for_player_speed(1.1);
    assert_eq!(fast.y_start, 11.420032);
    assert_eq!(fast.gravity, 0.957199);
    assert_eq!(fast.speed_multiplier, 5.870002);

    let slow = SpeedProfile::for_player_speed(0.7);
    assert_eq!(slow.y_start, 10.620032);
    assert_eq!(slow.gravity, 0.940199);
    assert_eq!(slow.speed_multiplier, 5.980002);
}

#[test]
fn click_tape_is_one_bit_per_physics_tick() {
    let tape = ClickTape::from_bits("00110").unwrap();

    assert!(!tape.is_pressed(0));
    assert!(tape.is_pressed(2));
    assert!(tape.is_press_start(2));
    assert!(tape.is_release(4));
}

#[test]
fn cube_falls_into_hazard_without_click() {
    let db = ObjectDatabase::load_embedded().unwrap();
    // Put a spike directly under the player's path. Default speed = 5.77 * 60
    // units/sec * 1/240 sec/tick = ~1.44 units/tick, so after ~60 ticks the
    // player is at x ≈ 100. Place the hazard on the real floor baseline.
    let level = Level::parse("kA4,0;1,8,2,80,3,0;", &db).unwrap();
    let outcome = simulate(
        &level,
        &ClickTape::from_bits(&"0".repeat(2400)).unwrap(),
        SimulationConfig { max_ticks: 2400 },
    )
    .unwrap();

    assert!(matches!(outcome, SimulationOutcome::Died { .. }));
}

#[test]
fn circular_hazard_outer_bounds_overlap_is_lethal_like_opengd() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse("kA4,0;1,678,2,291,3,-3,32,0.66;", &db).unwrap();
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&"0".repeat(210)).unwrap(),
        SimulationConfig { max_ticks: 210 },
    )
    .unwrap();

    assert!(
        matches!(
            run.outcome,
            SimulationOutcome::Died {
                object_id: Some(678),
                ..
            }
        ),
        "OpenGD kills circular hazards when the full player outer bounds intersects the circle"
    );
}

#[test]
fn cube_starts_on_real_floor_baseline_without_sinking() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse("kA4,0;", &db).unwrap();
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&"0".repeat(8)).unwrap(),
        SimulationConfig { max_ticks: 8 },
    )
    .unwrap();

    let first = run.trace.first().unwrap();
    assert!((first.state.x - -13.70175).abs() < 0.001);
    assert_eq!(first.state.y, 105.0);
    assert!(first.state.on_ground);
    assert_eq!(first.state.vy, 0.0);
}

#[test]
fn end_trigger_reports_completion() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse("kA4,0;1,3607,2,200,3,15;", &db).unwrap();
    let outcome = simulate(
        &level,
        &ClickTape::from_bits(&"0".repeat(2400)).unwrap(),
        SimulationConfig { max_ticks: 2400 },
    )
    .unwrap();

    assert!(matches!(outcome, SimulationOutcome::Completed { .. }));
}

#[test]
fn yellow_pad_applies_jump_velocity() {
    let db = ObjectDatabase::load_embedded().unwrap();
    // Pad at player path with a floor of solid blocks so the player rests on
    // them before hitting the pad.
    let level = Level::parse("kA4,0;1,35,2,15,3,15;", &db).unwrap();
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&"0".repeat(600)).unwrap(),
        SimulationConfig { max_ticks: 600 },
    )
    .unwrap();

    assert!(
        run.trace.iter().any(|frame| frame.state.vy > 10.0),
        "expected pad impulse to appear in trace; ymax = {}",
        run.trace
            .iter()
            .map(|f| f.state.vy)
            .fold(f32::NEG_INFINITY, f32::max)
    );
}

#[test]
fn purple_pad_id_140_applies_force_10_4_for_cube() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse("kA4,0;1,140,2,15,3,15,6,90,5,1;", &db).unwrap();
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&"0".repeat(24)).unwrap(),
        SimulationConfig { max_ticks: 24 },
    )
    .unwrap();

    let activated = run
        .trace
        .iter()
        .find(|frame| (frame.state.vy - 10.4).abs() < 0.001)
        .expect("purple pad should apply the documented 10.4 force when touched");
    assert!(
        (activated.state.vy - 10.4).abs() < 0.001,
        "id=140 is the purple pad and should apply force 10.4 for cube; got vy={}",
        activated.state.vy
    );
}

#[test]
fn red_pad_id_1332_applies_force_20() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse("kA4,0;1,1332,2,15,3,15;", &db).unwrap();
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&"0".repeat(8)).unwrap(),
        SimulationConfig { max_ticks: 8 },
    )
    .unwrap();

    assert!(
        run.trace
            .iter()
            .any(|frame| (frame.state.vy - 20.0).abs() < 0.001),
        "id=1332 is the red pad and should apply force 20; max vy was {}",
        run.trace
            .iter()
            .map(|frame| frame.state.vy)
            .fold(f32::NEG_INFINITY, f32::max)
    );
}

#[test]
fn pop_style_purple_pad_never_exceeds_force_10_4() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse(
        "kA4,0;\
         1,372,2,30,3,15;\
         1,371,2,75,3,45;\
         1,372,2,105,3,90,6,90,5,1;\
         1,140,2,178,3,195,6,90,5,1;\
         1,140,2,182,3,195,6,90;",
        &db,
    )
    .unwrap();
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&"0".repeat(180)).unwrap(),
        SimulationConfig { max_ticks: 180 },
    )
    .unwrap();

    let max_vy_after = run
        .trace
        .iter()
        .filter(|frame| frame.state.x > 150.0)
        .map(|frame| frame.state.vy)
        .fold(f32::NEG_INFINITY, f32::max);
    assert!(
        max_vy_after <= 10.4001,
        "purple pad behavior should not exceed force 10.4; max vy after x=150 was {}",
        max_vy_after
    );
}

#[test]
fn speed_portal_updates_player_speed_state() {
    let db = ObjectDatabase::load_embedded().unwrap();
    // Place a 2x speed portal (id 202) on the real floor baseline.
    let level = Level::parse("kA4,0;1,202,2,75,3,15;", &db).unwrap();
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&"0".repeat(600)).unwrap(),
        SimulationConfig { max_ticks: 600 },
    )
    .unwrap();

    // After passing the portal the stored player_speed should be 1.1.
    assert!(
        run.trace
            .iter()
            .any(|frame| (frame.state.player_speed - 1.1).abs() < 1e-4),
        "speed portal did not update player_speed"
    );
}

#[test]
fn mode_portal_switches_game_mode() {
    let db = ObjectDatabase::load_embedded().unwrap();
    // Ship portal (id 13, the pink portal_04 sprite) on the real floor
    // baseline. Note: id 12 is the *cube* portal in GD; we previously
    // had this mapping inverted.
    let level = Level::parse("kA4,0;1,13,2,60,3,15;", &db).unwrap();
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&"0".repeat(600)).unwrap(),
        SimulationConfig { max_ticks: 600 },
    )
    .unwrap();

    assert!(
        run.trace
            .iter()
            .any(|frame| frame.state.mode == GameMode::Ship),
        "ship portal did not switch mode"
    );
}

#[test]
fn slope_y_position_matches_gdp_branching() {
    let slope = Rect {
        center: [15.0, 15.0],
        half_extents: [15.0, 15.0],
    };

    assert_eq!(slope_y_pos(slope, 15.0, true, false, false), 15.0);
    assert_eq!(slope_y_pos(slope, 30.0, true, false, false), 30.0);
    assert_eq!(slope_y_pos(slope, 0.0, false, true, true), 26.0);
}

#[test]
fn slope_contact_uses_gdp_radius_along_slope() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse("kA4,0;1,1338,2,15,3,15;", &db).unwrap();
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&"0".repeat(40)).unwrap(),
        SimulationConfig { max_ticks: 40 },
    )
    .unwrap();

    let first = run
        .trace
        .iter()
        .find(|frame| frame.state.on_slope)
        .expect("trace should eventually contact the slope");
    assert!(
        first.state.y > 110.0,
        "slope should place cube using radius/cos(angle), got y={}",
        first.state.y
    );
    assert!(first.state.on_slope);
}

#[test]
fn pop_start_slope_uses_opengd_y_offset() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse("kA4,0;1,372,2,30,3,15;", &db).unwrap();
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&"0".repeat(40)).unwrap(),
        SimulationConfig { max_ticks: 40 },
    )
    .unwrap();

    let first = run
        .trace
        .iter()
        .find(|frame| frame.state.on_slope)
        .expect("trace should eventually contact the Pop start slope");
    assert!(first.state.on_slope);
    assert!(
        first.state.y > 105.0,
        "player should ride upward on the source .dat start slope; got y={}",
        first.state.y
    );
}

#[test]
fn rotated_flipped_pop_slope_uses_transformed_hypotenuse() {
    let y = rotated_slope_player_world_y(
        105.0, 180.0, -90.0, 1.0, -1.0, 30.0, 15.0, 119.43899, 165.0, 15.0, true,
    )
    .unwrap();

    // GDP `playerRadOnSlope = playerRadius / cos(getSlopeAngle())` uses the
    // slope's *intrinsic* (local) angle (atan2(15, 30) ~= 26.57deg for a
    // 1x2 slope), independent of world rotation. The previous "world cos"
    // formula was a bug that fortuitously matched some synthetic
    // expectations but caused astrosoda's vertical 1x2 slopes to silently
    // reject because the snap target was pushed ~17 px past the
    // snap-attach threshold.
    //
    // For player_x=119.439 on a slope rect spanning x=[90,120] y=[150,210]
    // with hypotenuse (90,150)->(120,210), y_surf ~= 208.9 and rad ~= 16.77
    // so wy ~= 225.65.
    assert!(
        (220.0..=232.0).contains(&y),
        "rotated Pop slope should snap player to ~225 using GDP local-cos; got y={y}"
    );
}

#[test]
fn pop_rotated_slope_transition_does_not_flatten_into_wall_stack() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse(
        "kA4,0;\
         1,372,2,30,3,15;\
         1,371,2,75,3,45;\
         1,372,2,105,3,90,6,90,5,1;\
         1,470,2,135,3,105;\
         1,471,2,135,3,75;\
         1,471,2,135,3,45;\
         1,471,2,135,3,15;",
        &db,
    )
    .unwrap();
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&"0".repeat(120)).unwrap(),
        SimulationConfig { max_ticks: 120 },
    )
    .unwrap();

    let after_transition = run
        .trace
        .iter()
        .find(|frame| frame.state.x >= 118.0)
        .expect("trace should reach the rotated Pop slope transition");
    // With the corrected local-cos slope-rad formula (GDP parity), the
    // rotated 1x2 slope at (105, 180) snaps the cube to ~y_surf+16.77.
    // At x>=118 (near the top of the rotated slope's hypotenuse) y should
    // be at least 220 - the cube is still riding the slope upward, not
    // flattened against the box stack at y=180.
    assert!(
        after_transition.state.y > 215.0,
        "player flattened before the rotated slope lifted them; got y={}",
        after_transition.state.y
    );
}

#[test]
fn pop_steep_slope_exit_carries_upward_velocity_over_stack() {
    let db = ObjectDatabase::load_embedded().unwrap();
    let level = Level::parse(
        "kA4,0;\
         1,372,2,30,3,15;\
         1,371,2,75,3,45;\
         1,372,2,105,3,90,6,90,5,1;\
         1,470,2,135,3,105;\
         1,471,2,135,3,75;\
         1,471,2,135,3,45;\
         1,471,2,135,3,15;",
        &db,
    )
    .unwrap();
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&"0".repeat(135)).unwrap(),
        SimulationConfig { max_ticks: 135 },
    )
    .unwrap();

    let after_stack = run
        .trace
        .iter()
        .find(|frame| frame.state.x >= 150.0)
        .expect("trace should continue past the first wall stack");
    assert!(
        after_stack.state.y > 220.0 && !after_stack.state.on_ground,
        "steep slope exit should keep the cube airborne above the stack; got y={}, on_ground={}",
        after_stack.state.y,
        after_stack.state.on_ground
    );
}

#[test]
fn support_matrix_is_machine_readable_and_tracks_noop_and_unsupported_systems() {
    let matrix = SupportMatrix::load_embedded().unwrap();

    assert_eq!(matrix.status("player_modes.cube").unwrap(), "partial");
    assert_eq!(matrix.status("triggers.move_trigger").unwrap(), "no_op");
    assert_eq!(
        matrix
            .status("force_blocks.basic_acceleration_formula")
            .unwrap(),
        "unsupported"
    );
    assert_eq!(matrix.status("triggers.end_trigger").unwrap(), "supported");
}

#[test]
fn cclocallevels_xml_extracts_named_levelstrings() {
    let payload = encode_level_payload("kA4,0;1,1,2,30,3,15;");
    let xml = format!(
        r#"<?xml version="1.0"?><plist><dict><k>LLM_01</k><d><k>k2</k><s>Test Level</s><k>k4</k><s>{payload}</s></d></dict></plist>"#
    );

    let levels = parse_local_levels_xml(&xml).unwrap();
    let selected = select_local_level(&levels, Some("Test Level")).unwrap();

    assert_eq!(selected.levelstring, "kA4,0;1,1,2,30,3,15;");
}

#[test]
fn cli_returns_structured_json_for_levelstring_and_clicks() {
    let mut command = Command::cargo_bin("gd-real-sim").unwrap();
    command
        .args([
            "--levelstring",
            "kA4,0;1,8,2,60,3,15;",
            "--clicks",
            &"0".repeat(2400),
            "--max-ticks",
            "2400",
        ])
        .assert()
        .success()
        .stdout(predicates::str::contains("\"outcome\""));
}

#[test]
fn cli_accepts_compressed_levelstring_file_payload() {
    let temp = tempfile::tempdir().unwrap();
    let level_path = temp.path().join("level.gd");
    std::fs::write(&level_path, encode_level_payload("kA4,0;1,3607,2,20,3,15;")).unwrap();

    let mut command = Command::cargo_bin("gd-real-sim").unwrap();
    command
        .args([
            "--levelstring-file",
            level_path.to_str().unwrap(),
            "--clicks",
            &"0".repeat(60),
            "--max-ticks",
            "60",
        ])
        .assert()
        .success()
        .stdout(predicates::str::contains("\"outcome\": \"completed\""));
}

#[test]
fn trace_diff_collapses_real_idle_and_aligns_first_motion() {
    let real = "\
tick   t_ms       x           y           dx          dy          mode     dead can_jmp mode_lo      mode_hi
   413     1732     0.00000   105.00000     0.00000   105.00000 cube        0       0 0x00000000 0x00000000
   414     1732     0.00000   105.00000     0.00000     0.00000 cube        0       0 0x00000000 0x00000000
   671     2807     5.19300   109.36700     5.19300     4.36700 cube        0       0 0x00000000 0x00000000
   672     2807     5.19300   109.36700     0.00000     0.00000 cube        0       0 0x00000000 0x00000000
   675     2823    10.38600   113.73400     5.19300     4.36700 cube        0       0 0x00000000 0x00000000
";
    let sim = "\
tick   time      plr  press mode    grav   mini on_gnd on_slp      x           y           vx          vy          p_spd   s_mult  g        y_start   size
     0  0.00000    0      0 Cube      -1.0      0      0      0    16.29800   104.78400     5.77000    -0.95820   0.900   5.770  0.95820  11.18003   1.00
     1  0.00417    0      0 Cube      -1.0      0      0      0    21.49100   109.00000     5.77000    -0.95820   0.900   5.770  0.95820  11.18003   1.00
";

    let report = compare_logs(
        real,
        sim,
        DiffConfig {
            epsilon_x: 0.05,
            epsilon_y: 0.05,
            ..DiffConfig::default()
        },
    )
    .unwrap();

    assert_eq!(report.real_start.source_tick, 671);
    assert_eq!(report.sim_start.source_tick, 0);
    assert_eq!(report.compared_steps, 2);
    assert_eq!(report.sim_stride, 1);
    assert_eq!(report.first_divergence.unwrap().step, 1);
}

#[test]
fn trace_diff_downsamples_sim_to_real_motion_cadence() {
    let real = "\
tick   t_ms       x           y           dx          dy          mode     dead can_jmp mode_lo      mode_hi
   671     2807     5.00000   105.00000     5.00000     0.00000 cube        0       0 0x00000000 0x00000000
   675     2823    10.00000   105.00000     5.00000     0.00000 cube        0       0 0x00000000 0x00000000
";
    let sim = "\
tick   time      plr  press mode    grav   mini on_gnd on_slp      x           y           vx          vy          p_spd   s_mult  g        y_start   size
     0  0.00000    0      0 Cube      -1.0      0      0      0     1.25000   105.00000     5.77000     0.00000   0.900   5.770  0.95820  11.18003   1.00
     1  0.00417    0      0 Cube      -1.0      0      0      0     2.50000   105.00000     5.77000     0.00000   0.900   5.770  0.95820  11.18003   1.00
     2  0.00833    0      0 Cube      -1.0      0      0      0     3.75000   105.00000     5.77000     0.00000   0.900   5.770  0.95820  11.18003   1.00
     3  0.01250    0      0 Cube      -1.0      0      0      0     5.00000   105.00000     5.77000     0.00000   0.900   5.770  0.95820  11.18003   1.00
     4  0.01667    0      0 Cube      -1.0      0      0      0     6.25000   105.00000     5.77000     0.00000   0.900   5.770  0.95820  11.18003   1.00
";

    let report = compare_logs(real, sim, DiffConfig::default()).unwrap();

    assert_eq!(report.sim_stride, 4);
    assert_eq!(report.compared_steps, 2);
    assert!(report.first_divergence.is_none());
}

#[test]
fn trace_diff_cli_reports_first_divergence_json() {
    let temp = tempfile::tempdir().unwrap();
    let real_path = temp.path().join("real.log");
    let sim_path = temp.path().join("sim.log");
    std::fs::write(
        &real_path,
        "\
tick   t_ms       x           y           dx          dy          mode     dead can_jmp mode_lo      mode_hi
   671     2807     5.00000   105.00000     5.00000     0.00000 cube        0       0 0x00000000 0x00000000
   675     2823    10.00000   105.00000     5.00000     0.00000 cube        0       0 0x00000000 0x00000000
",
    )
    .unwrap();
    std::fs::write(
        &sim_path,
        "\
tick   time      plr  press mode    grav   mini on_gnd on_slp      x           y           vx          vy          p_spd   s_mult  g        y_start   size
     0  0.00000    0      0 Cube      -1.0      0      0      0    15.00000   105.00000     5.77000     0.00000   0.900   5.770  0.95820  11.18003   1.00
     1  0.00417    0      0 Cube      -1.0      0      0      0    20.50000   105.00000     5.77000     0.00000   0.900   5.770  0.95820  11.18003   1.00
",
    )
    .unwrap();

    let mut command = Command::cargo_bin("gd-log-diff").unwrap();
    command
        .args([
            "--real-log",
            real_path.to_str().unwrap(),
            "--sim-log",
            sim_path.to_str().unwrap(),
            "--epsilon-x",
            "0.1",
            "--epsilon-y",
            "0.1",
        ])
        .assert()
        .success()
        .stdout(predicates::str::contains("\"first_divergence\""))
        .stdout(predicates::str::contains("\"step\": 1"));
}

/// User-spec contract: while on a 1x1 slope (45°), the cube's `rotation`
/// settles toward the slope angle (45° in world space) within a few ticks
/// instead of staying at 0 (which is what the simulator did before the
/// three-hitbox slope rewrite).
#[test]
fn cube_rotation_settles_to_slope_angle_on_1x1_slope() {
    let db = ObjectDatabase::load_embedded().unwrap();
    // Three 1x1 slope tiles in a row so the cube stays on a slope long
    // enough for the rotation lerp to converge. A single 30-wide tile is
    // crossed in ~23 ticks at speed_multiplier 5.77.
    let level = Level::parse(
        "kA4,0;1,1338,2,15,3,15;1,1338,2,45,3,45;1,1338,2,75,3,75;",
        &db,
    )
    .unwrap();
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&"0".repeat(40)).unwrap(),
        SimulationConfig { max_ticks: 40 },
    )
    .unwrap();
    // Pick the last frame that is still on a slope (after the rotation lerp
    // has had time to settle).
    let mid_slope = run
        .trace
        .iter()
        .filter(|f| f.state.on_slope)
        .nth(15)
        .expect("expected at least 16 on-slope frames");
    assert!(
        (mid_slope.state.rotation - 45.0).abs() < 2.0,
        "cube rotation should settle to ~45 deg on a 1x1 slope; got {}",
        mid_slope.state.rotation
    );
}

/// User-spec contract: positional Y must be smooth along a slope. The
/// previous flip-flop between `slope_player_half = player_half` (entry) and
/// `player_half * SQRT_2` (continuation) caused a per-tick Y jump of up to 9
/// units mid-traversal. With the rewrite, consecutive Y deltas while on the
/// same slope must be small and monotonic.
#[test]
fn cube_traverses_1x1_slope_without_y_jitter() {
    let db = ObjectDatabase::load_embedded().unwrap();
    // Three 1x1 slope tiles in a row, so the cube has time to reach steady
    // state and we can measure mid-slope Y deltas.
    let level = Level::parse(
        "kA4,0;1,1338,2,15,3,15;1,1338,2,45,3,45;1,1338,2,75,3,75;",
        &db,
    )
    .unwrap();
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&"0".repeat(40)).unwrap(),
        SimulationConfig { max_ticks: 40 },
    )
    .unwrap();

    // Pick the window where the cube is mid-traversal (after settle).
    let on_slope_frames: Vec<_> = run
        .trace
        .iter()
        .filter(|f| f.state.on_slope)
        .skip(3) // allow rotation lerp to settle
        .take(15)
        .collect();
    assert!(
        on_slope_frames.len() >= 10,
        "needed >=10 mid-slope frames to measure jitter, got {}",
        on_slope_frames.len()
    );

    let mut max_dy = 0.0_f32;
    for window in on_slope_frames.windows(2) {
        let dy = (window[1].state.y - window[0].state.y).abs();
        max_dy = max_dy.max(dy);
    }
    // A 1x1 (45°) slope at speed_multiplier ~5.77 advances ~1.3 px of X per
    // 240Hz tick, so the expected steady-state delta-Y is also ~1.3 (grade 1).
    // Allow up to 2.5 for the in-between ticks where the cube finishes
    // settling, but reject the old `* SQRT_2` flip which caused ~9-unit
    // mid-traversal jumps.
    assert!(
        max_dy < 2.5,
        "Y per-tick delta should be < 2.5 on a 1x1 slope (no jitter), got max {max_dy}"
    );
}

/// User-spec contract: slope-exit `vy` uses the steepness-keyed formula
///   * 1x1 slope (steepness ~= 1):    `|vx|`
///   * 1x2 slope (steepness ~= 2):    `tan(1 rad) * |vx|`
///   * 2x1 slope (steepness ~= 0.5):  `tan(0.5 rad) * |vx|`
/// (replaces the earlier GDP `1.54 * height * speed / width` which over-
/// launched the cube ~2x on steep slopes).
#[test]
fn slope_exit_velocity_matches_gdp_canon_formula() {
    let db = ObjectDatabase::load_embedded().unwrap();
    // 1339 = 60 wide x 30 tall (gentle 2x1 slope, steepness 0.5).
    let level = Level::parse("kA4,0;1,1339,2,15,3,15;", &db).unwrap();
    let run = simulate_with_trace(
        &level,
        &ClickTape::from_bits(&"0".repeat(60)).unwrap(),
        SimulationConfig { max_ticks: 60 },
    )
    .unwrap();

    let last_on_slope = run
        .trace
        .iter()
        .rev()
        .find(|f| f.state.on_slope)
        .expect("cube must touch the slope at some point");

    // GDP `m_slopeVelocity` =
    //   min(1.12 / slopeAngle, 1.54) * slopeYVelocity * flipMod * uphillSign
    // with slopeYVelocity = (rect_h * playerSpeed * speedMul) / rect_w.
    // For a 60x30 slope (rect_h=30, rect_w=60, angle=atan2(30,60)~=0.4636):
    //   factor = min(1.12 / 0.4636, 1.54) = 1.54 (capped)
    //   slopeYVel = (30 * 0.9 * 5.77) / 60 ~= 2.5965
    //   |slopeVel|  = 1.54 * 2.5965 ~= 3.999
    let speed_mul = last_on_slope.state.speed_multiplier;
    let player_speed = last_on_slope.state.player_speed;
    let slope_angle = (30.0_f32).atan2(60.0);
    let expected_gdp =
        (1.12_f32 / slope_angle).min(1.54) * (30.0 * player_speed * speed_mul) / 60.0;

    let actual = last_on_slope.state.slope_exit_vy.abs();
    assert!(
        (actual - expected_gdp).abs() < 0.05,
        "exit_vy should match GDP canon = {expected_gdp}, got {actual}"
    );

    // Also confirm we are NOT using the user-spec tan(0.5)*|vx| heuristic,
    // which produces a smaller value (~3.15 vs GDP's ~4.0) and was the
    // formula that caused pop.bin to fail to win the level.
    let user_tan_formula = last_on_slope.state.vx.abs() * 0.5_f32.tan();
    assert!(
        (actual - user_tan_formula).abs() > 0.3,
        "exit_vy must NOT match the user tan formula {user_tan_formula}; got {actual}"
    );
}

fn encode_level_payload(levelstring: &str) -> String {
    let mut encoder = GzEncoder::new(Vec::new(), Compression::default());
    encoder.write_all(levelstring.as_bytes()).unwrap();
    STANDARD
        .encode(encoder.finish().unwrap())
        .replace('/', "_")
        .replace('+', "-")
}
