//! Simulation core.
//!
//! Physics constants and control-flow shape are ported from the `gdp`
//! decomp. Object IDs and trigger architecture come from `gdclone`.
//! Orb and pad velocity tables are from the GD Docs reference
//! (<https://boomlings.dev/reference/player_physics/orbs_and_pads>).
//!
//! The `G` notation in the docs is the gravity-direction scalar. Concretely
//! `xG` means `x * flip_mod` where `flip_mod = -player.gravity_sign`
//! (+1 in normal gravity, -1 when flipped). "Cube jump velocity" = `y_start`
//! at the active speed tier.

use std::collections::HashSet;

use serde::Serialize;

use crate::{
    SimError, SimResult,
    clock::SpeedProfile,
    collision::{
        Rect, near_slope_hypotenuse, opengd_box_transform, rotated_slope_player_world_y,
        slope_local_to_world_point, slope_world_to_local_point, transformed_slope_world_grade,
    },
    input::ClickTape,
    level::{Level, LevelObject, ObjectKind},
    object_data::HitboxData,
};

/// Fixed physics tick in seconds (240 Hz, matches `gdp` substep cadence).
pub const DT: f32 = 1.0 / 240.0;
/// Horizontal integration uses `Δx = vx * dt * 60 * player_speed`.
pub const TIME_TO_FRAMES: f32 = 60.0;
const SUBSTEP_TO_FRAME: f32 = DT * TIME_TO_FRAMES;
/// gdclone's vertical `slowed_delta = delta_seconds * 0.9` scaling.
pub const VERTICAL_SLOW: f32 = 0.9;
const FIRST_TRACE_X: f32 = -1.0;
const GROUND_PROBE_HEIGHT: f32 = 0.1;
const GROUND_PROBE_DISTANCE: f32 = 0.0;
const WORLD_UNITS_PER_BLOCK: f32 = 30.0;
const CUBE_AIR_SPIN_ROTATIONS_1X: f32 = 0.5;
const CUBE_AIR_SPIN_JUMP_DISTANCE_1X_BLOCKS: f32 = 3.5;
const DASH_SPIN_DISTANCE_BLOCKS: f32 = 3.8;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimulationConfig {
    pub max_ticks: usize,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            max_ticks: 240 * 60 * 10,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum GameMode {
    Cube,
    Ship,
    Ball,
    Ufo,
    Wave,
    Robot,
    Spider,
    Swing,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct PlayerState {
    pub x: f32,
    pub y: f32,
    pub vx: f32,
    pub vy: f32,
    pub mode: GameMode,
    /// -1 = normal (falls down), +1 = flipped (falls up).
    pub gravity_sign: f32,
    pub mini: bool,
    pub player_speed: f32,
    pub speed_multiplier: f32,
    pub gravity: f32,
    pub y_start: f32,
    pub vehicle_size: f32,
    pub on_ground: bool,
    pub was_jump_buffered: bool,
    pub jump_buffered: bool,
    pub state_ring_jump: bool,
    pub on_slope: bool,
    /// Latched exit velocity from the most recently ridden slope. Applied as
    /// `vy` on the first tick where the player is no longer touching that
    /// slope (gdp::collidedWithSlopeInternal lines 25/266).
    pub slope_exit_vy: f32,
    pub slope_exit_vx: f32,
    pub slope_contact_cooldown: u8,
    pub slope_object: Option<usize>,
    pub slope_is_current_top: bool,
    pub slope_prev_radius: f32,
    /// World-space cube rotation in degrees. While on a slope this lerps
    /// toward the slope's surface angle so the rotated outer hitbox sits flat
    /// against the hypotenuse; airborne it eases back toward 0. The
    /// non-rotated outer 30x30 still selects which slope is active.
    pub rotation: f32,
    /// GDP `m_isAccelerating`. Set true by `boostPlayer` (pads/orbs) and
    /// cleared on ground contact for non-flying modes. Affects airborne
    /// gravity branches in `update_cube` per `updateJump.cpp` lines
    /// 240-267 (e.g. accelerating-and-not-jump-buffered zeros the v51
    /// gravity term, so a freshly pad-boosted cube falls slower than a
    /// naturally-jumping cube).
    pub is_accelerating: bool,
    pub snapped_object: Option<usize>,
    pub snap_distance: f32,
    /// Remaining horizontal travel (world units) for dash-orb spin pacing.
    pub dash_rotation_blocks_remaining: f32,
    /// Active dash movement angle in world degrees (docs-clamped).
    pub dash_angle_deg: f32,
    /// Consecutive ticks the jump input has been held. Used by 2-tick modes.
    pub hold_ticks: u8,
    /// A one-tick delayed vertical velocity override used by documented pad
    /// follow-ups (ship/UFO yellow pad: 16G now, 8G next tick).
    pub pending_yvel_next_tick: f32,
}

impl PlayerState {
    fn flip_mod(&self) -> f32 {
        -self.gravity_sign
    }

    fn player_half(&self) -> f32 {
        if self.mini { 9.0 } else { 15.0 }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct TraceFrame {
    pub tick: usize,
    pub time: f32,
    pub pressed: bool,
    pub state: PlayerState,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub partner: Option<PlayerState>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(tag = "outcome", rename_all = "snake_case")]
pub enum SimulationOutcome {
    Completed {
        tick: usize,
        time: f32,
        state: PlayerState,
    },
    Died {
        tick: usize,
        time: f32,
        state: PlayerState,
        object_id: Option<u32>,
        reason: String,
        which_player: u8,
    },
    Timeout {
        tick: usize,
        time: f32,
        state: PlayerState,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct SimulationRun {
    pub outcome: SimulationOutcome,
    pub trace: Vec<TraceFrame>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct LiveStep {
    pub frame: TraceFrame,
    pub outcome: Option<SimulationOutcome>,
}

pub struct LiveSimulationSession<'a> {
    level: &'a Level,
    player: PlayerState,
    partner: Option<PlayerState>,
    touched_pads: HashSet<usize>,
    touched_portals: HashSet<usize>,
    touched_orbs: HashSet<(usize, u8)>,
    touched_teleports: HashSet<usize>,
    teleport_exits: Vec<&'a LevelObject>,
    tick: usize,
    click_tick: usize,
}

fn starting_mode_from_header(level: &Level) -> GameMode {
    match level
        .header
        .get("kA2")
        .and_then(|value| value.parse::<u8>().ok())
    {
        Some(1) => GameMode::Ship,
        Some(2) => GameMode::Ball,
        Some(3) => GameMode::Ufo,
        Some(4) => GameMode::Wave,
        Some(5) => GameMode::Robot,
        Some(6) => GameMode::Spider,
        Some(7) => GameMode::Swing,
        _ => GameMode::Cube,
    }
}

fn starting_player_speed_from_header(level: &Level) -> f32 {
    match level
        .header
        .get("kA4")
        .and_then(|value| value.parse::<u8>().ok())
    {
        Some(1) => crate::consts::PLAYER_SPEED_0_5X,
        Some(2) => crate::consts::PLAYER_SPEED_2X,
        Some(3) => crate::consts::PLAYER_SPEED_3X,
        Some(4) => crate::consts::PLAYER_SPEED_4X,
        _ => crate::consts::PLAYER_SPEED_1X,
    }
}

pub fn simulate(
    level: &Level,
    clicks: &ClickTape,
    config: SimulationConfig,
) -> SimResult<SimulationOutcome> {
    simulate_with_trace(level, clicks, config).map(|run| run.outcome)
}

pub fn simulate_with_trace(
    level: &Level,
    clicks: &ClickTape,
    config: SimulationConfig,
) -> SimResult<SimulationRun> {
    let mut session = LiveSimulationSession::new(level)?;
    let mut trace: Vec<TraceFrame> = Vec::new();
    for _ in 0..config.max_ticks {
        let step = session.step_tape(clicks)?;
        let outcome = step.outcome.clone();
        trace.push(step.frame);
        if let Some(outcome) = outcome {
            return Ok(SimulationRun { outcome, trace });
        }
    }

    let outcome = SimulationOutcome::Timeout {
        tick: config.max_ticks,
        time: config.max_ticks as f32 / 240.0,
        state: session.player_state(),
    };
    Ok(SimulationRun { outcome, trace })
}

impl<'a> LiveSimulationSession<'a> {
    pub fn new(level: &'a Level) -> SimResult<Self> {
        reject_unsupported(level)?;

        let starting_player_speed = starting_player_speed_from_header(level);
        let speed_profile = SpeedProfile::for_player_speed(starting_player_speed);
        let first_step_dx =
            speed_profile.speed_multiplier * DT * TIME_TO_FRAMES * starting_player_speed;
        let player = PlayerState {
            x: FIRST_TRACE_X - first_step_dx,
            y: 105.0,
            vx: speed_profile.speed_multiplier,
            vy: 0.0,
            mode: starting_mode_from_header(level),
            gravity_sign: -1.0,
            mini: false,
            player_speed: starting_player_speed,
            speed_multiplier: speed_profile.speed_multiplier,
            gravity: speed_profile.gravity,
            y_start: speed_profile.y_start,
            vehicle_size: 1.0,
            on_ground: true,
            was_jump_buffered: false,
            jump_buffered: false,
            state_ring_jump: false,
            on_slope: false,
            slope_exit_vy: 0.0,
            slope_exit_vx: 0.0,
            slope_contact_cooldown: 1,
            slope_object: None,
            slope_is_current_top: false,
            slope_prev_radius: 15.0,
            rotation: 0.0,
            is_accelerating: false,
            snapped_object: None,
            snap_distance: 0.0,
            dash_rotation_blocks_remaining: 0.0,
            dash_angle_deg: 0.0,
            hold_ticks: 0,
            pending_yvel_next_tick: 0.0,
        };

        let teleport_exits = level
            .objects
            .iter()
            .filter(|o| o.object_id == 749)
            .collect();

        Ok(Self {
            level,
            player,
            partner: None,
            touched_pads: HashSet::new(),
            touched_portals: HashSet::new(),
            touched_orbs: HashSet::new(),
            touched_teleports: HashSet::new(),
            teleport_exits,
            tick: 0,
            click_tick: 0,
        })
    }

    pub fn player_state(&self) -> PlayerState {
        self.player
    }

    pub fn tick(&self) -> usize {
        self.tick
    }

    pub fn step_live(&mut self, held: bool) -> SimResult<LiveStep> {
        self.step_resolved(held)
    }

    pub fn step_tape(&mut self, clicks: &ClickTape) -> SimResult<LiveStep> {
        let pressed = clicks.is_pressed(self.click_tick);
        self.click_tick += 1;
        self.step_resolved(pressed)
    }

    fn step_resolved(&mut self, pressed: bool) -> SimResult<LiveStep> {
        let tick = self.tick;
        refresh_ground_probe(self.level, &mut self.player);
        step_player(&mut self.player, pressed);
        if let Some(p2) = self.partner.as_mut() {
            refresh_ground_probe(self.level, p2);
            step_player(p2, pressed);
        }

        let old_x = self.player.x;
        integrate_player_position(&mut self.player);
        if let Some(p2) = self.partner.as_mut() {
            integrate_player_position(p2);
        }

        let mut dual_activate = false;
        let mut dual_deactivate = false;
        apply_portals(
            self.level,
            &mut self.player,
            &mut self.touched_portals,
            &self.teleport_exits,
            &mut self.touched_teleports,
            &mut dual_activate,
            &mut dual_deactivate,
            0,
        );
        if let Some(p2) = self.partner.as_mut() {
            apply_portals(
                self.level,
                p2,
                &mut self.touched_portals,
                &self.teleport_exits,
                &mut self.touched_teleports,
                &mut dual_activate,
                &mut dual_deactivate,
                1,
            );
        }
        if dual_activate && self.partner.is_none() {
            self.partner = Some(make_partner(&self.player));
        }
        if dual_deactivate {
            self.partner = None;
        }

        apply_blue_pads_pre_collision(self.level, &mut self.player, &mut self.touched_pads, 0);
        if let Some(p2) = self.partner.as_mut() {
            apply_blue_pads_pre_collision(self.level, p2, &mut self.touched_pads, 1);
        }

        let mut outcome = resolve_collisions(self.level, tick, &mut self.player, 0);
        if outcome.is_none()
            && let Some(p2) = self.partner.as_mut()
        {
            outcome = resolve_collisions(self.level, tick, p2, 1);
        }

        if outcome.is_none() {
            apply_pads(self.level, &mut self.player, &mut self.touched_pads, 0);
            apply_orbs(
                self.level,
                &mut self.player,
                &mut self.touched_orbs,
                0,
                pressed,
            );
            if let Some(p2) = self.partner.as_mut() {
                apply_pads(self.level, p2, &mut self.touched_pads, 1);
                apply_orbs(self.level, p2, &mut self.touched_orbs, 1, pressed);
            }

            if end_reached(self.level, old_x, self.player.x) {
                outcome = Some(SimulationOutcome::Completed {
                    tick,
                    time: tick as f32 / 240.0,
                    state: self.player,
                });
            }
        }

        let frame = make_trace_frame(tick, pressed, self.player, self.partner);
        self.tick += 1;
        Ok(LiveStep { frame, outcome })
    }
}

fn make_trace_frame(
    tick: usize,
    pressed: bool,
    player: PlayerState,
    partner: Option<PlayerState>,
) -> TraceFrame {
    TraceFrame {
        tick,
        time: tick as f32 / 240.0,
        pressed,
        state: player,
        partner,
    }
}

fn make_partner(primary: &PlayerState) -> PlayerState {
    let mut partner = *primary;
    partner.gravity_sign = -primary.gravity_sign;
    partner.vy = 0.0;
    partner.slope_exit_vy = 0.0;
    partner.slope_exit_vx = 0.0;
    partner.slope_contact_cooldown = 0;
    partner.slope_object = None;
    partner.slope_is_current_top = false;
    partner.slope_prev_radius = primary.slope_prev_radius;
    partner.rotation = 0.0;
    partner.snapped_object = None;
    partner.snap_distance = 0.0;
    partner.dash_rotation_blocks_remaining = 0.0;
    partner.dash_angle_deg = 0.0;
    partner.hold_ticks = 0;
    partner.pending_yvel_next_tick = 0.0;
    partner
}

fn integrate_player_position(player: &mut PlayerState) {
    if player.dash_rotation_blocks_remaining > 0.0 {
        let step_world = player.vx.abs() * DT * TIME_TO_FRAMES * player.player_speed;
        let travel = step_world.min(player.dash_rotation_blocks_remaining);
        let theta = player.dash_angle_deg.to_radians();
        player.x += travel * theta.cos();
        player.y += travel * theta.sin();
        player.vy = 0.0;
        player.dash_rotation_blocks_remaining =
            (player.dash_rotation_blocks_remaining - travel).max(0.0);
        return;
    }
    player.x += player.vx * DT * TIME_TO_FRAMES * player.player_speed;
    player.y += player.vy * DT * TIME_TO_FRAMES * VERTICAL_SLOW;
}

fn step_player(player: &mut PlayerState, pressed: bool) {
    let press_start = pressed && !player.jump_buffered;
    if pressed {
        player.hold_ticks = player.hold_ticks.saturating_add(1);
    } else {
        player.hold_ticks = 0;
    }
    if !pressed {
        // Ball click buffering is edge-triggered and consumed on landing.
        // Do not clear the queued air-click on key release.
        if player.mode != GameMode::Ball {
            player.state_ring_jump = false;
        }
        if player.dash_rotation_blocks_remaining > 0.0 {
            // Hold-to-dash behavior: release cancels active dash immediately.
            player.dash_rotation_blocks_remaining = 0.0;
        }
    } else if press_start {
        // OpenGD keeps `_queuedHold` for a press that starts while airborne,
        // allowing rings to fire when touched later in the same hold.
        player.state_ring_jump = !player.on_ground;
    }
    player.was_jump_buffered = player.jump_buffered;
    player.jump_buffered = pressed;
    apply_mode_physics(player, pressed);
    if player.pending_yvel_next_tick != 0.0 {
        player.vy = player.pending_yvel_next_tick;
        player.pending_yvel_next_tick = 0.0;
    }
    if player.dash_rotation_blocks_remaining > 0.0 {
        player.vy = 0.0;
    }
}

include!("support_checks.rs");

include!("mode_physics.rs");

include!("portals.rs");

include!("pads.rs");

include!("orbs.rs");

include!("ground_bounds.rs");

include!("collision_resolution.rs");

include!("geometry_helpers.rs");

// ---------- End / completion detection ----------

fn end_reached(level: &Level, old_x: f32, new_x: f32) -> bool {
    if level
        .objects
        .iter()
        .any(|object| object.object_id == 3607 && object.x > old_x && object.x <= new_x)
    {
        return true;
    }

    let Some(last_block_x) = last_gameplay_block_x(level) else {
        return false;
    };
    let finish_x = last_block_x + 60.0;
    finish_x > old_x && finish_x <= new_x
}

fn last_gameplay_block_x(level: &Level) -> Option<f32> {
    level
        .objects
        .iter()
        .filter(|object| {
            matches!(
                object.kind,
                ObjectKind::Solid | ObjectKind::Slope | ObjectKind::Hazard
            )
        })
        .map(|object| object.x)
        .max_by(|a, b| a.total_cmp(b))
}

#[cfg(test)]
mod tests {
    use std::collections::{HashMap, HashSet};

    use crate::level::{LevelObject, ObjectKind};

    use super::*;

    #[test]
    fn header_ka2_ship_start_sets_initial_mode_to_ship() {
        let mut header = HashMap::new();
        header.insert("kA2".to_owned(), "1".to_owned());
        let level = Level {
            header,
            objects: vec![],
        };
        let clicks = ClickTape::from_bits("").unwrap();
        let run = simulate_with_trace(&level, &clicks, SimulationConfig { max_ticks: 1 }).unwrap();
        assert_eq!(run.trace[0].state.mode, GameMode::Ship);
    }

    #[test]
    fn header_ka2_invalid_value_falls_back_to_cube() {
        let mut header = HashMap::new();
        header.insert("kA2".to_owned(), "not_a_number".to_owned());
        let level = Level {
            header,
            objects: vec![],
        };
        let clicks = ClickTape::from_bits("").unwrap();
        let run = simulate_with_trace(&level, &clicks, SimulationConfig { max_ticks: 1 }).unwrap();
        assert_eq!(run.trace[0].state.mode, GameMode::Cube);
    }

    #[test]
    fn header_ka4_start_speed_sets_initial_speed_profile() {
        let mut header = HashMap::new();
        header.insert("kA4".to_owned(), "2".to_owned());
        let level = Level {
            header,
            objects: vec![],
        };
        let clicks = ClickTape::from_bits("").unwrap();
        let run = simulate_with_trace(&level, &clicks, SimulationConfig { max_ticks: 1 }).unwrap();

        assert!((run.trace[0].state.player_speed - crate::consts::PLAYER_SPEED_2X).abs() < 0.0001);
        assert!((run.trace[0].state.speed_multiplier - 5.870002).abs() < 0.0001);
    }

    #[test]
    fn live_session_step_tape_matches_offline_trace() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![],
        };
        let clicks = ClickTape::from_bits("00111100001111").unwrap();
        let offline =
            simulate_with_trace(&level, &clicks, SimulationConfig { max_ticks: 64 }).unwrap();
        let mut session = LiveSimulationSession::new(&level).unwrap();
        let mut live_trace = Vec::new();

        for _ in 0..64 {
            let step = session.step_tape(&clicks).unwrap();
            live_trace.push(step.frame);
            assert!(step.outcome.is_none());
        }

        assert_eq!(offline.trace, live_trace);
    }

    #[test]
    fn click_tape_starts_on_first_attempt_tick_at_recording_start_x() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![],
        };
        let clicks = ClickTape::from_bits("1").unwrap();
        let run = simulate_with_trace(&level, &clicks, SimulationConfig { max_ticks: 1 }).unwrap();

        assert!(run.trace[0].pressed);
        assert!(
            (run.trace[0].state.x - FIRST_TRACE_X).abs() < 0.001,
            "first trace frame should align with bitstring recorder sample 0"
        );
    }

    #[test]
    fn cube_gravity_is_scaled_to_240hz_substep() {
        let mut player = PlayerState {
            x: 0.0,
            y: 180.0,
            vx: 5.77,
            vy: 10.0,
            mode: GameMode::Cube,
            gravity_sign: -1.0,
            mini: false,
            player_speed: 0.9,
            speed_multiplier: 5.77,
            gravity: 0.9582,
            y_start: 11.18,
            vehicle_size: 1.0,
            on_ground: false,
            was_jump_buffered: false,
            jump_buffered: false,
            state_ring_jump: false,
            on_slope: false,
            slope_exit_vy: 0.0,
            slope_exit_vx: 0.0,
            slope_contact_cooldown: 0,
            slope_object: None,
            slope_is_current_top: false,
            slope_prev_radius: 15.0,
            rotation: 0.0,
            is_accelerating: false,
            snapped_object: None,
            snap_distance: 0.0,
            dash_rotation_blocks_remaining: 0.0,
            dash_angle_deg: 0.0,
            hold_ticks: 0,
            pending_yvel_next_tick: 0.0,
        };

        update_cube(&mut player, false, 1.0);

        // GDP `updateJump.cpp` cube non-flying branch (lines 419-477):
        //   delta_vy = -flipMod * g * dtSlow * float_b   with float_b = 1.0
        // No state machine - same factor whether button held or not.
        let expected = 10.0 - 0.9582 * SUBSTEP_TO_FRAME * VERTICAL_SLOW * 1.0;
        assert!((player.vy - expected).abs() < 0.0001, "got {}", player.vy);
    }

    #[test]
    fn cube_hitbox_sizes_match_docs_for_normal_and_mini() {
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.mini = false;
        assert!(
            (player.player_half() - 15.0).abs() < 0.001,
            "normal cube red/main hitbox should be 30 units wide"
        );
        assert!(
            (cube_lethal_inner_half(&player) - 4.5).abs() < 0.001,
            "normal cube blue hitbox should be 9 units wide"
        );

        player.mini = true;
        assert!(
            (player.player_half() - 9.0).abs() < 0.001,
            "mini cube red/main hitbox should be 18 units wide"
        );
        assert!(
            (cube_lethal_inner_half(&player) - 5.0).abs() < 0.001,
            "mini cube blue hitbox should be 10 units wide"
        );
    }

    #[test]
    fn ship_hold_and_release_use_opengd_acceleration_profile() {
        // Canonical GDP `updateJump` ship branch (lines 237-303 of
        // `PlayerObject_updateJump.cpp`). Per-substep delta is
        //
        //   dvy = -v51 * float_c * dt * flipMod * v52 / v16
        //
        // with `usedGravity = 0.9582`, `dt = SUBSTEP_TO_FRAME *
        // VERTICAL_SLOW`, `v16 = 1.0` for normal ship, `v52 = 0.4`
        // (no falling-bugged), and the v51/float_c values selected
        // below. The previous version of this test asserted the
        // pre-port "extra_boost = 0.5 if pressed && falling" path,
        // which conflated playerIsFallingBugged with our naive
        // "vy < gravity" flag; that codepath is gone.
        let mut player = test_player_state();
        player.mode = GameMode::Ship;
        player.on_ground = false;
        player.is_accelerating = false;
        player.vy = 0.0;

        let flip_mod = player.flip_mod();
        update_ship(&mut player, true, flip_mod);
        // Pressed, !is_accelerating:
        // v51 = -1.0, float_c = usedGravity, v52 depends on falling-bugged.
        // At vy=0 in normal gravity, ship_player_is_falling_bugged() is true.
        let expected_hold =
            -(-1.0) * 0.9582 * SUBSTEP_TO_FRAME * VERTICAL_SLOW * 1.0 * SHIP_HOLD_FALLING_V52 / 1.0;
        assert!(
            (player.vy - expected_hold).abs() < 0.0001,
            "pressed-thrust should add {expected_hold}, got {}",
            player.vy
        );

        player.vy = 0.0;
        update_ship(&mut player, false, flip_mod);
        // Released, !is_accelerating at vy=0:
        // falling-bugged=true => v51=0.8, then GDP's released
        // non-platformer/non-accelerating branch forces the release scalar.
        let expected_release_falling =
            -0.8 * 0.9582 * SUBSTEP_TO_FRAME * VERTICAL_SLOW * 1.0 * SHIP_RELEASE_FALLING_V52 / 1.0;
        assert!(
            (player.vy - expected_release_falling).abs() < 0.0001,
            "release should add {expected_release_falling}, got {}",
            player.vy
        );

        player.vy = 2.0;
        update_ship(&mut player, false, flip_mod);
        // Released, !is_accelerating, vy > gravity (rising): falling-bugged
        // is false in normal gravity, so v51=1.2 and the release-rising scalar applies.
        let expected_release_rising = 2.0
            + (-1.2 * 0.9582 * SUBSTEP_TO_FRAME * VERTICAL_SLOW * 1.0 * SHIP_RELEASE_RISING_V52
                / 1.0);
        assert!(
            (player.vy - expected_release_rising).abs() < 0.0001,
            "rising release should yield {expected_release_rising}, got {}",
            player.vy
        );
    }

    #[test]
    fn mini_ship_uses_opengd_player_size_for_acceleration_and_lower_clamp() {
        let mut player = test_player_state();
        player.mode = GameMode::Ship;
        player.on_ground = false;
        player.is_accelerating = false;
        player.mini = true;
        player.vehicle_size = 0.6;
        player.vy = 0.0;

        let flip_mod = player.flip_mod();
        update_ship(&mut player, true, flip_mod);
        // Same path as above with v16 = 0.85 (mini-ship flyer override
        // at `updateJump.cpp` lines 128-130). At vy=0 in normal gravity,
        // falling-bugged=true so the hold-falling scalar applies.
        let expected_hold =
            -(-1.0) * 0.9582 * SUBSTEP_TO_FRAME * VERTICAL_SLOW * 1.0 * SHIP_HOLD_FALLING_V52
                / 0.85;
        assert!(
            (player.vy - expected_hold).abs() < 0.0001,
            "mini ship should use v16=0.85; got {} (expected {expected_hold})",
            player.vy
        );

        player.vy = -20.0;
        apply_mode_physics(&mut player, false);
        let expected_lower_clamp = -6.4 / 0.85;
        assert!(
            (player.vy - expected_lower_clamp).abs() < 0.0001,
            "mini ship lower clamp should be {expected_lower_clamp}, got {}",
            player.vy
        );

        player.mini = false;
        player.vehicle_size = 1.0;
        player.vy = 20.0;
        apply_mode_physics(&mut player, true);
        assert!(
            (player.vy - 8.0).abs() < 0.0001,
            "normal-gravity ship upper clamp should be 8.0, got {}",
            player.vy
        );
    }

    #[test]
    fn ship_flipped_gravity_release_accelerates_upward() {
        let mut player = test_player_state();
        player.mode = GameMode::Ship;
        player.on_ground = false;
        player.is_accelerating = false;
        player.gravity_sign = 1.0;
        player.vy = 0.5;

        let before = player.vy;
        let flip_mod = player.flip_mod();
        update_ship(&mut player, false, flip_mod);

        // Flipped gravity should make released ship gravity accelerate upward
        // (positive Y in our world coordinates). With vy=0.5 and gravity=0.9582,
        // falling-bugged=false => v51=1.2 and the release-rising scalar applies.
        let expected_delta =
            -1.2 * 0.9582 * SUBSTEP_TO_FRAME * VERTICAL_SLOW * -1.0 * SHIP_RELEASE_RISING_V52 / 1.0;
        assert!(
            (player.vy - (before + expected_delta)).abs() < 0.0001,
            "flipped gravity release should accelerate upward, got {} expected {}",
            player.vy,
            before + expected_delta
        );
    }

    #[test]
    fn ship_held_thrust_settles_at_canon_upper_clamp() {
        // Sanity check: a ship that holds the button forever in normal
        // gravity, with no external boosts, should settle at the GDP
        // upper vy clamp = +8.0 (`updateJump.cpp` line 298 -> `8.0/v16`,
        // v16=1.0). Drift past that means the post-step clamp is not
        // running, which used to happen when the previous code clamped
        // unconditionally - any boost (`is_accelerating`) lifted the
        // ceiling. Here `is_accelerating` is false, so the clamp must
        // engage every substep once vy >= 8.0.
        let mut player = test_player_state();
        player.mode = GameMode::Ship;
        player.on_ground = false;
        player.is_accelerating = false;
        player.vy = 0.0;
        player.hold_ticks = 3;

        for _ in 0..1000 {
            apply_mode_physics(&mut player, true);
        }
        assert!(
            (player.vy - 8.0).abs() < 1e-4,
            "held ship thrust should settle at upper clamp 8.0, got {}",
            player.vy
        );
    }

    #[test]
    fn ship_accelerating_boost_decays_back_to_normal_band() {
        // After a pad/orb impulse sets `is_accelerating = true` and
        // pushes vy past the normal band, the GDP "boost decayed"
        // gate (`updateJump.cpp` lines 133-145) clears the flag once
        // vy is back in `[v33, v30] = [-6.4, 8.0]`. The post-step
        // clamp then keeps vy inside that band. This test simulates
        // a burst at +20 with no input and checks both behaviors.
        let mut player = test_player_state();
        player.mode = GameMode::Ship;
        player.on_ground = false;
        player.is_accelerating = true;
        player.vy = 20.0;

        // Run plenty of frames with no input.
        for _ in 0..2000 {
            apply_mode_physics(&mut player, false);
        }
        assert!(
            !player.is_accelerating,
            "is_accelerating should be cleared once vy returns to band"
        );
        assert!(
            player.vy >= -6.4 - 1e-3 && player.vy <= 8.0 + 1e-3,
            "vy should be inside the normal band after decay, got {}",
            player.vy
        );
    }

    #[test]
    fn entering_ship_portal_halves_carried_y_velocity() {
        let mut player = test_player_state();
        player.vy = 8.0;
        player.on_ground = true;
        let level = Level {
            header: HashMap::new(),
            objects: vec![test_portal(13, ObjectKind::ModePortal, player.x, player.y)],
        };
        let mut touched = HashSet::new();
        let mut touched_teleports = HashSet::new();
        let mut dual_activate = false;
        let mut dual_deactivate = false;
        let exits = Vec::new();

        apply_portals(
            &level,
            &mut player,
            &mut touched,
            &exits,
            &mut touched_teleports,
            &mut dual_activate,
            &mut dual_deactivate,
            0,
        );

        assert_eq!(player.mode, GameMode::Ship);
        assert!((player.vy - 4.0).abs() < 0.0001, "got {}", player.vy);
        assert!(!player.on_ground, "Ship entry should clear grounded state");
    }

    #[test]
    fn ship_gravity_portal_halves_y_velocity_on_actual_flip() {
        let mut player = test_player_state();
        player.mode = GameMode::Ship;
        player.vy = 6.0;
        player.gravity_sign = -1.0;
        let level = Level {
            header: HashMap::new(),
            objects: vec![test_portal(
                11,
                ObjectKind::GravityPortal,
                player.x,
                player.y,
            )],
        };
        let mut touched = HashSet::new();
        let mut touched_teleports = HashSet::new();
        let mut dual_activate = false;
        let mut dual_deactivate = false;
        let exits = Vec::new();

        apply_portals(
            &level,
            &mut player,
            &mut touched,
            &exits,
            &mut touched_teleports,
            &mut dual_activate,
            &mut dual_deactivate,
            0,
        );

        assert_eq!(player.gravity_sign, 1.0);
        assert!((player.vy - 3.0).abs() < 0.0001, "got {}", player.vy);
    }

    #[test]
    fn gravity_portal_does_not_halve_y_velocity_without_actual_flip() {
        let mut player = test_player_state();
        player.mode = GameMode::Ship;
        player.vy = 6.0;
        player.gravity_sign = -1.0;
        let level = Level {
            header: HashMap::new(),
            objects: vec![test_portal(
                10,
                ObjectKind::GravityPortal,
                player.x,
                player.y,
            )],
        };
        let mut touched = HashSet::new();
        let mut touched_teleports = HashSet::new();
        let mut dual_activate = false;
        let mut dual_deactivate = false;
        let exits = Vec::new();

        apply_portals(
            &level,
            &mut player,
            &mut touched,
            &exits,
            &mut touched_teleports,
            &mut dual_activate,
            &mut dual_deactivate,
            0,
        );

        assert_eq!(player.gravity_sign, -1.0);
        assert!((player.vy - 6.0).abs() < 0.0001, "got {}", player.vy);
    }

    #[test]
    fn normal_gravity_has_no_fake_300px_ceiling() {
        let mut player = PlayerState {
            x: 0.0,
            y: 299.0,
            vx: 5.77,
            vy: 10.0,
            mode: GameMode::Cube,
            gravity_sign: -1.0,
            mini: false,
            player_speed: 0.9,
            speed_multiplier: 5.77,
            gravity: 0.9582,
            y_start: 11.18,
            vehicle_size: 1.0,
            on_ground: false,
            was_jump_buffered: false,
            jump_buffered: false,
            state_ring_jump: false,
            on_slope: false,
            slope_exit_vy: 0.0,
            slope_exit_vx: 0.0,
            slope_contact_cooldown: 0,
            slope_object: None,
            slope_is_current_top: false,
            slope_prev_radius: 15.0,
            rotation: 0.0,
            is_accelerating: false,
            snapped_object: None,
            snap_distance: 0.0,
            dash_rotation_blocks_remaining: 0.0,
            dash_angle_deg: 0.0,
            hold_ticks: 0,
            pending_yvel_next_tick: 0.0,
        };

        let level = Level {
            header: HashMap::new(),
            objects: Vec::new(),
        };
        let grounded = apply_implicit_bounds(&level, &mut player);

        assert!(!grounded);
        assert_eq!(player.y, 299.0);
        assert_eq!(player.vy, 10.0);
    }

    #[test]
    fn pop_saw_at_360_330_does_not_kill_near_miss() {
        let player = PlayerState {
            x: 298.59726,
            y: 331.79346,
            vx: 5.77,
            vy: -11.715417,
            mode: GameMode::Cube,
            gravity_sign: -1.0,
            mini: false,
            player_speed: 0.9,
            speed_multiplier: 5.77,
            gravity: 0.9582,
            y_start: 11.18,
            vehicle_size: 1.0,
            on_ground: false,
            was_jump_buffered: false,
            jump_buffered: false,
            state_ring_jump: false,
            on_slope: false,
            slope_exit_vy: 0.0,
            slope_exit_vx: 0.0,
            slope_contact_cooldown: 0,
            slope_object: None,
            slope_is_current_top: false,
            slope_prev_radius: 15.0,
            rotation: 0.0,
            is_accelerating: false,
            snapped_object: None,
            snap_distance: 0.0,
            dash_rotation_blocks_remaining: 0.0,
            dash_angle_deg: 0.0,
            hold_ticks: 0,
            pending_yvel_next_tick: 0.0,
        };
        // Original recorded scenario was a near-miss against a saw whose
        // *world* center was (360, 330). Under the previous (incorrect)
        // `+15, +15` collision shift, that saw was stored as
        // `LevelObject { x: 345, y: 315 }`. Now object position is the
        // world center directly, so the fixture uses (360, 330).
        let hazard = test_circle_hazard(1705, 360.0, 330.0, 32.3);
        let rect = object_rect(&hazard).unwrap();

        assert!(
            !lethal_hazard_intersects(&hazard, rect, player, player.player_half()),
            "saw circle is at object position (no +15 shift); recorded Pop near-miss should not kill"
        );
    }

    #[test]
    fn pop_id_468_uses_opengd_outer_bounds_after_rotation() {
        let object = LevelObject {
            object_id: 468,
            x: 420.8,
            y: 135.0,
            rotation: -270.0,
            scale: 1.0,
            scale_x: 1.0,
            scale_y: 1.0,
            groups: Vec::new(),
            kind: ObjectKind::Solid,
            hitbox: Some(HitboxData::Box {
                offset: [0.0, 0.0],
                half_extents: [15.0, 0.75],
            }),
            raw: HashMap::new(),
        };

        let rect = object_rect(&object).unwrap();

        // Object position is the cell center directly (no hidden +15
        // shift), so the rotated bounds center matches `(object.x, object.y)`.
        assert!((rect.center[0] - 420.8).abs() < 0.001);
        assert!((rect.center[1] - 135.0).abs() < 0.001);
        assert!((rect.half_extents[0] - 0.75).abs() < 0.001);
        assert!((rect.half_extents[1] - 15.0).abs() < 0.001);
    }

    #[test]
    fn level_completes_sixty_units_past_last_gameplay_block_without_end_trigger() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![
                test_solid(1, 30.0, 90.0),
                LevelObject {
                    object_id: 1006,
                    x: 500.0,
                    y: 15.0,
                    rotation: 0.0,
                    scale: 1.0,
                    scale_x: 1.0,
                    scale_y: 1.0,
                    groups: Vec::new(),
                    kind: ObjectKind::Trigger,
                    hitbox: None,
                    raw: HashMap::new(),
                },
                test_solid(2, 300.0, 90.0),
            ],
        };

        assert!(!end_reached(&level, 359.0, 359.9));
        assert!(end_reached(&level, 359.0, 360.0));
    }

    #[test]
    fn airborne_held_press_can_fire_orb_after_entering_hitbox() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 1333,
                x: 720.0,
                y: 240.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Orb,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [18.0, 18.0],
                }),
                raw: HashMap::new(),
            }],
        };
        let mut player = PlayerState {
            x: 734.0,
            y: 230.0,
            vx: 5.77,
            vy: -7.0,
            mode: GameMode::Cube,
            gravity_sign: -1.0,
            mini: false,
            player_speed: 0.9,
            speed_multiplier: 5.77,
            gravity: 0.9582,
            y_start: 11.18,
            vehicle_size: 1.0,
            on_ground: false,
            was_jump_buffered: true,
            jump_buffered: true,
            state_ring_jump: true,
            on_slope: false,
            slope_exit_vy: 0.0,
            slope_exit_vx: 0.0,
            slope_contact_cooldown: 0,
            slope_object: None,
            slope_is_current_top: false,
            slope_prev_radius: 15.0,
            rotation: 0.0,
            is_accelerating: false,
            snapped_object: None,
            snap_distance: 0.0,
            dash_rotation_blocks_remaining: 0.0,
            dash_angle_deg: 0.0,
            hold_ticks: 0,
            pending_yvel_next_tick: 0.0,
        };
        let mut touched = HashSet::new();

        apply_orbs(&level, &mut player, &mut touched, 0, true);

        assert!(player.vy > 14.0);
        assert!(!player.state_ring_jump);
    }

    #[test]
    fn gravity_jump_orb_synonym_1022_uses_green_orb_behavior() {
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.gravity_sign = 1.0; // inverted before hit
        player.y_start = 11.1800318;
        player.x = 15.0;
        player.y = 15.0;

        apply_orbs(
            &Level {
                header: HashMap::new(),
                objects: vec![LevelObject {
                    object_id: 1022,
                    x: 15.0,
                    y: 15.0,
                    rotation: 0.0,
                    scale: 1.0,
                    scale_x: 1.0,
                    scale_y: 1.0,
                    groups: Vec::new(),
                    kind: ObjectKind::Orb,
                    hitbox: Some(HitboxData::Box {
                        offset: [0.0, 0.0],
                        half_extents: [18.0, 18.0],
                    }),
                    raw: HashMap::new(),
                }],
            },
            &mut player,
            &mut HashSet::new(),
            0,
            true,
        );

        assert_eq!(player.gravity_sign, -1.0);
        assert!(
            (player.vy - 11.1800318).abs() < 0.001,
            "id=1022 should match green orb: flip then yellow impulse, got vy={}",
            player.vy
        );
    }

    #[test]
    fn gravity_jump_orb_family_fires_on_held_touch_without_press_start() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 1022,
                x: 120.0,
                y: 120.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Orb,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [18.0, 18.0],
                }),
                raw: HashMap::new(),
            }],
        };
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.gravity_sign = 1.0;
        player.y_start = 11.1800318;
        player.x = 120.0;
        player.y = 120.0;
        player.was_jump_buffered = true; // pressed but not a fresh press-start
        player.state_ring_jump = false; // no queued-air-press either
        let mut touched = HashSet::new();

        apply_orbs(&level, &mut player, &mut touched, 0, true);

        assert_eq!(player.gravity_sign, -1.0);
        assert!(
            player.vy > 11.0,
            "held touch should still fire gravity-jump family, got vy={}",
            player.vy
        );
    }

    #[test]
    fn implicit_ceiling_at_2505_is_death_not_ground() {
        let mut player = PlayerState {
            x: 100.0,
            y: 2505.0,
            vx: 5.77,
            vy: 12.0,
            mode: GameMode::Cube,
            gravity_sign: 1.0,
            mini: false,
            player_speed: 0.9,
            speed_multiplier: 5.77,
            gravity: 0.9582,
            y_start: 11.18,
            vehicle_size: 1.0,
            on_ground: false,
            was_jump_buffered: false,
            jump_buffered: false,
            state_ring_jump: false,
            on_slope: false,
            slope_exit_vy: 0.0,
            slope_exit_vx: 0.0,
            slope_contact_cooldown: 0,
            slope_object: None,
            slope_is_current_top: false,
            slope_prev_radius: 15.0,
            rotation: 0.0,
            is_accelerating: false,
            snapped_object: None,
            snap_distance: 0.0,
            dash_rotation_blocks_remaining: 0.0,
            dash_angle_deg: 0.0,
            hold_ticks: 0,
            pending_yvel_next_tick: 0.0,
        };
        let level = Level {
            header: HashMap::new(),
            objects: Vec::new(),
        };

        let outcome = resolve_collisions(&level, 42, &mut player, 0);

        assert!(matches!(
            outcome,
            Some(SimulationOutcome::Died {
                reason,
                object_id: None,
                ..
            }) if reason == "ceiling"
        ));
    }

    #[test]
    fn h_square_1859_blocks_cube_ceiling_death_when_overlapping() {
        let mut player = PlayerState {
            x: 1605.0,
            y: 2505.0,
            vx: 5.77,
            vy: 6.0,
            mode: GameMode::Cube,
            gravity_sign: 1.0,
            mini: false,
            player_speed: 0.9,
            speed_multiplier: 5.77,
            gravity: 0.9582,
            y_start: 11.18,
            vehicle_size: 1.0,
            on_ground: false,
            was_jump_buffered: false,
            jump_buffered: false,
            state_ring_jump: false,
            on_slope: false,
            slope_exit_vy: 0.0,
            slope_exit_vx: 0.0,
            slope_contact_cooldown: 0,
            slope_object: None,
            slope_is_current_top: false,
            slope_prev_radius: 15.0,
            rotation: 0.0,
            is_accelerating: false,
            snapped_object: None,
            snap_distance: 0.0,
            dash_rotation_blocks_remaining: 0.0,
            dash_angle_deg: 0.0,
            hold_ticks: 0,
            pending_yvel_next_tick: 0.0,
        };
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 1859,
                x: 1605.0,
                y: 2505.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Solid,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [15.0, 15.0],
                }),
                raw: HashMap::new(),
            }],
        };

        let outcome = resolve_collisions(&level, 42, &mut player, 0);
        assert!(
            outcome.is_none(),
            "overlapping id=1859 should suppress cube ceiling death"
        );
    }

    #[test]
    fn ceiling_hit_kills_cube_without_h_square_even_if_in_other_solid() {
        let mut player = PlayerState {
            x: 1665.0,
            y: 2505.0,
            vx: 5.77,
            vy: 6.0,
            mode: GameMode::Cube,
            gravity_sign: 1.0,
            mini: false,
            player_speed: 0.9,
            speed_multiplier: 5.77,
            gravity: 0.9582,
            y_start: 11.18,
            vehicle_size: 1.0,
            on_ground: false,
            was_jump_buffered: false,
            jump_buffered: false,
            state_ring_jump: false,
            on_slope: false,
            slope_exit_vy: 0.0,
            slope_exit_vx: 0.0,
            slope_contact_cooldown: 0,
            slope_object: None,
            slope_is_current_top: false,
            slope_prev_radius: 15.0,
            rotation: 0.0,
            is_accelerating: false,
            snapped_object: None,
            snap_distance: 0.0,
            dash_rotation_blocks_remaining: 0.0,
            dash_angle_deg: 0.0,
            hold_ticks: 0,
            pending_yvel_next_tick: 0.0,
        };
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 1,
                x: 1650.0,
                y: 2490.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Solid,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [15.0, 15.0],
                }),
                raw: HashMap::new(),
            }],
        };

        let outcome = resolve_collisions(&level, 42, &mut player, 0);
        assert!(matches!(
            outcome,
            Some(SimulationOutcome::Died { reason, .. }) if reason == "ceiling" || reason == "ceiling hit"
        ));
    }

    #[test]
    fn inverted_gravity_does_not_treat_ceiling_as_ground_before_death_height() {
        let mut player = PlayerState {
            x: 100.0,
            y: 2490.0,
            vx: 5.77,
            vy: 12.0,
            mode: GameMode::Cube,
            gravity_sign: 1.0,
            mini: false,
            player_speed: 0.9,
            speed_multiplier: 5.77,
            gravity: 0.9582,
            y_start: 11.18,
            vehicle_size: 1.0,
            on_ground: false,
            was_jump_buffered: false,
            jump_buffered: false,
            state_ring_jump: false,
            on_slope: false,
            slope_exit_vy: 0.0,
            slope_exit_vx: 0.0,
            slope_contact_cooldown: 0,
            slope_object: None,
            slope_is_current_top: false,
            slope_prev_radius: 15.0,
            rotation: 0.0,
            is_accelerating: false,
            snapped_object: None,
            snap_distance: 0.0,
            dash_rotation_blocks_remaining: 0.0,
            dash_angle_deg: 0.0,
            hold_ticks: 0,
            pending_yvel_next_tick: 0.0,
        };

        let level = Level {
            header: HashMap::new(),
            objects: Vec::new(),
        };
        let grounded = apply_implicit_bounds(&level, &mut player);

        assert!(!grounded);
        assert!(!player.on_ground);
        assert_eq!(player.y, 2490.0);
        assert_eq!(player.vy, 12.0);
    }

    #[test]
    fn check_snap_jump_to_object_applies_gdp_little_stair_x_correction() {
        let mut player = PlayerState {
            x: 31.5,
            y: 105.0,
            vx: 5.77,
            vy: 0.0,
            mode: GameMode::Cube,
            gravity_sign: -1.0,
            mini: false,
            player_speed: 0.9,
            speed_multiplier: 5.77,
            gravity: 0.9582,
            y_start: 11.18,
            vehicle_size: 1.0,
            on_ground: true,
            was_jump_buffered: false,
            jump_buffered: false,
            state_ring_jump: false,
            on_slope: false,
            slope_exit_vy: 0.0,
            slope_exit_vx: 0.0,
            slope_contact_cooldown: 0,
            slope_object: None,
            slope_is_current_top: false,
            slope_prev_radius: 15.0,
            rotation: 0.0,
            is_accelerating: false,
            snapped_object: Some(0),
            snap_distance: 1.5,
            dash_rotation_blocks_remaining: 0.0,
            dash_angle_deg: 0.0,
            hold_ticks: 0,
            pending_yvel_next_tick: 0.0,
        };
        let previous = test_solid(1, 30.0, 90.0);
        let next = test_solid(2, 150.0, 120.0);

        check_snap_jump_to_object(&mut player, 1, &previous, &next);

        assert_eq!(player.snapped_object, Some(1));
        assert_eq!(player.snap_distance, -118.5);
        assert!((player.x - 32.5).abs() < 0.001);
    }

    #[test]
    fn side_hit_is_not_lethal_during_slope_transition_without_core_contact() {
        let player = PlayerState {
            x: 105.0,
            y: 75.0,
            vx: 5.77,
            vy: 0.0,
            mode: GameMode::Cube,
            gravity_sign: -1.0,
            mini: false,
            player_speed: 0.9,
            speed_multiplier: 5.77,
            gravity: 0.9582,
            y_start: 11.18,
            vehicle_size: 1.0,
            on_ground: true,
            was_jump_buffered: false,
            jump_buffered: false,
            state_ring_jump: false,
            on_slope: true,
            slope_exit_vy: 0.0,
            slope_exit_vx: 0.0,
            slope_contact_cooldown: 0,
            slope_object: None,
            slope_is_current_top: false,
            slope_prev_radius: 15.0,
            rotation: 0.0,
            is_accelerating: false,
            snapped_object: None,
            snap_distance: 0.0,
            dash_rotation_blocks_remaining: 0.0,
            dash_angle_deg: 0.0,
            hold_ticks: 0,
            pending_yvel_next_tick: 0.0,
        };
        let block = Rect {
            center: [116.0, 90.0],
            half_extents: [15.0, 15.0],
        };

        assert!(!side_hit_is_lethal(block, player, 15.0, true, true, 0));
    }

    #[test]
    fn side_hit_is_not_lethal_during_slope_contact_cooldown() {
        let mut player = PlayerState {
            x: 127.0,
            y: 105.0,
            vx: 5.77,
            vy: 0.0,
            mode: GameMode::Cube,
            gravity_sign: -1.0,
            mini: false,
            player_speed: 0.9,
            speed_multiplier: 5.77,
            gravity: 0.9582,
            y_start: 11.18,
            vehicle_size: 1.0,
            on_ground: true,
            was_jump_buffered: false,
            jump_buffered: false,
            state_ring_jump: false,
            on_slope: false,
            slope_exit_vy: 0.0,
            slope_exit_vx: 0.0,
            slope_contact_cooldown: 1,
            slope_object: None,
            slope_is_current_top: false,
            slope_prev_radius: 15.0,
            rotation: 0.0,
            is_accelerating: false,
            snapped_object: None,
            snap_distance: 0.0,
            dash_rotation_blocks_remaining: 0.0,
            dash_angle_deg: 0.0,
            hold_ticks: 0,
            pending_yvel_next_tick: 0.0,
        };
        let block = Rect {
            center: [135.0, 105.0],
            half_extents: [15.0, 15.0],
        };

        assert!(!side_hit_is_lethal(block, player, 15.0, false, false, 1));
        player.slope_contact_cooldown = 0;
        assert!(side_hit_is_lethal(block, player, 15.0, false, false, 0));
    }

    #[test]
    fn ship_side_hit_uses_rotated_inner_hitbox() {
        let mut player = test_player_state();
        player.mode = GameMode::Ship;
        player.on_ground = false;
        player.x = 0.0;
        player.y = 0.0;
        // 45deg ship: rotated inner square reaches farther on X than
        // the axis-aligned half (4.5).
        player.rotation = 45.0;
        let probe = Rect {
            center: [5.6, 0.0],
            half_extents: [0.2, 0.2],
        };
        assert!(
            side_hit_is_lethal(probe, player, 15.0, false, false, 0),
            "rotated ship inner hitbox should intersect this probe"
        );

        player.rotation = 0.0;
        assert!(
            !side_hit_is_lethal(probe, player, 15.0, false, false, 0),
            "unrotated ship inner hitbox should miss this probe"
        );
    }

    #[test]
    fn ground_probe_marks_player_grounded_within_point_one_units_of_floor() {
        let level = Level {
            header: HashMap::new(),
            objects: Vec::new(),
        };
        let mut player = test_player_state();
        player.y = 105.05;
        player.vy = -2.0;
        player.on_ground = false;

        refresh_ground_probe(&level, &mut player);
        assert!(player.on_ground);

        player.y = 105.25;
        player.on_ground = false;
        refresh_ground_probe(&level, &mut player);
        assert!(!player.on_ground);
    }

    #[test]
    fn blue_pad_respects_rotation_half_plane_for_normal_gravity() {
        let level_inactive = Level {
            header: HashMap::new(),
            objects: vec![test_blue_pad(-180.0)],
        };
        let mut touched = HashSet::new();
        let mut player = test_player_state();
        player.x = 15.0;
        player.y = 15.0;

        apply_pads(&level_inactive, &mut player, &mut touched, 0);
        assert_eq!(
            player.gravity_sign, -1.0,
            "rotation 180 should not activate in normal gravity"
        );

        let level_active = Level {
            header: HashMap::new(),
            objects: vec![test_blue_pad(0.0)],
        };
        let mut touched_active = HashSet::new();
        let mut player_active = test_player_state();
        player_active.x = 15.0;
        player_active.y = 15.0;
        apply_pads(&level_active, &mut player_active, &mut touched_active, 0);
        assert_eq!(
            player_active.gravity_sign, 1.0,
            "rotation 0 should activate in normal gravity"
        );
    }

    #[test]
    fn blue_pad_respects_rotation_half_plane_for_flipped_gravity() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![test_blue_pad(-180.0)],
        };
        let mut touched = HashSet::new();
        let mut player = test_player_state();
        player.gravity_sign = 1.0;
        player.x = 15.0;
        player.y = 15.0;

        apply_pads(&level, &mut player, &mut touched, 0);
        assert_eq!(
            player.gravity_sign, -1.0,
            "rotation 180 should activate in flipped gravity"
        );
    }

    #[test]
    fn blue_pad_not_marked_touched_when_half_plane_rejects_then_can_activate_later() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![test_blue_pad(-180.0)],
        };
        let mut touched = HashSet::new();
        let mut player = test_player_state();
        player.mode = GameMode::Ball;
        player.x = 15.0;
        player.y = 15.0;
        player.gravity_sign = -1.0; // rotation -180 rejected in normal gravity

        apply_pads(&level, &mut player, &mut touched, 0);
        assert!(
            touched.is_empty(),
            "blue pad should not be consumed when rejected by gravity half-plane"
        );
        assert_eq!(player.gravity_sign, -1.0);

        // Same overlap, now valid from flipped gravity side.
        player.gravity_sign = 1.0;
        apply_pads(&level, &mut player, &mut touched, 0);
        assert_eq!(player.gravity_sign, -1.0, "blue pad should flip once valid");
        assert!(
            !touched.is_empty(),
            "successful activation should consume pad"
        );
    }

    #[test]
    fn pre_collision_blue_pad_can_fire_before_embedded_solid_resolution() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![
                LevelObject {
                    object_id: 67,
                    x: 15.0,
                    y: 15.0,
                    rotation: 0.0,
                    scale: 1.0,
                    scale_x: 1.0,
                    scale_y: 1.0,
                    groups: Vec::new(),
                    kind: ObjectKind::Pad,
                    hitbox: Some(HitboxData::Box {
                        offset: [0.0, 0.0],
                        half_extents: [15.0, 15.0],
                    }),
                    raw: HashMap::new(),
                },
                LevelObject {
                    object_id: 1,
                    x: 15.0,
                    y: 15.0,
                    rotation: 0.0,
                    scale: 1.0,
                    scale_x: 1.0,
                    scale_y: 1.0,
                    groups: Vec::new(),
                    kind: ObjectKind::Solid,
                    hitbox: Some(HitboxData::Box {
                        offset: [0.0, 0.0],
                        half_extents: [15.0, 15.0],
                    }),
                    raw: HashMap::new(),
                },
            ],
        };
        let mut touched = HashSet::new();
        let mut player = test_player_state();
        player.x = 15.0;
        player.y = 15.0;
        player.gravity_sign = -1.0;

        apply_blue_pads_pre_collision(&level, &mut player, &mut touched, 0);
        assert_eq!(
            player.gravity_sign, 1.0,
            "blue pad should flip before solid resolution"
        );
    }

    #[test]
    fn blue_pad_flip_does_not_force_hold_jump_same_frame() {
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.gravity_sign = 1.0;
        player.jump_buffered = true;
        player.vy = 0.0;

        apply_blue_pad(&mut player, -180.0, 1.0);
        assert_eq!(player.gravity_sign, -1.0);
        assert!(
            (player.vy + 6.4).abs() < 0.01,
            "blue pad should only apply its own impulse; hold-jump is resolved by grounded update"
        );
    }

    #[test]
    fn airborne_mini_cube_triggers_non_rotated_blue_pad_with_outer_box() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![test_blue_pad(0.0)],
        };
        let mut touched = HashSet::new();
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.mini = true;
        player.on_ground = false;
        player.gravity_sign = -1.0;
        // Construct a case where the mini cube's non-rotated outer box
        // actually overlaps the blue pad's transformed visible hitbox.
        player.x = 15.9;
        player.y = 38.9;

        apply_pads(&level, &mut player, &mut touched, 0);
        assert_eq!(
            player.gravity_sign, 1.0,
            "airborne mini cube should activate non-rotated blue pad via outer box overlap"
        );
    }

    #[test]
    fn airborne_mini_cube_triggers_rotated_yellow_pad_with_rotated_hitbox() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![test_yellow_pad(90.0)],
        };
        let mut touched = HashSet::new();
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.mini = true;
        player.on_ground = false;
        player.gravity_sign = -1.0;
        player.vy = -1.0;
        // Place the mini cube where the rotated yellow-pad activation zone
        // should overlap in Pop-like layouts.
        player.x = 15.0;
        player.y = 30.0;

        let vy_before = player.vy;
        apply_pads(&level, &mut player, &mut touched, 0);
        assert!(
            player.vy > vy_before,
            "rotated yellow pad should fire on airborne mini-cube overlap"
        );
    }

    #[test]
    fn mini_cube_purple_pad_impulse_is_scaled_to_point_eight() {
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.mini = true;
        player.gravity_sign = -1.0; // flip_mod = +1
        player.vy = 0.0;
        let flip_mod = player.flip_mod();

        apply_purple_pad(&mut player, flip_mod, 0.0, 1.0);
        assert!(
            (player.vy - 8.32).abs() < 0.001,
            "mini cube purple pad should be 10.4 * 0.8 = 8.32, got {}",
            player.vy
        );
    }

    #[test]
    fn purple_pad_does_not_activate_while_player_is_still_above_visual_hitbox() {
        // Regression from live pop_ticks/object dump:
        // purple pad is at x=645,y=242 and old activation triggered around
        // x=634.1272,y=264.0419 despite no visible hitbox overlap.
        let pad = LevelObject {
            object_id: 140,
            x: 645.0,
            y: 242.0,
            rotation: 0.0,
            scale: 1.0,
            scale_x: 1.0,
            scale_y: 1.0,
            groups: Vec::new(),
            kind: ObjectKind::Pad,
            hitbox: Some(HitboxData::Box {
                offset: [0.0, 0.0],
                half_extents: [12.5, 2.5],
            }),
            raw: HashMap::new(),
        };
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.mini = true;
        player.on_ground = false;
        player.x = 634.1272;
        player.y = 264.0419;

        assert!(
            !intersects_pad_activation(&pad, player),
            "purple pad should not activate before player hitboxes overlap visual pad hitbox"
        );
    }

    #[test]
    fn flipped_ball_does_not_activate_rotated_pad_from_rotated_aabb_corner() {
        let pad = LevelObject {
            object_id: 140,
            x: 100.0,
            y: 100.0,
            rotation: 45.0,
            scale: 1.0,
            scale_x: 1.0,
            scale_y: 1.0,
            groups: Vec::new(),
            kind: ObjectKind::Pad,
            hitbox: Some(HitboxData::Box {
                offset: [0.0, 0.0],
                half_extents: [12.5, 2.5],
            }),
            raw: HashMap::new(),
        };
        let mut player = test_player_state();
        player.mode = GameMode::Ball;
        player.gravity_sign = 1.0;
        player.on_ground = false;
        // This overlaps the rotated pad's expanded AABB corner, but not the
        // actual rotated pad quad. Ball should use its normal 30x30 hitbox
        // against the pad's rotated hitbox, not AABB-vs-AABB.
        player.x = 125.0;
        player.y = 125.0;

        assert!(
            !intersects_pad_activation(&pad, player),
            "flipped ball should not activate rotated pad from the pad AABB corner"
        );
    }

    #[test]
    fn ball_pad_activation_requires_visible_pad_hitbox_overlap() {
        let pad = LevelObject {
            object_id: 67,
            x: 100.0,
            y: 100.0,
            rotation: 0.0,
            scale: 1.0,
            scale_x: 1.0,
            scale_y: 1.0,
            groups: Vec::new(),
            kind: ObjectKind::Pad,
            hitbox: Some(HitboxData::Box {
                offset: [0.0, 0.0],
                half_extents: [12.5, 3.0],
            }),
            raw: HashMap::new(),
        };
        let mut player = test_player_state();
        player.mode = GameMode::Ball;
        player.gravity_sign = 1.0;
        player.on_ground = false;
        player.x = 100.0;
        // The old gravity-pad activation volume reaches upward to y=122, but
        // the visible pad hitbox only reaches y=103. The ball bottom is 0.1u
        // above the visible pad, so activation here is visibly early.
        player.y = 118.1;

        assert!(
            !intersects_pad_activation(&pad, player),
            "ball should not activate a pad until its normal hitbox overlaps the visible pad hitbox"
        );
    }

    #[test]
    fn all_pad_ids_require_visible_hitbox_overlap_not_legacy_activation_zone() {
        let pad_specs = [
            (35, [12.5, 2.0]),
            (67, [12.5, 3.0]),
            (140, [12.5, 2.5]),
            (1332, [14.5, 3.5]),
        ];

        for (object_id, half_extents) in pad_specs {
            let pad = LevelObject {
                object_id,
                x: 100.0,
                y: 100.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Pad,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents,
                }),
                raw: HashMap::new(),
            };
            let mut player = test_player_state();
            player.mode = GameMode::Cube;
            player.on_ground = false;
            player.x = 100.0;
            // Place the player's bottom just above the visible pad, while
            // still inside the old 25px+ legacy activation rectangles.
            player.y = 100.0 + half_extents[1] + player.player_half() + 0.1;

            assert!(
                !intersects_pad_activation(&pad, player),
                "pad id {object_id} should not activate from legacy-only overlap"
            );
        }
    }

    #[test]
    fn mini_cube_pink_orb_impulse_is_scaled_to_point_eight() {
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.mini = true;
        player.gravity_sign = -1.0; // flip_mod = +1
        player.vy = 0.0;

        apply_pink_orb(&mut player);
        let expected = 11.18 * 0.72 * 0.8;
        assert!(
            (player.vy - expected).abs() < 0.001,
            "mini cube pink orb should be cube yellow * 0.72 * 0.8 = {expected}, got {}",
            player.vy
        );
    }

    #[test]
    fn non_flyer_upward_velocity_is_not_capped_in_normal_gravity() {
        let mut player = test_player_state();
        player.mode = GameMode::Ball;
        player.gravity_sign = -1.0; // normal gravity: upward is +vy
        player.on_ground = false;
        player.vy = 20.0;

        apply_mode_physics(&mut player, false);

        assert!(
            player.vy > 19.0,
            "upward vy should not be clamped to 15 in normal gravity; got {}",
            player.vy
        );
    }

    #[test]
    fn non_flyer_upward_velocity_is_not_capped_in_inverted_gravity() {
        let mut player = test_player_state();
        player.mode = GameMode::Ball;
        player.gravity_sign = 1.0; // inverted gravity: upward is -vy
        player.on_ground = false;
        player.vy = -20.0;

        apply_mode_physics(&mut player, false);

        assert!(
            player.vy < -19.0,
            "upward vy should not be clamped to -15 in inverted gravity; got {}",
            player.vy
        );
    }

    #[test]
    fn non_flyer_falling_velocity_is_capped_by_gravity_direction() {
        let mut normal = test_player_state();
        normal.mode = GameMode::Ball;
        normal.gravity_sign = -1.0;
        normal.on_ground = false;
        normal.vy = -20.0; // falling in normal gravity
        apply_mode_physics(&mut normal, false);
        assert!(
            (normal.vy + 15.0).abs() < 0.001,
            "normal-gravity falling speed should clamp to -15, got {}",
            normal.vy
        );

        let mut inverted = test_player_state();
        inverted.mode = GameMode::Ball;
        inverted.gravity_sign = 1.0;
        inverted.on_ground = false;
        inverted.vy = 20.0; // falling in inverted gravity
        apply_mode_physics(&mut inverted, false);
        assert!(
            (inverted.vy - 15.0).abs() < 0.001,
            "inverted-gravity falling speed should clamp to +15, got {}",
            inverted.vy
        );
    }

    #[test]
    fn ball_click_uses_point_three_cube_jump_then_flips_gravity() {
        let mut player = test_player_state();
        player.mode = GameMode::Ball;
        player.on_ground = true;
        player.gravity_sign = -1.0;
        player.y_start = 11.1800318;
        player.was_jump_buffered = false;

        step_player(&mut player, true);

        assert_eq!(player.gravity_sign, 1.0);
        assert!(
            (player.vy - (11.1800318 * 0.3)).abs() < 0.001,
            "ball click should set vy to 0.3*cube jump before gravity flip; got {}",
            player.vy
        );
    }

    #[test]
    fn ball_held_input_does_not_flip_without_new_press_start() {
        let mut player = test_player_state();
        player.mode = GameMode::Ball;
        player.on_ground = true;
        player.gravity_sign = -1.0;
        player.y_start = 11.1800318;
        // Simulate a hold that started earlier (no new press-start this tick).
        player.jump_buffered = true;
        player.was_jump_buffered = true;

        step_player(&mut player, true);

        assert_eq!(
            player.gravity_sign, -1.0,
            "held input without press_start must not flip ball gravity"
        );
        assert!(
            player.vy.abs() < 0.001,
            "held input without press_start should not apply impulse; got {}",
            player.vy
        );
    }

    #[test]
    fn ball_midair_click_buffers_single_flip_until_landing() {
        let mut player = test_player_state();
        player.mode = GameMode::Ball;
        player.on_ground = false;
        player.gravity_sign = -1.0;
        player.y_start = 11.1800318;

        // Midair click edge queues a landing flip.
        step_player(&mut player, true);
        assert!(
            player.state_ring_jump,
            "midair click should queue ball flip"
        );
        let queued_gravity = player.gravity_sign;

        // Release before landing: queue should persist.
        step_player(&mut player, false);
        assert!(
            player.state_ring_jump,
            "releasing should not clear queued ball flip"
        );
        assert_eq!(player.gravity_sign, queued_gravity);

        // On landing, queued click is consumed once even without fresh press.
        player.on_ground = true;
        step_player(&mut player, false);
        assert_eq!(player.gravity_sign, -queued_gravity);
        assert!(
            !player.state_ring_jump,
            "queued flip should be consumed on landing"
        );

        // Staying held/released should not immediately flip again.
        player.on_ground = true;
        let post_flip_gravity = player.gravity_sign;
        step_player(&mut player, false);
        assert_eq!(player.gravity_sign, post_flip_gravity);
    }

    #[test]
    fn ball_window_from_portal_allows_ground_landing_without_death() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 47,
                x: 100.0,
                y: 210.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::ModePortal,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [12.5, 37.5],
                }),
                raw: HashMap::new(),
            }],
        };
        let mut player = test_player_state();
        player.mode = GameMode::Ball;
        player.gravity_sign = -1.0;
        player.x = 140.0;
        player.y = 80.0;
        player.vy = -4.0;
        player.on_ground = false;

        let outcome = resolve_collisions(&level, 10, &mut player, 0);
        assert!(outcome.is_none());
        assert!(player.on_ground, "ball should ground on window floor");
        assert!(
            (player.y - 105.0).abs() < 0.001,
            "expected floor clamp at y=105"
        );
    }

    #[test]
    fn ball_window_from_portal_allows_ceiling_landing_when_flipped() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 47,
                x: 100.0,
                y: 210.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::ModePortal,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [12.5, 37.5],
                }),
                raw: HashMap::new(),
            }],
        };
        let mut player = test_player_state();
        player.mode = GameMode::Ball;
        player.gravity_sign = 1.0;
        player.x = 140.0;
        player.y = 335.0;
        player.vy = 4.0;
        player.on_ground = false;

        let outcome = resolve_collisions(&level, 10, &mut player, 0);
        assert!(outcome.is_none());
        assert!(
            player.on_ground,
            "flipped ball should ground on window ceiling"
        );
        assert!(
            (player.y - 315.0).abs() < 0.001,
            "expected ceiling clamp at y=315"
        );
    }

    #[test]
    fn returning_to_cube_disables_ball_window_ceiling_protection() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 47,
                x: 100.0,
                y: 210.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::ModePortal,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [12.5, 37.5],
                }),
                raw: HashMap::new(),
            }],
        };
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.x = 140.0;
        player.y = IMPLICIT_CEILING_DEATH_Y;

        let outcome = resolve_collisions(&level, 10, &mut player, 0);
        assert!(matches!(
            outcome,
            Some(SimulationOutcome::Died { reason, .. }) if reason == "ceiling"
        ));
    }

    #[test]
    fn gravity_portal_changes_ball_gravity_on_overlap() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 11,
                x: 120.0,
                y: 200.0,
                rotation: -27.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::GravityPortal,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [12.5, 37.5],
                }),
                raw: HashMap::new(),
            }],
        };
        let mut player = test_player_state();
        player.mode = GameMode::Ball;
        player.x = 120.0;
        player.y = 200.0;
        player.gravity_sign = -1.0;
        player.vy = 6.0;
        let mut touched = HashSet::new();
        let teleports: Vec<&LevelObject> = Vec::new();
        let mut touched_teleports = HashSet::new();
        let mut dual_activate = false;
        let mut dual_deactivate = false;

        apply_portals(
            &level,
            &mut player,
            &mut touched,
            &teleports,
            &mut touched_teleports,
            &mut dual_activate,
            &mut dual_deactivate,
            0,
        );

        assert_eq!(player.gravity_sign, 1.0);
    }

    #[test]
    fn speed_portal_updates_player_speed_and_profile_on_overlap() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 202,
                x: 120.0,
                y: 200.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::SpeedPortal,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [25.5, 28.0],
                }),
                raw: HashMap::new(),
            }],
        };
        let mut player = test_player_state();
        player.mode = GameMode::Ball;
        player.x = 120.0;
        player.y = 200.0;
        let mut touched = HashSet::new();
        let teleports: Vec<&LevelObject> = Vec::new();
        let mut touched_teleports = HashSet::new();
        let mut dual_activate = false;
        let mut dual_deactivate = false;

        apply_portals(
            &level,
            &mut player,
            &mut touched,
            &teleports,
            &mut touched_teleports,
            &mut dual_activate,
            &mut dual_deactivate,
            0,
        );

        let expected_ps = player_speed_for_portal(202);
        let expected_profile = SpeedProfile::for_player_speed(expected_ps);
        assert!(
            (player.player_speed - expected_ps).abs() < 0.0001,
            "speed portal should set player_speed to {expected_ps}, got {}",
            player.player_speed
        );
        assert!((player.speed_multiplier - expected_profile.speed_multiplier).abs() < 0.0001);
        assert!((player.vx - expected_profile.speed_multiplier).abs() < 0.0001);
    }

    #[test]
    fn ball_portal_overlap_uses_outer_hitbox_not_inner_core_box() {
        let portal = test_portal(11, ObjectKind::GravityPortal, 120.0, 200.0);
        let mut player = test_player_state();
        player.mode = GameMode::Ball;
        player.mini = false;
        // Overlaps with outer 30x30 player box, but not a 9x9 inner box.
        player.x = 147.4;
        player.y = 200.0;

        assert!(
            intersects_player(&portal, player),
            "ball should trigger portal when outer hitbox overlaps activation rect"
        );
    }

    #[test]
    fn rotated_gravity_portal_does_not_trigger_far_left_from_center() {
        let portal = LevelObject {
            object_id: 11,
            x: 1327.0,
            y: 307.0,
            rotation: 43.0,
            scale: 1.0,
            scale_x: 1.0,
            scale_y: 1.0,
            groups: Vec::new(),
            kind: ObjectKind::GravityPortal,
            hitbox: Some(HitboxData::Box {
                offset: [0.0, 0.0],
                half_extents: [12.5, 37.5],
            }),
            raw: HashMap::new(),
        };
        let mut player = test_player_state();
        player.mode = GameMode::Ball;
        player.gravity_sign = -1.0;
        player.x = 1280.0; // previously could trigger due rotated-AABB expansion
        player.y = 307.0;

        assert!(
            !intersects_player(&portal, player),
            "rotated gravity portal should not trigger this far left of center"
        );
    }

    #[test]
    fn inverted_gravity_purple_pad_applies_exact_pulse_10_4_for_cube() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 140,
                x: 15.0,
                y: 15.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Pad,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [15.0, 15.0],
                }),
                raw: HashMap::new(),
            }],
        };
        let mut touched = HashSet::new();
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.mini = false;
        player.gravity_sign = 1.0; // inverted gravity => flip_mod = -1
        player.on_ground = false;
        player.vy = 0.0;
        player.x = 15.0;
        player.y = 15.0;

        apply_pads(&level, &mut player, &mut touched, 0);
        assert!(
            (player.vy + 10.4).abs() < 0.001,
            "inverted-gravity purple pad should set cube vy to -10.4, got {}",
            player.vy
        );
    }

    #[test]
    fn cube_pink_orb_uses_docs_yellow_orb_times_point_72() {
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.gravity_sign = -1.0;
        player.y_start = 11.1800318;

        apply_pink_orb(&mut player);
        let expected = 11.1800318 * 0.72;
        assert!(
            (player.vy - expected).abs() < 0.001,
            "cube pink orb should be cube yellow * 0.72 = {expected}, got {}",
            player.vy
        );
    }

    #[test]
    fn blue_orb_sets_point_4_yellow_velocity_then_toggles_gravity() {
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.gravity_sign = -1.0;
        player.y_start = 11.1800318;

        apply_blue_orb(&mut player);
        let expected = 11.1800318 * 0.4;
        assert_eq!(player.gravity_sign, 1.0);
        assert!(
            (player.vy - expected).abs() < 0.001,
            "blue orb should set pre-toggle cube yellow * 0.4 = {expected}, got {}",
            player.vy
        );
    }

    #[test]
    fn robot_red_orb_uses_docs_yellow_orb_times_1_28() {
        let mut player = test_player_state();
        player.mode = GameMode::Robot;
        player.gravity_sign = -1.0;
        player.y_start = 11.1800318;

        apply_red_orb(&mut player);
        let expected = 11.1800318 * 1.28;
        assert!(
            (player.vy - expected).abs() < 0.001,
            "robot red orb should be cube yellow * 1.28 = {expected}, got {}",
            player.vy
        );
    }

    #[test]
    fn ufo_click_requires_two_ticks_held_before_pulse() {
        let mut player = test_player_state();
        player.mode = GameMode::Ufo;
        player.on_ground = false;
        player.gravity_sign = -1.0;
        player.vy = 0.0;

        step_player(&mut player, true);
        assert!(
            player.vy < 1.0,
            "first held tick should not fire UFO pulse; got vy={}",
            player.vy
        );
        step_player(&mut player, true);
        assert!(
            player.vy < 1.0,
            "second held tick should not fire until following tick; got vy={}",
            player.vy
        );
        step_player(&mut player, true);
        assert!(
            player.vy > 6.5,
            "third tick should fire UFO pulse after two held ticks; got vy={}",
            player.vy
        );
    }

    #[test]
    fn ship_yellow_pad_sets_followup_velocity_to_8_next_tick() {
        let mut player = test_player_state();
        player.mode = GameMode::Ship;
        player.gravity_sign = -1.0;
        let flip_mod = player.flip_mod();

        apply_yellow_pad(&mut player, flip_mod, 0.0, 1.0);
        assert!((player.vy - 16.0).abs() < 0.001);
        step_player(&mut player, false);
        assert!(
            (player.vy - 8.0).abs() < 0.001,
            "ship yellow pad should clamp to 8G on following tick; got {}",
            player.vy
        );
    }

    #[test]
    fn ufo_yellow_pad_sets_followup_velocity_to_8_next_tick() {
        let mut player = test_player_state();
        player.mode = GameMode::Ufo;
        player.gravity_sign = -1.0;
        let flip_mod = player.flip_mod();

        apply_yellow_pad(&mut player, flip_mod, 0.0, 1.0);
        assert!((player.vy - 16.0).abs() < 0.001);
        step_player(&mut player, false);
        assert!(
            (player.vy - 8.0).abs() < 0.001,
            "UFO yellow pad should clamp to 8G on following tick; got {}",
            player.vy
        );
    }

    #[test]
    fn cube_green_orb_flips_then_applies_yellow_impulse() {
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.gravity_sign = 1.0;
        player.y_start = 11.1800318;

        apply_green_orb(&mut player);
        assert_eq!(player.gravity_sign, -1.0);
        assert!(
            (player.vy - 11.1800318).abs() < 0.001,
            "cube green orb should flip then apply yellow impulse, got {}",
            player.vy
        );
    }

    #[test]
    fn ufo_green_orb_flips_then_applies_yellow_impulse() {
        let mut player = test_player_state();
        player.mode = GameMode::Ufo;
        player.gravity_sign = 1.0;

        apply_green_orb(&mut player);
        assert_eq!(player.gravity_sign, -1.0);
        assert!(
            (player.vy - 8.0).abs() < 0.001,
            "ufo green orb should flip then apply yellow impulse, got {}",
            player.vy
        );
    }

    #[test]
    fn wave_green_orb_toggles_gravity_only() {
        let mut player = test_player_state();
        player.mode = GameMode::Wave;
        player.gravity_sign = -1.0;
        player.vy = 2.5;

        apply_green_orb(&mut player);
        assert_eq!(player.gravity_sign, 1.0);
        assert!(
            (player.vy - 2.5).abs() < 0.001,
            "wave green orb should only toggle gravity, got vy={}",
            player.vy
        );
    }

    #[test]
    fn green_orb_from_inverted_gravity_gains_y_after_hit() {
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.gravity_sign = 1.0; // starts inverted
        player.y_start = 11.1800318;
        player.y = 100.0;

        apply_green_orb(&mut player);
        assert_eq!(player.gravity_sign, -1.0);
        assert!(player.vy > 0.0, "expected positive vy after flip-to-normal");

        let y_before = player.y;
        integrate_player_position(&mut player);
        assert!(
            player.y > y_before,
            "player should gain y immediately after green orb, before={}, after={}",
            y_before,
            player.y
        );
    }

    #[test]
    fn h_square_is_not_a_physical_block_for_collision_resolution() {
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.gravity_sign = -1.0;
        player.x = 150.0;
        player.y = 179.0;
        player.vy = -5.0;
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 1859,
                x: 150.0,
                y: 150.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Solid,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [15.0, 15.0],
                }),
                raw: HashMap::new(),
            }],
        };

        let outcome = resolve_collisions(&level, 1, &mut player, 0);
        assert!(outcome.is_none());
        assert!(!player.on_ground, "h-square should not ground the player");
        assert!(
            (player.y - 179.0).abs() < 0.001,
            "h-square should not move player by solid MTV; got {}",
            player.y
        );
    }

    #[test]
    fn grounded_cube_lerps_toward_nearest_90_degree_angle() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![],
        };

        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.on_ground = true;
        player.rotation = 92.0;

        let _ = resolve_collisions(&level, 1, &mut player, 0);
        assert!(
            (player.rotation - 91.5).abs() < 0.001,
            "grounded cube should lerp toward nearest 90deg (target=90), got {}",
            player.rotation
        );
    }

    #[test]
    fn airborne_cube_rotation_is_forward_and_slope_detach_reverses() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![],
        };

        // Normal gravity + airborne (not from slope) => clockwise (negative),
        // normalized to near 360.
        let mut airborne = test_player_state();
        airborne.mode = GameMode::Cube;
        airborne.gravity_sign = -1.0;
        airborne.on_ground = false;
        airborne.on_slope = false;
        airborne.rotation = 0.0;
        let _ = resolve_collisions(&level, 1, &mut airborne, 0);
        assert!(
            airborne.rotation > 300.0,
            "normal-gravity airborne cube should rotate clockwise"
        );

        // Detach from slope in normal gravity => reverse spin direction.
        let mut detached = test_player_state();
        detached.mode = GameMode::Cube;
        detached.gravity_sign = -1.0;
        detached.on_ground = false;
        detached.on_slope = true;
        detached.rotation = 0.0;
        let _ = resolve_collisions(&level, 1, &mut detached, 0);
        assert!(
            detached.rotation > 0.0 && detached.rotation < 60.0,
            "slope detach should reverse airborne spin direction"
        );
    }

    #[test]
    fn dash_orb_resets_rotation_and_uses_dash_spin_pacing() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 1704,
                x: 15.0,
                y: 15.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Orb,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [15.0, 15.0],
                }),
                raw: HashMap::new(),
            }],
        };
        let mut touched = HashSet::new();
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.gravity_sign = -1.0;
        player.rotation = 137.0;
        player.on_ground = false;
        player.hold_ticks = 1;
        player.x = 15.0;
        player.y = 15.0;

        apply_orbs(&level, &mut player, &mut touched, 0, true);
        assert!(
            player.rotation.abs() < 0.001,
            "dash orb should reset normal-gravity rotation to 0deg"
        );
        assert!(
            player.dash_rotation_blocks_remaining > 0.0,
            "dash orb should enable dash spin override window"
        );

        let empty = Level {
            header: HashMap::new(),
            objects: vec![],
        };
        let _ = resolve_collisions(&empty, 1, &mut player, 0);
        assert!(
            player.rotation > 300.0,
            "dash spin in normal gravity should rotate clockwise"
        );
    }

    #[test]
    fn green_dash_click_starts_dash_immediately() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 1704,
                x: 15.0,
                y: 15.0,
                rotation: 80.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Orb,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [15.0, 15.0],
                }),
                raw: HashMap::new(),
            }],
        };
        let mut touched = HashSet::new();
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.gravity_sign = -1.0;
        player.rotation = 33.0;
        player.vy = 4.0;
        player.hold_ticks = 1;
        player.x = 15.0;
        player.y = 15.0;

        apply_orbs(&level, &mut player, &mut touched, 0, true);
        assert!(
            (player.rotation - 70.0).abs() < 0.001,
            "green dash should start immediately and clamp 80deg to 70deg, got {}",
            player.rotation
        );
        assert!(
            player.dash_rotation_blocks_remaining > 0.0,
            "green dash click should start dash state immediately"
        );
        assert!(
            player.vy.abs() < 0.001,
            "green dash start should zero vy, got {}",
            player.vy
        );
    }

    #[test]
    fn green_dash_click_clamps_angle() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 1704,
                x: 15.0,
                y: 15.0,
                rotation: 140.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Orb,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [15.0, 15.0],
                }),
                raw: HashMap::new(),
            }],
        };
        let mut touched = HashSet::new();
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.gravity_sign = -1.0;
        player.rotation = 0.0;
        player.hold_ticks = 0;
        player.x = 15.0;
        player.y = 15.0;

        apply_orbs(&level, &mut player, &mut touched, 0, true);
        assert!(
            player.dash_rotation_blocks_remaining > 0.0,
            "green dash click should start dash state"
        );
        assert!(
            (player.rotation - 40.0).abs() < 0.001,
            "green dash angle should remap from 140 to 40 degrees, got {}",
            player.rotation
        );
    }

    #[test]
    fn pink_dash_click_starts_dash_and_toggles_gravity() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 1751,
                x: 15.0,
                y: 15.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Orb,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [15.0, 15.0],
                }),
                raw: HashMap::new(),
            }],
        };
        let mut touched = HashSet::new();
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.gravity_sign = -1.0;
        player.vy = 10.0;
        player.hold_ticks = 1;
        player.x = 15.0;
        player.y = 15.0;

        apply_orbs(&level, &mut player, &mut touched, 0, true);
        assert_eq!(player.gravity_sign, 1.0);
        assert!(
            player.vy.abs() < 0.001,
            "pink dash click should start dash and zero vy, got {}",
            player.vy
        );
        assert!(
            player.dash_rotation_blocks_remaining > 0.0,
            "pink dash click should start dash state"
        );
    }

    #[test]
    fn pink_dash_click_works_without_hold_threshold() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 1751,
                x: 15.0,
                y: 15.0,
                rotation: 20.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Orb,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [15.0, 15.0],
                }),
                raw: HashMap::new(),
            }],
        };
        let mut touched = HashSet::new();
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.gravity_sign = -1.0;
        player.hold_ticks = 0;
        player.x = 15.0;
        player.y = 15.0;

        apply_orbs(&level, &mut player, &mut touched, 0, true);
        assert_eq!(player.gravity_sign, 1.0);
        assert!(
            player.dash_rotation_blocks_remaining > 0.0,
            "pink dash click should start dash state without hold threshold"
        );
    }

    #[test]
    fn releasing_click_cancels_active_dash_immediately() {
        let mut player = test_player_state();
        player.dash_rotation_blocks_remaining = 10.0;
        player.vy = 0.0;
        player.jump_buffered = true;
        player.was_jump_buffered = true;

        step_player(&mut player, false);

        assert!(
            player.dash_rotation_blocks_remaining.abs() < 0.001,
            "releasing click should cancel active dash"
        );
    }

    #[test]
    fn cube_head_hit_on_non_h_square_is_lethal() {
        let mut player = test_player_state();
        player.x = 15.0;
        player.y = 15.0;
        player.vy = 2.0;
        player.gravity_sign = -1.0;
        player.mode = GameMode::Cube;
        // Preserve the original world-space geometry of this test:
        // collision used to read box position as `(object.x + 15, object.y + 15)`,
        // so the box was at world (15, 15). Now collision uses object
        // position directly, so we put the object at (15, 15) directly.
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 1,
                x: 15.0,
                y: 15.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Solid,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [15.0, 15.0],
                }),
                raw: HashMap::new(),
            }],
        };

        let outcome = resolve_collisions(&level, 10, &mut player, 0);
        assert!(matches!(
            outcome,
            Some(SimulationOutcome::Died {
                reason,
                object_id: Some(1),
                ..
            }) if reason == "ceiling hit"
        ));
    }

    #[test]
    fn cube_can_graze_ceiling_without_dying_when_core_misses() {
        let mut player = test_player_state();
        player.x = 44.6;
        player.y = 15.0;
        player.vy = 2.0;
        player.gravity_sign = -1.0;
        player.mode = GameMode::Cube;
        // See `cube_head_hit_on_non_h_square_is_lethal` for the +15
        // adjustment rationale (preserving original world geometry).
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 1,
                x: 15.0,
                y: 15.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Solid,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [15.0, 15.0],
                }),
                raw: HashMap::new(),
            }],
        };

        let outcome = resolve_collisions(&level, 10, &mut player, 0);
        assert!(
            outcome.is_none(),
            "corner graze should not be lethal when core hitbox misses"
        );
    }

    #[test]
    fn mini_cube_floor_contact_uses_outer_hitbox_even_when_side_overlap_is_shallower() {
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.mini = true;
        player.x = 171.0;
        player.y = 166.0;
        player.vy = -5.0;
        player.gravity_sign = -1.0;

        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 1,
                x: 150.0,
                y: 150.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Solid,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [15.0, 15.0],
                }),
                raw: HashMap::new(),
            }],
        };

        let outcome = resolve_collisions(&level, 10, &mut player, 0);
        assert!(outcome.is_none(), "floor contact should not be lethal");
        assert!(
            (player.y - 174.0).abs() < 0.001,
            "mini cube should snap to block top immediately; got y={}",
            player.y
        );
        assert!(player.on_ground);
    }

    #[test]
    fn cube_floor_snap_still_works_with_nonzero_cooldown_without_slope_carry() {
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.mini = false;
        player.x = 15.0;
        player.y = 149.0;
        player.vy = -2.0;
        player.gravity_sign = -1.0;
        player.on_ground = false;
        // Cooldown can be nonzero near slope transitions. It should not block
        // plain outer-hitbox floor landing when there is no upward slope carry.
        player.slope_contact_cooldown = 1;
        player.slope_exit_vy = 0.0;
        player.on_slope = false;

        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 1,
                x: 15.0,
                y: 120.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Solid,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [15.0, 15.0],
                }),
                raw: HashMap::new(),
            }],
        };

        let outcome = resolve_collisions(&level, 10, &mut player, 0);
        assert!(outcome.is_none(), "floor landing should not be lethal");
        assert!(
            (player.y - 150.0).abs() < 0.001,
            "cube should snap to block top via outer hitbox; got y={}",
            player.y
        );
        assert!(player.on_ground);
        assert!(player.vy.abs() < 0.001);
    }

    #[test]
    fn mini_cube_ceiling_hit_without_h_square_is_lethal_when_core_overlaps() {
        let mut player = test_player_state();
        player.mode = GameMode::Cube;
        player.mini = true;
        player.x = 15.0;
        player.y = -3.0;
        player.vy = 3.0;
        player.gravity_sign = -1.0;

        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 1,
                x: 15.0,
                y: 15.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Solid,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [15.0, 15.0],
                }),
                raw: HashMap::new(),
            }],
        };

        let outcome = resolve_collisions(&level, 10, &mut player, 0);
        assert!(matches!(
            outcome,
            Some(SimulationOutcome::Died {
                reason,
                object_id: Some(1),
                ..
            }) if reason == "ceiling hit"
        ));
    }

    #[test]
    fn cube_outer_overlap_with_block_is_not_lethal_when_opengd_inner_box_misses() {
        let mut player = test_player_state();
        player.x = 1300.85;
        player.y = 274.98;
        player.vy = 10.94;
        player.mode = GameMode::Cube;
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 6,
                x: 1305.0,
                y: 255.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Solid,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [15.0, 15.0],
                }),
                raw: HashMap::new(),
            }],
        };

        let outcome = resolve_collisions(&level, 1001, &mut player, 0);

        assert!(
            outcome.is_none(),
            "outer/light-red overlap should not kill unless the OpenGD inner/blue box touches"
        );
    }

    #[test]
    fn cube_inner_blue_box_touching_block_is_lethal() {
        let mut player = test_player_state();
        player.x = 1301.4;
        player.y = 274.98;
        player.vy = 10.94;
        player.mode = GameMode::Cube;
        // Preserve world geometry: collision used to add +15, +15 to
        // object position, so the box was at world (1320, 270). Now we
        // store the world center directly.
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 6,
                x: 1320.0,
                y: 270.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Solid,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [15.0, 15.0],
                }),
                raw: HashMap::new(),
            }],
        };

        let outcome = resolve_collisions(&level, 1001, &mut player, 0);

        assert!(matches!(
            outcome,
            Some(SimulationOutcome::Died {
                reason,
                object_id: Some(6),
                ..
            }) if reason == "side hit"
        ));
    }

    #[test]
    fn non_rotated_spike_uses_outer_cube_hitbox_not_inner_blue_box() {
        // Per GD Creator School "Advanced Hitboxes": the main (outer
        // 30x30) hitbox is what collides with spikes facing upward.
        // The inner blue/solid hitbox is reserved for solid-block
        // side-hit deaths.
        let mut player = test_player_state();
        player.x = 87.0;
        player.y = 105.0;
        player.mode = GameMode::Cube;
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 8,
                x: 80.0,
                y: 90.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Hazard,
                hitbox: Some(HitboxData::Box {
                    offset: [0.0, 0.0],
                    half_extents: [3.0, 6.0],
                }),
                raw: HashMap::new(),
            }],
        };

        let outcome = resolve_collisions(&level, 20, &mut player, 0);

        assert!(matches!(
            outcome,
            Some(SimulationOutcome::Died {
                reason,
                object_id: Some(8),
                ..
            }) if reason == "hazard"
        ));
    }

    #[test]
    fn axis_aligned_slope_exit_velocity_uses_gdp_player_uphill_sign() {
        let mut player = test_player_state();
        player.x = 15.0;
        // With rotated cube slope contact, place the cube slightly higher so
        // the rotated bottom arc intersects the 1x2 slope hypotenuse.
        player.y = 130.0;
        player.vy = -8.0;
        player.player_speed = 0.9;
        player.vx = 5.77000189;
        player.speed_multiplier = 5.77000189;
        player.on_slope = true;
        player.slope_object = Some(0);
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 1338,
                x: 15.0,
                y: 105.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Slope,
                hitbox: Some(HitboxData::Slope {
                    half_extents: [30.0, 15.0],
                }),
                raw: HashMap::new(),
            }],
        };

        let _ = resolve_collisions(&level, 1, &mut player, 0);

        // GDP canonical slope-exit velocity
        // (`PlayerObject_collidedWithSlopeInternal.cpp` line 266):
        //   slopeYVelocity  = (rect_h * playerSpeed * speedMultiplier) / rect_w
        //   m_slopeVelocity = min(1.12 / slopeAngle, 1.54)
        //                   * slopeYVelocity * flipMod * (playerUphill ? -1 : 1)
        // Fixture: half_extents [30, 15] -> width=60, height=30,
        // angle=atan2(15,30)~=0.4636 rad, normal gravity, moving right
        // (playerUphill=false). Expected upward magnitude:
        //   min(1.12/0.4636, 1.54) * (30 * 0.9 * 5.77000189) / 60
        //   = 1.54 * 2.59650... ~= 3.99861
        let rect_h = 30.0_f32;
        let rect_w = 60.0_f32;
        let slope_angle = 15.0_f32.atan2(30.0_f32);
        let expected = (1.12_f32 / slope_angle).min(1.54)
            * (rect_h * player.player_speed * player.speed_multiplier)
            / rect_w;
        assert!(
            (player.slope_exit_vy - expected).abs() < 0.0001,
            "expected slope_exit_vy={expected}, got {}",
            player.slope_exit_vy
        );
    }

    #[test]
    fn ball_uses_cube_slope_exit_velocity() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![LevelObject {
                object_id: 1338,
                x: 15.0,
                y: 105.0,
                rotation: 0.0,
                scale: 1.0,
                scale_x: 1.0,
                scale_y: 1.0,
                groups: Vec::new(),
                kind: ObjectKind::Slope,
                hitbox: Some(HitboxData::Slope {
                    half_extents: [30.0, 15.0],
                }),
                raw: HashMap::new(),
            }],
        };
        let mut cube = test_player_state();
        cube.mode = GameMode::Cube;
        cube.x = 15.0;
        cube.y = 130.0;
        cube.vy = -8.0;
        cube.on_slope = true;
        cube.slope_object = Some(0);

        let mut ball = cube;
        ball.mode = GameMode::Ball;

        let cube_outcome = resolve_collisions(&level, 1, &mut cube, 0);
        let ball_outcome = resolve_collisions(&level, 1, &mut ball, 0);

        assert!(cube_outcome.is_none());
        assert!(ball_outcome.is_none());
        assert_eq!(cube.on_slope, ball.on_slope);
        assert!((cube.y - ball.y).abs() < 0.001);
        assert!(
            (cube.slope_exit_vy - ball.slope_exit_vy).abs() < 0.001,
            "ball slope carry should match cube: cube={} ball={}",
            cube.slope_exit_vy,
            ball.slope_exit_vy
        );
    }

    fn test_player_state() -> PlayerState {
        PlayerState {
            x: 0.0,
            y: 105.0,
            vx: 5.77,
            vy: 0.0,
            mode: GameMode::Cube,
            gravity_sign: -1.0,
            mini: false,
            player_speed: 0.9,
            speed_multiplier: 5.77,
            gravity: 0.9582,
            y_start: 11.18,
            vehicle_size: 1.0,
            on_ground: false,
            was_jump_buffered: false,
            jump_buffered: false,
            state_ring_jump: false,
            on_slope: false,
            slope_exit_vy: 0.0,
            slope_exit_vx: 0.0,
            slope_contact_cooldown: 0,
            slope_object: None,
            slope_is_current_top: false,
            slope_prev_radius: 15.0,
            rotation: 0.0,
            is_accelerating: false,
            snapped_object: None,
            snap_distance: 0.0,
            dash_rotation_blocks_remaining: 0.0,
            dash_angle_deg: 0.0,
            hold_ticks: 0,
            pending_yvel_next_tick: 0.0,
        }
    }

    fn test_portal(object_id: u32, kind: ObjectKind, x: f32, y: f32) -> LevelObject {
        LevelObject {
            object_id,
            x,
            y,
            rotation: 0.0,
            scale: 1.0,
            scale_x: 1.0,
            scale_y: 1.0,
            groups: Vec::new(),
            kind,
            hitbox: None,
            raw: HashMap::new(),
        }
    }

    fn test_blue_pad(rotation: f32) -> LevelObject {
        LevelObject {
            object_id: 67,
            x: 15.0,
            y: 15.0,
            rotation,
            scale: 1.0,
            scale_x: 1.0,
            scale_y: 1.0,
            groups: Vec::new(),
            kind: ObjectKind::Pad,
            hitbox: Some(HitboxData::Box {
                offset: [0.0, 0.0],
                half_extents: [15.0, 15.0],
            }),
            raw: HashMap::new(),
        }
    }

    fn test_yellow_pad(rotation: f32) -> LevelObject {
        LevelObject {
            object_id: 35,
            x: 15.0,
            y: 15.0,
            rotation,
            scale: 1.0,
            scale_x: 1.0,
            scale_y: 1.0,
            groups: Vec::new(),
            kind: ObjectKind::Pad,
            hitbox: Some(HitboxData::Box {
                offset: [0.0, 0.0],
                half_extents: [15.0, 15.0],
            }),
            raw: HashMap::new(),
        }
    }

    fn test_solid(object_id: u32, x: f32, y: f32) -> LevelObject {
        LevelObject {
            object_id,
            x,
            y,
            rotation: 0.0,
            scale: 1.0,
            scale_x: 1.0,
            scale_y: 1.0,
            groups: Vec::new(),
            kind: ObjectKind::Solid,
            hitbox: None,
            raw: HashMap::new(),
        }
    }

    fn test_circle_hazard(object_id: u32, x: f32, y: f32, radius: f32) -> LevelObject {
        LevelObject {
            object_id,
            x,
            y,
            rotation: 0.0,
            scale: 1.0,
            scale_x: 1.0,
            scale_y: 1.0,
            groups: Vec::new(),
            kind: ObjectKind::Hazard,
            hitbox: Some(HitboxData::Circle { radius }),
            raw: HashMap::new(),
        }
    }
}
