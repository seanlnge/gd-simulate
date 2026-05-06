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
const GROUND_PROBE_HEIGHT: f32 = 0.1;
const GROUND_PROBE_DISTANCE: f32 = 0.0;

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
}

impl PlayerState {
    fn flip_mod(&self) -> f32 {
        -self.gravity_sign
    }

    fn player_half(&self) -> f32 {
        if self.mini { 7.5 } else { 15.0 }
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
    reject_unsupported(level)?;

    let speed_profile = SpeedProfile::for_player_speed(0.9);
    let initial = PlayerState {
        x: 0.0,
        y: 105.0,
        vx: speed_profile.speed_multiplier,
        vy: 0.0,
        mode: GameMode::Cube,
        gravity_sign: -1.0,
        mini: false,
        player_speed: 0.9,
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
    };

    let mut player = initial;
    let mut partner: Option<PlayerState> = None;

    let mut trace: Vec<TraceFrame> = Vec::new();
    let mut touched_pads: HashSet<usize> = HashSet::new();
    let mut touched_portals: HashSet<usize> = HashSet::new();
    let mut touched_orbs: HashSet<(usize, u8)> = HashSet::new(); // (object_idx, which_player)
    let mut touched_teleports: HashSet<usize> = HashSet::new();

    // Precompute teleport portal pairs: entry (747) paired with nearest exit (749).
    let teleport_exits: Vec<&LevelObject> = level
        .objects
        .iter()
        .filter(|o| o.object_id == 749)
        .collect();

    for tick in 0..config.max_ticks {
        let pressed = clicks.is_pressed(tick);
        refresh_ground_probe(level, &mut player);
        step_player(&mut player, pressed);
        if let Some(p2) = partner.as_mut() {
            refresh_ground_probe(level, p2);
            step_player(p2, pressed);
        }

        // Collision / portal / pad / orb handling for both players.
        let old_x = player.x;
        player.x += player.vx * DT * TIME_TO_FRAMES * player.player_speed;
        player.y += player.vy * DT * TIME_TO_FRAMES * VERTICAL_SLOW;
        if let Some(p2) = partner.as_mut() {
            p2.x += p2.vx * DT * TIME_TO_FRAMES * p2.player_speed;
            p2.y += p2.vy * DT * TIME_TO_FRAMES * VERTICAL_SLOW;
        }

        // Portals (incl. mirror no-op), size, speed, gravity, teleport, dual.
        let mut dual_activate = false;
        let mut dual_deactivate = false;
        apply_portals(
            level,
            &mut player,
            &mut touched_portals,
            &teleport_exits,
            &mut touched_teleports,
            &mut dual_activate,
            &mut dual_deactivate,
            0,
        );
        if let Some(p2) = partner.as_mut() {
            apply_portals(
                level,
                p2,
                &mut touched_portals,
                &teleport_exits,
                &mut touched_teleports,
                &mut dual_activate,
                &mut dual_deactivate,
                1,
            );
        }
        if dual_activate && partner.is_none() {
            partner = Some(make_partner(&player));
        }
        if dual_deactivate {
            partner = None;
        }

        // Pre-collision trigger pass for blue pads. In real GD, embedded
        // gravity pads can fire before the enclosing block resolves.
        apply_blue_pads_pre_collision(level, &mut player, &mut touched_pads, 0);
        if let Some(p2) = partner.as_mut() {
            apply_blue_pads_pre_collision(level, p2, &mut touched_pads, 1);
        }

        // Collisions first (mirrors gdp::collidedWithObjectInternal path for
        // normal block landings and side-hit logic).
        if let Some(outcome) = resolve_collisions(level, tick, &mut player, 0) {
            trace.push(make_trace_frame(tick, pressed, player, partner));
            return Ok(SimulationRun { outcome, trace });
        }
        if let Some(p2) = partner.as_mut() {
            if let Some(outcome) = resolve_collisions(level, tick, p2, 1) {
                trace.push(make_trace_frame(tick, pressed, player, Some(*p2)));
                return Ok(SimulationRun { outcome, trace });
            }
        }

        // Pads + orbs (touch / press-start activated). Run after collision so
        // the floor under a pad doesn't clamp the impulse vy back to 0.
        apply_pads(level, &mut player, &mut touched_pads, 0);
        apply_orbs(level, &mut player, &mut touched_orbs, 0, pressed);
        if let Some(p2) = partner.as_mut() {
            apply_pads(level, p2, &mut touched_pads, 1);
            apply_orbs(level, p2, &mut touched_orbs, 1, pressed);
        }

        trace.push(make_trace_frame(tick, pressed, player, partner));

        // Completion check.
        if end_reached(level, old_x, player.x) {
            let outcome = SimulationOutcome::Completed {
                tick,
                time: tick as f32 / 240.0,
                state: player,
            };
            return Ok(SimulationRun { outcome, trace });
        }
    }

    let outcome = SimulationOutcome::Timeout {
        tick: config.max_ticks,
        time: config.max_ticks as f32 / 240.0,
        state: player,
    };
    Ok(SimulationRun { outcome, trace })
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
    partner
}

fn step_player(player: &mut PlayerState, pressed: bool) {
    let press_start = pressed && !player.jump_buffered;
    if !pressed {
        player.state_ring_jump = false;
    } else if press_start {
        // OpenGD keeps `_queuedHold` for a press that starts while airborne,
        // allowing rings to fire when touched later in the same hold.
        player.state_ring_jump = !player.on_ground;
    }
    player.was_jump_buffered = player.jump_buffered;
    player.jump_buffered = pressed;
    apply_mode_physics(player, pressed);
}

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

// ---------- Mode physics (port of gdp::PlayerObject::updateJump) ----------

fn apply_mode_physics(player: &mut PlayerState, pressed: bool) {
    let flip_mod = player.flip_mod();
    let size_scale = if player.vehicle_size == 1.0 { 1.0 } else { 0.8 };

    match player.mode {
        GameMode::Cube => update_cube(player, pressed, flip_mod),
        GameMode::Ship => update_ship(player, pressed, flip_mod),
        GameMode::Ball => update_ball(player, pressed, flip_mod),
        GameMode::Ufo => update_ufo(player, pressed, flip_mod, size_scale),
        GameMode::Wave => update_wave(player, pressed, flip_mod, size_scale),
        GameMode::Robot => update_robot(player, pressed, flip_mod),
        GameMode::Spider => update_spider(player, pressed, flip_mod),
        GameMode::Swing => update_swing(player, pressed, flip_mod),
    }

    match player.mode {
        // GDP `updateJump` lines 282-303 only run the flyer vy clamp
        // `if (!m_isAccelerating && !m_isDart)`. Clamping while
        // accelerating instantly snuffs out pad/orb boosts in the air,
        // which makes the ship feel like it cannot be boosted. We don't
        // yet track `m_isDart`, but treating it as false matches all
        // non-dart ship gameplay.
        GameMode::Ship | GameMode::Ufo | GameMode::Wave => {
            if !player.is_accelerating {
                clamp_opengd_flyer_y_velocity(player);
            }
        }
        GameMode::Swing => {
            let clamp = 8.0 / size_scale;
            player.vy = player.vy.clamp(-clamp, clamp);
        }
        _ => player.vy = player.vy.clamp(-15.0, 15.0),
    }
}

fn update_cube(player: &mut PlayerState, pressed: bool, flip_mod: f32) {
    // OpenGD `updateJump`: `playerSize = _mini ? 0.8f : 1.0f` for non-flyer paths.
    let jump_scale = if player.mini { 0.8 } else { 1.0 };

    if player.on_ground && pressed {
        let base_jump_vy = player.y_start * jump_scale * flip_mod;
        player.vy = base_jump_vy;
        // Jump bonus from slope velocity applies ONLY while the non-rotated
        // outer hitbox is still touching the slope (`player.on_slope`).
        // Once detached, the cube is no longer affected by the slope, so
        // the post-detach `slope_contact_cooldown > 0` carry-window is gone.
        if player.on_slope && player.slope_exit_vy * flip_mod > 0.0 {
            player.vy += player.slope_exit_vy * 0.25;
            let cap_along_flip = base_jump_vy.abs() * 1.4;
            if player.vy * flip_mod > cap_along_flip {
                player.vy = cap_along_flip * flip_mod;
            }
        }
        player.on_ground = false;
    } else if player.on_ground {
        player.vy = 0.0;
        // GDP `boostPlayer` sets `m_isAccelerating = true`; per
        // `updateJump.cpp` the flag is cleared (for non-flying modes) on
        // the next ground contact so the next plain jump uses ordinary
        // gravity. Clearing here matches that observable behavior.
        player.is_accelerating = false;
    } else {
        // Canonical GDP cube airborne gravity (`updateJump.cpp` lines
        // 419-477, the *non-flying* `else` branch):
        //
        //   float_b = (isBall || isSpider || isSwing) ? 0.6
        //           : isRobot                          ? 0.9
        //           : 1.0                              // <- cube
        //
        //   if (m_maybeIsBoosted) {
        //     float_d = flipMod * float_c * dt * float_b;
        //     addToYVelocity(-float_d);                       // line 429
        //   } else {
        //     addToYVelocity(-float_c * dt * flipMod * float_b); // line 454
        //   }
        //   // then clamp y-vel to [-15, +15] per `m_isUpsideDown`
        //   // (lines 456-460).
        //
        // For cube both branches collapse to the same per-substep delta:
        //   delta_vy = -flipMod * g * dtSlow * 1.0
        // i.e. a single flat factor with no button-held / accelerating /
        // falling-bugged splits. (Earlier code used `1.08 / 0.2`, which
        // was misread from the *ship* branch lines 237-303 - those
        // `v41 / v51 / v52` factors do not apply to cube.)
        //
        // `pressed` and `is_accelerating` are intentionally NOT read
        // here: the cube branch never branches on them. They remain
        // tracked on `PlayerState` for future ship/ufo gravity work,
        // where they *are* canonically used.
        //
        // Our `dtSlow` is `SUBSTEP_TO_FRAME * VERTICAL_SLOW` (60 Hz frame
        // units * 0.9), matching gd's `dt * 0.9` slow-time pass-through.
        let _ = pressed; // GDP cube gravity is button-independent.
        let float_b = 1.0_f32;
        player.vy += player.gravity * -flip_mod * SUBSTEP_TO_FRAME * VERTICAL_SLOW * float_b;
        // Lines 457/459: clamp to [-15, 15], with the side-of-zero
        // depending on `m_isUpsideDown` (tracked here as `gravity_sign`).
        if player.gravity_sign < 0.0 {
            player.vy = player.vy.max(-15.0);
        } else {
            player.vy = player.vy.min(15.0);
        }
    }
}

fn update_ship(player: &mut PlayerState, pressed: bool, _flip_mod: f32) {
    // Canonical GDP `updateJump` ship branch (lines 121-303 of
    // `PlayerObject_updateJump.cpp`, the `m_isShip` flying path).
    //
    // Key pieces (paraphrased):
    //
    //   usedGravity = 0.9582                          // forced for flyers
    //   float_c     = usedGravity * m_gravityMod
    //   v16         = (m_vehicleSize == 1.0) ? 1.0 : 0.8
    //   v52         = playerIsFallingBugged() ? 0.5 : 0.4
    //
    //   // pre-step: clear m_isAccelerating once vy is back in normal band
    //   //           (this is the canonical "boost expires" gate)
    //   v30, v33 = +/- vy clamp limits scaled by v16
    //   if (vy in [v33, v30] band) m_isAccelerating = false
    //
    //   // press direction multiplier
    //   if jumpBuffered:
    //     v51 = (m_isAccelerating && going-up-rel-gravity) ? 0.8 : -1.0
    //   elif !m_isAccelerating:
    //     v51 = playerIsFallingBugged() ? 0.8 : 1.2
    //   else:
    //     v51 = (uninitialized in GDP; we treat as 0.8 - the floating
    //            stable case where the boost is decaying back into the
    //            normal band; the velocity clamp at the bottom will
    //            reign it in next frame)
    //
    //   // float_c flip override
    //   if m_isAccelerating:
    //     if v51 < 0: float_c = usedGravity      // pressed thrust drops flip
    //   elif jumpBuffered:
    //     float_c = usedGravity                  // ditto
    //   // else: float_c keeps the gravityMod sign
    //
    //   addToYVelocity(-v51 * float_c * dt * flipMod * v52 / v16)
    //
    //   // post-step clamp: only when !m_isAccelerating && !m_isDart
    //   if !m_isAccelerating: clamp vy to GDP's normal-band limits.
    //
    // The previous port used a single `falling` flag that conflated GDP's
    // `playerIsFallingBugged` with "vy > gravity". It also ignored
    // `m_isAccelerating` entirely, so pad/orb boosts immediately got
    // re-clamped to the normal band and the ship felt sluggish after
    // any boost.

    let used_gravity = 0.9582_f32;
    // GDP `m_gravityMod = m_isUpsideDown ? -1 : 1`. Our `gravity_sign`
    // is `-1` for normal and `+1` for flipped (see PlayerState comment),
    // so `m_gravityMod = -gravity_sign`. For the ship, `flipMod()` and
    // `m_gravityMod` are the same value (no swing-style decoupling).
    let gravity_mod = -player.gravity_sign;
    let flip_mod = gravity_mod;
    // GDP `updateJump.cpp` lines 117-130: `v16` is initially `0.8`/`1.0`
    // by mini, but the flyer branch *overwrites* mini-`v16` to `0.85`
    // (lines 128-130). `opengd_flyer_player_size` returns this final
    // flyer-mini value; using the cube-style `0.8` here is wrong.
    let v16 = opengd_flyer_player_size(player);

    // GDP "boost decayed" gate (lines 133-145): clear m_isAccelerating
    // once vy slides back into the normal band. v30 / v33 are the same
    // limits used by the post-step clamp.
    let (v30, v33) = if player.vehicle_size == 1.0 {
        (8.0 / v16, -6.4 / v16)
    } else {
        // mini ship has its own band (lines 128-130).
        (9.4118_f32, -7.5294_f32)
    };
    if player.gravity_sign > 0.0 {
        // upside-down: bands are mirrored.
        if player.vy <= 0.0 && player.vy > -v30 {
            player.is_accelerating = false;
        }
        if player.vy >= 0.0 && player.vy < -v33 {
            player.is_accelerating = false;
        }
    } else {
        if player.vy >= 0.0 && player.vy < v30 {
            player.is_accelerating = false;
        }
        if player.vy <= 0.0 && player.vy > v33 {
            player.is_accelerating = false;
        }
    }

    // We don't yet track playerIsFallingBugged - it requires porting
    // the GDP gate from the bottom of `updateJump`. Treated as false
    // for now; v52 stays at 0.4. This is the dominant path for normal
    // ship gameplay so the parity loss is small (it kicks in only after
    // certain mid-air collisions).
    let falling_bugged = false;
    let v52 = if falling_bugged { 0.5 } else { 0.4 };

    // GDP "going up relative to gravity" check.
    let going_up_rel_gravity = if player.gravity_sign > 0.0 {
        // upside-down: "up" means toward the (flipped) ceiling, i.e. vy <= 0
        player.vy <= 0.0
    } else {
        player.vy >= 0.0
    };

    let v51 = if pressed {
        if player.is_accelerating && going_up_rel_gravity {
            0.8
        } else {
            -1.0
        }
    } else if !player.is_accelerating {
        if falling_bugged { 0.8 } else { 1.2 }
    } else {
        // jumpBuffered=false, m_isAccelerating=true: GDP leaves v51
        // uninitialized. The boost decay branch above will clear
        // is_accelerating once the velocity is back in band, so this
        // codepath is short-lived. 0.8 is a stable hold value.
        0.8
    };

    let mut float_c = used_gravity * gravity_mod;
    if player.is_accelerating {
        if v51 < 0.0 {
            float_c = used_gravity;
        }
    } else if pressed {
        float_c = used_gravity;
    }

    player.vy +=
        -v51 * float_c * SUBSTEP_TO_FRAME * VERTICAL_SLOW * flip_mod * v52 / v16;
}

fn opengd_flyer_player_size(player: &PlayerState) -> f32 {
    if player.mini { 0.85 } else { 1.0 }
}

fn clamp_opengd_flyer_y_velocity(player: &mut PlayerState) {
    let player_size = opengd_flyer_player_size(player);
    let upper_velocity_limit = 8.0 / player_size;
    let lower_velocity_limit = -6.4 / player_size;

    if player.gravity_sign < 0.0 {
        if player.vy <= lower_velocity_limit {
            player.vy = lower_velocity_limit;
        }
        if player.vy >= upper_velocity_limit {
            player.vy = upper_velocity_limit;
        }
    } else {
        if player.vy <= -upper_velocity_limit {
            player.vy = -upper_velocity_limit;
        }
        let flipped_upper_limit = 6.4 / player_size;
        if player.vy >= flipped_upper_limit {
            player.vy = flipped_upper_limit;
        }
    }
}

fn update_ball(player: &mut PlayerState, pressed: bool, flip_mod: f32) {
    if player.on_ground && pressed && !player.was_jump_buffered {
        player.gravity_sign = -player.gravity_sign;
        player.on_ground = false;
        player.vy = player.vy * 0.6; // gdp: m_yVelocity * 0.600000024
    } else {
        player.vy += 0.9582 * -flip_mod * SUBSTEP_TO_FRAME; // gdp flyer/ball gravity
    }
}

fn update_ufo(player: &mut PlayerState, pressed: bool, flip_mod: f32, size: f32) {
    if pressed && !player.was_jump_buffered {
        player.vy = if size == 1.0 { 7.0 } else { 8.0 } * flip_mod;
    }
    player.vy += 0.9582 * -flip_mod * SUBSTEP_TO_FRAME;
}

fn update_wave(player: &mut PlayerState, pressed: bool, flip_mod: f32, size: f32) {
    let magnitude = player.speed_multiplier;
    let factor = if size == 1.0 { 1.0 } else { 2.0 };
    let sign = if pressed { 1.0 } else { -1.0 };
    player.vy = magnitude * factor * sign * flip_mod;
}

fn update_robot(player: &mut PlayerState, pressed: bool, flip_mod: f32) {
    if player.on_ground && pressed && !player.was_jump_buffered {
        player.vy = player.y_start * 0.5 * flip_mod;
        player.on_ground = false;
    } else if pressed && player.vy * flip_mod > 0.0 {
        player.vy += player.gravity * 0.5 * flip_mod * SUBSTEP_TO_FRAME;
    } else {
        player.vy += player.gravity * -flip_mod * SUBSTEP_TO_FRAME;
    }
}

fn update_spider(player: &mut PlayerState, pressed: bool, flip_mod: f32) {
    if player.on_ground && pressed && !player.was_jump_buffered {
        player.gravity_sign = -player.gravity_sign;
        player.vy = 0.0;
        player.on_ground = false;
    } else {
        player.vy += player.gravity * -flip_mod * SUBSTEP_TO_FRAME;
    }
}

fn update_swing(player: &mut PlayerState, pressed: bool, flip_mod: f32) {
    let accel = player.gravity * 0.4 * SUBSTEP_TO_FRAME;
    if pressed {
        player.vy += accel * flip_mod;
    } else {
        player.vy -= accel * flip_mod;
    }
}

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
        200 => 0.7,
        201 => 0.9,
        202 => 1.1,
        203 => 1.3,
        1334 => 1.6,
        _ => 0.9,
    }
}

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
        touched.insert(key);
        let flip_mod = player.flip_mod();
        let rot = object.rotation;
        let sy = object.scale_y;
        match object.object_id {
            35 => apply_yellow_pad(player, flip_mod, rot, sy),
            67 => {
                if gravity_pad_can_activate(player.gravity_sign, rot, sy) {
                    apply_blue_pad(player, rot, sy);
                } else {
                    continue;
                }
            }
            140 => apply_purple_pad(player, flip_mod, rot, sy),
            1332 => apply_red_pad(player, flip_mod, rot, sy),
            _ => {}
        }
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
        let key = object_index + which_player as usize * level.objects.len();
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

/// Pad impulse tables from GD Docs (boomlings.dev/reference/player_physics).
/// "G" in the table = `flip_mod` (gravity-direction scalar) for an unrotated pad.
fn apply_yellow_pad(player: &mut PlayerState, flip_mod: f32, rot_deg: f32, scale_y: f32) {
    let g = flip_mod;
    let mag = match player.mode {
        GameMode::Cube | GameMode::Robot => 16.0 * g,
        GameMode::Ship | GameMode::Ufo => 16.0 * g,
        GameMode::Ball | GameMode::Spider | GameMode::Swing => 9.6 * g,
        GameMode::Wave => 0.0,
    };
    apply_pad_vector(player, mag, rot_deg, scale_y);
}

fn apply_purple_pad(player: &mut PlayerState, flip_mod: f32, rot_deg: f32, scale_y: f32) {
    let g = flip_mod;
    let mag = match player.mode {
        GameMode::Cube | GameMode::Robot => 12.0 * g,
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
    let mag = match player.mode {
        GameMode::Cube | GameMode::Robot => 20.0 * g,
        GameMode::Ship => 10.08 * g,
        GameMode::Ufo => 9.6 * g,
        GameMode::Ball | GameMode::Spider | GameMode::Swing => 12.0 * g,
        GameMode::Wave => 0.0,
    };
    apply_pad_vector(player, mag, rot_deg, scale_y);
}

fn apply_blue_pad(player: &mut PlayerState, rot_deg: f32, scale_y: f32) {
    let flip_mod_before = player.flip_mod();
    let mag = match player.mode {
        GameMode::Cube | GameMode::Robot | GameMode::Ship | GameMode::Ufo => 6.4 * flip_mod_before,
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
    let facing_world_y = scale_y.signum() * theta.cos();
    if gravity_sign < 0.0 {
        facing_world_y >= 0.0
    } else {
        facing_world_y <= 0.0
    }
}

fn intersects_pad_activation(object: &LevelObject, player: PlayerState) -> bool {
    let player_half = player.player_half();
    if let Some(rect) = opengd_pad_activation_rect(object) {
        return intersects_pad_player(rect, player, player_half);
    }
    let Some(rect) = object_rect(object) else {
        return false;
    };
    intersects_pad_player(rect, player, player_half)
}

fn intersects_pad_player(rect: Rect, player: PlayerState, player_half: f32) -> bool {
    // Pads should trigger from the cube's rotated outer hitbox while airborne,
    // not the inner centered "blue box" used for some block lethality checks.
    // Without explicit per-tick visual rotation state, use the cube's
    // circumscribed radius as a robust proxy for rotated-corner contact.
    if player.mode == GameMode::Cube && !player.on_ground {
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

fn opengd_pad_activation_rect(object: &LevelObject) -> Option<Rect> {
    // OpenGD LongData.cpp `_pHitboxes` entries for pad outer bounds. These
    // activation boxes are taller and offset upward relative to gdclone's
    // compact visual/collision boxes, which is why Pop's rotated red pads were
    // missed despite the player intersecting the real pad activation zone.
    let (w, h, ox, oy) = match object.object_id {
        35 => (4.0, 25.0, -12.5, -2.0),
        67 => (6.0, 25.0, -12.5, -3.0),
        140 => (5.0, 25.0, -12.5, -2.5),
        1332 => (7.0, 29.0, -14.5, -3.5),
        _ => return None,
    };
    let sx = object.scale_x.abs();
    let sy = object.scale_y.abs();
    Some(Rect {
        center: [
            object.x + ox * sx + (w * sx) / 2.0,
            object.y + oy * sy + (h * sy) / 2.0,
        ],
        half_extents: [(w * sx) / 2.0, (h * sy) / 2.0],
    })
}

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
    if !(press_start || queued_air_press) {
        return;
    }

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
        touched.insert(key);

        match object.object_id {
            36 => apply_yellow_orb(player),
            84 => apply_gravity_jump_orb(player),
            141 => apply_pink_orb(player),
            1333 => apply_red_orb(player),
            1022 => apply_gravity_jump_orb(player),
            1330 => apply_black_orb(player),
            1594 => apply_green_orb(player),
            // Dash orb (1704) and spider orb (1751) and toggle orb (3004):
            // documented but require additional state machinery. For now fire
            // loudly so the user knows they were hit.
            1704 | 1751 | 3004 => {
                // Minimal behavior: dash orb = preserve vy; spider orb = gravity
                // flip + zero vy; toggle orb = no-op on physics.
                if object.object_id == 1751 {
                    player.gravity_sign = -player.gravity_sign;
                    player.vy = 0.0;
                }
            }
            _ => {}
        }
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
    let base = cube_yellow_velocity(player);
    player.vy = match player.mode {
        GameMode::Cube | GameMode::Ball | GameMode::Robot | GameMode::Spider | GameMode::Swing => {
            match player.mode {
                GameMode::Cube => base,
                GameMode::Ball | GameMode::Spider => base * 0.7,
                GameMode::Robot => base * 0.9,
                GameMode::Swing => base * 0.6,
                _ => base,
            }
        }
        GameMode::Ship | GameMode::Ufo => 8.0 * g,
        GameMode::Wave => player.vy, // docs: no effect
    };
}

fn apply_pink_orb(player: &mut PlayerState) {
    let g = player.flip_mod();
    player.vy = match player.mode {
        GameMode::Cube | GameMode::Robot => 12.0 * g,
        // Keep previous proportional behavior for non-cube modes until we have
        // explicit per-mode parity traces.
        GameMode::Ship => cube_yellow_velocity(player) * 0.37,
        GameMode::Ball | GameMode::Spider => cube_yellow_velocity(player) * 0.7 * 0.77,
        GameMode::Ufo => cube_yellow_velocity(player) * 0.42,
        GameMode::Swing => cube_yellow_velocity(player) * 0.6 * 0.72,
        GameMode::Wave => player.vy,
    };
}

fn apply_red_orb(player: &mut PlayerState) {
    let base = cube_yellow_velocity(player);
    player.vy = match player.mode {
        GameMode::Cube | GameMode::Robot => base * 1.38,
        GameMode::Ship => base, // same as cube yellow
        GameMode::Ball | GameMode::Spider => base * 0.7 * 1.34,
        GameMode::Ufo => base * 1.02,
        GameMode::Swing => base * 0.6 * 1.38,
        GameMode::Wave => player.vy,
    };
}

fn apply_gravity_jump_orb(player: &mut PlayerState) {
    // Parity target: gravity ring flips first, then applies yellow-style pulse
    // in the new gravity frame.
    player.gravity_sign = -player.gravity_sign;
    apply_yellow_orb(player);
}

fn apply_green_orb(player: &mut PlayerState) {
    let base = cube_yellow_velocity(player);
    let g = player.flip_mod();
    match player.mode {
        GameMode::Cube | GameMode::Robot => {
            player.vy = base * -1.0;
        }
        GameMode::Ship => {
            player.vy = base * -0.7;
        }
        GameMode::Ufo => {
            player.vy = -8.0 * g;
        }
        GameMode::Ball | GameMode::Spider => {
            player.vy = base * 0.7 * -1.0;
        }
        GameMode::Swing => {
            player.vy = base * 0.7 * -1.0; // docs: "spider yellow * -1"
        }
        GameMode::Wave => {}
    }
    player.gravity_sign = -player.gravity_sign;
}

fn apply_black_orb(player: &mut PlayerState) {
    let g = player.flip_mod();
    // Black orb: yvel = -15G for most modes; ship = -14G (then decays); UFO = -11.2G.
    player.vy = match player.mode {
        GameMode::Cube | GameMode::Ball | GameMode::Robot | GameMode::Spider => -15.0 * g,
        GameMode::Ship | GameMode::Swing => -14.0 * g,
        GameMode::Ufo => -11.2 * g,
        GameMode::Wave => player.vy,
    };
}

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
const OPENGD_CUBE_INNER_HALF: f32 = 3.75;

fn apply_implicit_bounds(player: &mut PlayerState) -> bool {
    let mut grounded = false;
    let player_half = player.player_half();
    let floor_top = IMPLICIT_FLOOR_Y;
    let gravity_down = player.gravity_sign < 0.0;
    if gravity_down {
        // Floor catches the player when falling.
        if player.y - player_half <= floor_top && player.vy <= 0.0 {
            player.y = floor_top + player_half;
            player.vy = 0.0;
            player.on_ground = true;
            grounded = true;
        }
    } else {
        if player.y - player_half <= floor_top && player.vy < 0.0 {
            player.y = floor_top + player_half;
            player.vy = 0.0;
        }
    }
    grounded
}

fn resolve_collisions(
    level: &Level,
    tick: usize,
    player: &mut PlayerState,
    which_player: u8,
) -> Option<SimulationOutcome> {
    if player.y < -1200.0 {
        return Some(SimulationOutcome::Died {
            tick,
            time: tick as f32 / 240.0,
            state: *player,
            object_id: None,
            reason: "floor".to_owned(),
            which_player,
        });
    }

    let player_half = player.player_half();
    // snap_up_threshold from gdp::collidedWithObjectInternal. Non-platformer,
    // non-flying, vehicle-size 1 uses 10; mini uses 5; flyers use 6.
    let snap_threshold: f32 = match player.mode {
        GameMode::Ship | GameMode::Ufo | GameMode::Wave | GameMode::Swing => 6.0,
        _ if player.mini => 5.0,
        _ => 10.0,
    };

    let mut grounded_this_tick = false;
    let was_on_slope = player.on_slope;
    let mut on_slope_this_tick = false;
    let mut best_slope_snap_score: Option<f32> = None;
    // Target rotation (degrees) of the winning slope this tick, fed into the
    // post-pass cube-rotation lerp. None = no slope contact this tick.
    let mut slope_target_rotation_deg: Option<f32> = None;

    // Pass 1: slopes (gdclone triangle + RobTop rotation).
    for (slope_index, object) in nearby_objects(level, player.x) {
        if object.kind != ObjectKind::Slope {
            continue;
        }
        let Some(HitboxData::Slope { half_extents }) = object.hitbox else {
            continue;
        };
        let (hx, hy) = (half_extents[0], half_extents[1]);
        let Some(rect) = object_rect(object) else {
            continue;
        };
        // Three-hitbox split (per the user-specified GD model):
        //   * Non-rotated outer 30x30 (or mini 15x15): selects which slope is
        //     active, gates entry & detach. AABB overlap with the slope's
        //     transformed bounds.
        //   * Rotated outer (circumscribed half = `player_half * SQRT_2`):
        //     interacts with the slope surface (hypotenuse probe + lift).
        // Selection gate using non-rotated outer:
        if !intersects_box_player(rect, *player, player_half) {
            continue;
        }
        // Surface contact: when the cube is rotated to match the slope
        // angle, its 30x30 outer hitbox sits flat on the slope and the
        // perpendicular reach to the slope surface is exactly `player_half`
        // (a flat side of the rotated square against a flat surface). Using
        // a single consistent `player_half` here (instead of the previous
        // flip-flop between `player_half` on entry and `SQRT_2 * player_half`
        // on continuation) eliminates the Y-jump that was the visible Pop
        // slope jitter, while still respecting the rotated-cube model: the
        // cube renders rotated, sits flat on the slope, and snaps smoothly.
        let slope_player_half = player_half;
        // GDP `collidedWithSlopeInternal`: if not already on a slope, require
        // intersection with an exit-rect (obj rect trimmed by 1px vertically)
        // before accepting slope contact.
        if !was_on_slope {
            let exit_half_y = (rect.half_extents[1] - 1.0).max(0.0);
            let exit_rect = Rect {
                center: [rect.center[0], rect.center[1] + 1.0],
                half_extents: [rect.half_extents[0], exit_half_y],
            };
            if !intersects_box_player(exit_rect, *player, slope_player_half) {
                continue;
            }
        }
        let gravity_down = player.gravity_sign < 0.0;
        // GDP slope contact is not just AABB overlap; constrain to points that
        // are near the transformed slope hypotenuse in local space.
        let probe_y = if gravity_down {
            player.y - slope_player_half
        } else {
            player.y + slope_player_half
        };
        let Some((px_l, py_l)) = slope_world_to_local_point(
            object.x,
            object.y,
            object.rotation,
            object.scale_x,
            object.scale_y,
            player.x,
            probe_y,
        ) else {
            continue;
        };
        if !near_slope_hypotenuse(px_l, py_l, hx, hy) {
            continue;
        }
        let wy_opt = rotated_slope_player_world_y(
            object.x,
            object.y,
            object.rotation,
            object.scale_x,
            object.scale_y,
            hx,
            hy,
            player.x,
            player.y,
            slope_player_half,
            gravity_down,
        );
        let Some(wy_raw) = wy_opt else {
            continue;
        };
        let (sx1, sy1) = slope_local_to_world_point(
            object.x,
            object.y,
            object.rotation,
            object.scale_x,
            object.scale_y,
            -hx,
            -hy,
        );
        let (sx2, sy2) = slope_local_to_world_point(
            object.x,
            object.y,
            object.rotation,
            object.scale_x,
            object.scale_y,
            hx,
            hy,
        );
        let slope_dx = sx2 - sx1;
        let slope_dy = sy2 - sy1;
        // World-rotated hyp length is no longer used for player_rad_on_slope
        // (we use the local intrinsic slope angle per GDP). Kept here for
        // potential future debug; suppressed-warning let _.
        let _slope_len = (slope_dx * slope_dx + slope_dy * slope_dy).sqrt().max(1e-4);
        // GDP `playerRadOnSlope = playerRadius / cos(getSlopeAngle())` uses
        // the slope's *intrinsic* angle (from its local hitbox shape:
        // atan2(hy, hx)), not the world-rotated hypotenuse cos. For a 1x2
        // (hx=30, hy=15) the intrinsic angle is 26.57deg regardless of
        // whether the slope is rotated -90deg in world space. The previous
        // world-cos formula yielded rad=33.5 for vertical 1x2 slopes,
        // pushing the snap target out of range of the snap-attach threshold
        // and silently rejecting astrosoda's first slope.
        let local_slope_angle = hy.atan2(hx);
        let local_cos = local_slope_angle.cos().abs().max(1e-4);
        let player_rad_on_slope = slope_player_half / local_cos;
        let (_tx3, ty3) = slope_local_to_world_point(
            object.x,
            object.y,
            object.rotation,
            object.scale_x,
            object.scale_y,
            hx,
            -hy,
        );
        // `slope_floor_top` = is the slope's solid floor on the *upper* Y
        // side of the rect (i.e. cube hangs from underneath, ceiling
        // ramp)? Cross-product of (sx1->sx2) vs (sx1->t3) flips sign with
        // rotation/flipY combinations and incorrectly flags rotated 1x2
        // slopes (e.g. astrosoda's vertical wall slopes) as ceiling
        // slopes. Robust test: compare the right-angle vertex Y to the
        // hypotenuse mid-Y. If t3 is *above* the hypotenuse mid, the
        // solid mass of the triangle sits above and the cube hangs.
        let hyp_mid_y = (sy1 + sy2) / 2.0;
        let slope_floor_top = ty3 > hyp_mid_y;
        let mut wy = wy_raw;
        if !was_on_slope {
            // Progressive GDP parity: apply first-contact `newPlayerY` clamp
            // by inferred slopeFloorTop polarity, while keeping existing
            // slope-to-slope carry behavior untouched.
            wy = if slope_floor_top {
                let temp = rect.min_y() - slope_player_half;
                wy.max(temp).min(rect.max_y())
            } else {
                let temp = rect.max_y() + slope_player_half;
                wy.min(temp).max(rect.min_y())
            };
        }
        // GDP new-slope transition scalar: when switching between different
        // slope contacts with opposite top/bottom polarity, use
        // `newSlopeScalar = vehicleSize * 20` in the snapped Y solve.
        let is_new_slope = was_on_slope
            && matches!(player.slope_object, Some(prev_idx) if prev_idx != slope_index)
            && player.slope_is_current_top != slope_floor_top;
        if is_new_slope {
            let y_surf = if gravity_down {
                wy_raw - player_rad_on_slope
            } else {
                wy_raw + player_rad_on_slope
            };
            let new_slope_scalar = player.vehicle_size * 20.0;
            let floor_sign = if slope_floor_top { -1.0 } else { 1.0 };
            wy = y_surf + (player_rad_on_slope - new_slope_scalar) * floor_sign;
        }
        // Prevent pathological multi-slope overlaps from teleporting the
        // player onto a faraway surface in one tick. GDP does not use the
        // block snap threshold for slope-to-slope continuity: when the player
        // was already on a slope, collidedWithSlopeInternal compares against
        // the previous slope radius (`onSlopeThreshold`) instead. This larger
        // carry distance is needed for Pop's 26.5° -> 45° -> 63.4° transition.
        let slope_snap_delta = if gravity_down {
            wy - player.y
        } else {
            player.y - wy
        };
        let slope_grade = if slope_dx.abs() > 1e-4 {
            Some(slope_dy / slope_dx)
        } else {
            None
        };
        let player_uphill = slope_grade
            .map(|grade| grade * player.vx * player.flip_mod() < 0.0)
            .unwrap_or(false);
        let float_g = if player_uphill {
            if was_on_slope { 4.0 } else { 1.0 }
        } else {
            0.0
        };
        let prev_rad_on_slope = if was_on_slope {
            player.slope_prev_radius.max(slope_player_half)
        } else {
            player_rad_on_slope
        };
        let on_slope_threshold = prev_rad_on_slope + float_g;
        // Re-attach uses the relaxed `on_slope_threshold` while still in
        // the slope-context window. `slope_contact_cooldown > 0` covers the
        // 1-2 tick gap between back-to-back slope tiles in a chain (Pop's
        // 372 -> 371 -> 372 rotated transition). This is a slope-chaining
        // mechanic, NOT an "affected by previous slope after fully leaving"
        // effect — once the cube fully detaches and the cooldown expires,
        // no further slope influence applies.
        let slope_attach_threshold =
            if was_on_slope || on_slope_this_tick || player.slope_contact_cooldown > 0 {
                on_slope_threshold
            } else {
                snap_threshold
            };
        if slope_snap_delta > slope_attach_threshold {
            continue;
        }
        // GDP-style collidedSlope gate (`bool_h` window): if already above the
        // solved slope Y, allow a narrow re-attach band while moving uphill.
        let bool_h = player_uphill && !is_new_slope && !on_slope_this_tick;
        let continuing_same_slope = was_on_slope
            && matches!(player.slope_object, Some(prev_idx) if prev_idx == slope_index);
        let crossed = if gravity_down {
            player.y <= wy || (bool_h && player.y < wy + float_g) || continuing_same_slope
        } else {
            player.y >= wy || (bool_h && player.y > wy - float_g) || continuing_same_slope
        };
        if crossed {
            // Resolve at most one slope contact per tick: choose the "best"
            // surface in gravity direction to avoid loop-order stair-stepping.
            let snap_score = if gravity_down { wy } else { -wy };
            if let Some(best) = best_slope_snap_score && snap_score <= best {
                continue;
            }
            best_slope_snap_score = Some(snap_score);
            player.slope_object = Some(slope_index);
            player.slope_is_current_top = slope_floor_top;
            player.slope_prev_radius = player_rad_on_slope;
            player.y = wy;
            player.on_ground = true;
            grounded_this_tick = true;
            on_slope_this_tick = true;
            // Target rotation = slope surface angle in world space. In the
            // existing convention `slope_floor_top=true` means the floor is
            // on the top side of the rect (cube hangs underneath = ceiling
            // ramp, requires +180); `false` is the standard ground-side
            // ramp (cube sits on top, rotation = surface angle).
            let raw_angle_deg = slope_dy.atan2(slope_dx).to_degrees();
            let target_deg = if slope_floor_top {
                raw_angle_deg + 180.0
            } else {
                raw_angle_deg
            };
            slope_target_rotation_deg = Some(target_deg);
            // For grounded slope contact, non-flying modes should not keep
            // integrating a large downward velocity while "stuck" to slope.
            // gdp's ground path runs hitGround/updateCollide for these cases.
            // `collidedWithSlopeInternal` also restores pre-hit velocity when
            // it is strongly upward along gravity-opposed direction
            // (`upsideMod * oldVelocity > upsideMod * 5.0`).
            match player.mode {
                GameMode::Cube | GameMode::Robot | GameMode::Spider | GameMode::Ball => {
                    let old_vy = player.vy;
                    let slope_upside_down = (player.gravity_sign > 0.0) != slope_floor_top;
                    if slope_upside_down {
                        // GDP upside-down-slope branch:
                        // keep only velocity moving away from the slope.
                        if old_vy * player.flip_mod() > 0.0 {
                            player.vy = 0.0;
                        } else {
                            player.vy = old_vy;
                        }
                    } else {
                        // GDP non-upside-down branch:
                        // preserve strong upward velocity only.
                        player.vy = 0.0;
                        if old_vy * player.flip_mod() > 5.0 {
                            player.vy = old_vy;
                        }
                    }
                }
                _ => {}
            }
            let slope_grade = transformed_slope_world_grade(
                object.x,
                object.y,
                object.rotation,
                object.scale_x,
                object.scale_y,
                hx,
                hy,
            );
            // Canonical GDP `m_slopeVelocity` port from
            // `PlayerObject_collidedWithSlopeInternal.cpp` line 266:
            //
            //   slopeYVelocity   = (objRect.height * playerSpeed * speedMultiplier) / objRect.width;
            //   m_slopeVelocity  = min(1.12 / slopeAngle, 1.54)
            //                    * slopeYVelocity
            //                    * flipMod
            //                    * (playerUphill ? -1 : 1);
            //   if (flying || ball) m_slopeVelocity *= 0.75;
            //
            // GDP itself only applies `m_slopeVelocity` on a *jump* from
            // (or just-after) a slope (`addToYVelocity(m_slopeVelocity*0.25, 60)`
            // capped at 1.4x base jump in `updateJump.cpp`). Our sim also
            // applies it on detach to approximate the implicit
            // `vy = grade * vx` inertia that GDP gets for free from its
            // slope-Y-tracking position update. This proves out against
            // the recorded `pop.bin` click tape that beat Pop with these
            // values.
            // GDP `m_slopeVelocity` is computed from the slope's *world*
            // bounding rect (`objRect`), not the local hitbox extents.
            // Mixing the two (local-angle multiplier x world-rect ratio)
            // is what made the rotated 1x2 "vertical" slope in Pop launch
            // the cube at ~16 (flat-clamped to 15) instead of the
            // canonical ~10.5: a 1x2 rotated 90deg has world half-extents
            // [15, 30] (steep, atan(2)~=1.107rad, mult=1.012) but local
            // half-extents [30, 15] (gentle, atan2(15,30)=0.464rad,
            // clamped mult=1.54). The world ratio always matches GDP's
            // `objRect.height / objRect.width`.
            let rect_height = rect.half_extents[1] * 2.0;
            let rect_width = (rect.half_extents[0] * 2.0).max(1e-4);
            let slope_angle = (rect_height / rect_width).atan();
            let slope_exit_mult = (1.12_f32 / slope_angle).min(1.54);
            player.slope_exit_vy = slope_grade
                .map(|grade| {
                    let flip_mod = player.flip_mod();
                    let player_uphill = grade * player.vx * flip_mod < 0.0;
                    let slope_dir = if player_uphill { -1.0 } else { 1.0 };
                    let mut slope_velocity = slope_exit_mult
                        * ((rect_height * player.player_speed * player.speed_multiplier)
                            / rect_width)
                        * flip_mod
                        * slope_dir;
                    if matches!(
                        player.mode,
                        GameMode::Ship | GameMode::Ufo | GameMode::Wave | GameMode::Ball
                    ) {
                        slope_velocity *= 0.75;
                    }
                    slope_velocity
                })
                .unwrap_or(0.0);
            // GDP stores scalar slope velocity (`m_slopeVelocity`). Horizontal
            // carry is not applied as a separate slope-exit x impulse.
            player.slope_exit_vx = 0.0;
        }
    }
    player.on_slope = on_slope_this_tick;
    if on_slope_this_tick {
        // `slope_exit_*` updated inside the loop (last matching slope wins).
        // GDP keeps special slope collision context for 0.2 seconds after
        // leaving a slope (`m_totalTime - m_slopeEndTime < 0.2`).
        player.slope_contact_cooldown = 48;
    } else if was_on_slope && (player.slope_exit_vy != 0.0 || player.slope_exit_vx != 0.0) {
        let flip_mod = -player.gravity_sign;
        if player.slope_exit_vy * flip_mod > 0.0
            && player.slope_exit_vy * flip_mod > player.vy * flip_mod
        {
            player.vy = player.slope_exit_vy;
        }
        player.slope_exit_vy = 0.0;
        player.slope_exit_vx = 0.0;
    }
    if !on_slope_this_tick {
        player.slope_object = None;
    }

    // Cube rotation update. While on a slope, lerp toward the slope's
    // surface angle so the rotated outer hitbox sits flat against the
    // hypotenuse (visually smooth, no jitter because the slope angle is
    // constant per slope). Airborne / not-on-slope, ease back toward 0.
    // Other gamemodes manage their own rotation; this is cube-only for now.
    if player.mode == GameMode::Cube {
        if let Some(target) = slope_target_rotation_deg {
            // Choose the equivalent angle within +/- 180 of current rotation
            // so the lerp takes the short way around (avoids 359->0 wrap).
            let mut delta = target - player.rotation;
            while delta > 180.0 {
                delta -= 360.0;
            }
            while delta < -180.0 {
                delta += 360.0;
            }
            const SLOPE_LERP: f32 = 0.4;
            player.rotation += delta * SLOPE_LERP;
        } else if !player.on_ground {
            // Airborne: keep current rotation (GD spins the cube in air; we
            // don't reproduce the full spin here, just hold).
        } else {
            // Grounded on a flat solid: ease back toward 0.
            const RETURN_LERP: f32 = 0.25;
            player.rotation -= player.rotation * RETURN_LERP;
            if player.rotation.abs() < 0.05 {
                player.rotation = 0.0;
            }
        }
    }

    // Pass 2: solids and hazards (and re-process slopes only as no-ops).
    let block_snap: f32 = snap_threshold;

    for (object_index, object) in nearby_objects(level, player.x) {
        let Some(rect) = object_rect(object) else {
            continue;
        };
        if !intersects_box_player(rect, *player, player_half) {
            continue;
        }

        match object.kind {
            ObjectKind::Slope => {
                // Already handled in pass 1.
                continue;
            }
            ObjectKind::Hazard => {
                // OpenGD uses the player outer/main 30x30 box for non-circular
                // hazards (per GD Creator School "Advanced Hitboxes": the main
                // AABB hitbox is what collides with spikes facing upward).
                if lethal_hazard_intersects(object, rect, *player, player_half) {
                    return Some(SimulationOutcome::Died {
                        tick,
                        time: tick as f32 / 240.0,
                        state: *player,
                        object_id: Some(object.object_id),
                        reason: "hazard".to_owned(),
                        which_player,
                    });
                }
            }
            ObjectKind::Solid => {
                // Minimum-translation-vector AABB resolution: pick the axis
                // (horizontal vs vertical) with the smaller penetration.
                // Vertical penetration => land/head; horizontal penetration =>
                // either stair-snap (if block top is within reach) or death.
                let dx = player.x - rect.center[0];
                let dy = player.y - rect.center[1];
                let overlap_x = (player_half + rect.half_extents[0]) - dx.abs();
                let overlap_y = (player_half + rect.half_extents[1]) - dy.abs();
                let gravity_down = player.gravity_sign < 0.0;
                let top_surface = rect.max_y();
                let bottom_surface = rect.min_y();
                let land_y = if gravity_down {
                    top_surface + player_half
                } else {
                    bottom_surface - player_half
                };
                let head_y = if gravity_down {
                    bottom_surface - player_half
                } else {
                    top_surface + player_half
                };

                // Vertical separation is shallower (or equal) → it's a top/bottom hit.
                if overlap_y <= overlap_x {
                    let above_block = (gravity_down && dy > 0.0) || (!gravity_down && dy < 0.0);
                    if above_block {
                        let flip_mod = -player.gravity_sign;
                        if on_slope_this_tick
                            || player.slope_contact_cooldown > 0
                            || was_on_slope && player.vy * flip_mod > 0.0
                        {
                            // GDP preserves upward slope exit motion through
                            // nearby block contacts; otherwise Pop snaps flat
                            // onto the first wall-stack tile and loses the ramp.
                            continue;
                        }
                        // Land on the gravity-up surface.
                        player.y = land_y;
                        player.vy = 0.0;
                        player.on_ground = true;
                        grounded_this_tick = true;
                        if player.mode == GameMode::Cube {
                            check_snap_jump_to_object_for_level(level, player, object_index);
                        }
                    } else {
                        // Head bonk on the gravity-down surface.
                        if player.mode == GameMode::Cube
                            && !ceiling_death_blocked_by_h_square(level, *player)
                            && intersects_box_player(rect, *player, cube_lethal_inner_half(player))
                        {
                            return Some(SimulationOutcome::Died {
                                tick,
                                time: tick as f32 / 240.0,
                                state: *player,
                                object_id: Some(object.object_id),
                                reason: "ceiling hit".to_owned(),
                                which_player,
                            });
                        }
                        player.y = head_y;
                        if (gravity_down && player.vy > 0.0) || (!gravity_down && player.vy < 0.0) {
                            player.vy = 0.0;
                        }
                    }
                } else {
                    // Horizontal penetration. gdp's checkSnapJumpToObject lets
                    // the player ride up onto a block whose top is at most
                    // snap_threshold above the current y. If the block top is
                    // out of reach, GD treats horizontal contact as death.
                    let stair_top_within_reach = if gravity_down {
                        land_y - player.y <= block_snap && land_y - player.y >= 0.0
                    } else {
                        player.y - land_y <= block_snap && player.y - land_y >= 0.0
                    };
                    let descending_toward_surface = if gravity_down {
                        player.vy <= 0.0
                    } else {
                        player.vy >= 0.0
                    };
                    let can_stair_snap = stair_top_within_reach
                        && (descending_toward_surface
                            || player.on_ground
                            || was_on_slope
                            || on_slope_this_tick
                            || player.slope_contact_cooldown > 0);
                    if can_stair_snap {
                        player.y = land_y;
                        player.vy = 0.0;
                        player.on_ground = true;
                        grounded_this_tick = true;
                        if player.mode == GameMode::Cube {
                            check_snap_jump_to_object_for_level(level, player, object_index);
                        }
                    } else if side_hit_is_lethal(
                        rect,
                        *player,
                        player_half,
                        on_slope_this_tick,
                        was_on_slope,
                        player.slope_contact_cooldown,
                    ) {
                        return Some(SimulationOutcome::Died {
                            tick,
                            time: tick as f32 / 240.0,
                            state: *player,
                            object_id: Some(object.object_id),
                            reason: "side hit".to_owned(),
                            which_player,
                        });
                    }
                }
            }
            _ => {}
        }
    }

    if apply_implicit_bounds(player) {
        grounded_this_tick = true;
    }
    if !grounded_this_tick {
        player.on_ground = false;
    }
    if !on_slope_this_tick && player.slope_contact_cooldown > 0 {
        player.slope_contact_cooldown -= 1;
    }
    if player.y >= IMPLICIT_CEILING_DEATH_Y
        && !(player.mode == GameMode::Cube && ceiling_death_blocked_by_h_square(level, *player))
    {
        return Some(SimulationOutcome::Died {
            tick,
            time: tick as f32 / 240.0,
            state: *player,
            object_id: None,
            reason: "ceiling".to_owned(),
            which_player,
        });
    }
    None
}

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
    let Some(rect) = object_rect(object) else {
        return false;
    };
    intersects_box_player(rect, player, player.player_half())
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
    intersects_box_player(rect, player, cube_lethal_inner_half(&player))
}

fn cube_lethal_inner_half(player: &PlayerState) -> f32 {
    if player.mode == GameMode::Cube {
        OPENGD_CUBE_INNER_HALF
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
        };

        update_cube(&mut player, false, 1.0);

        // GDP `updateJump.cpp` cube non-flying branch (lines 419-477):
        //   delta_vy = -flipMod * g * dtSlow * float_b   with float_b = 1.0
        // No state machine - same factor whether button held or not.
        let expected = 10.0 - 0.9582 * SUBSTEP_TO_FRAME * VERTICAL_SLOW * 1.0;
        assert!((player.vy - expected).abs() < 0.0001, "got {}", player.vy);
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
        // Pressed, !is_accelerating: v51 = -1.0, float_c = usedGravity
        // (= 0.9582, no flip override applies because gravity_mod = 1).
        let expected_hold = -(-1.0) * 0.9582 * SUBSTEP_TO_FRAME * VERTICAL_SLOW * 1.0 * 0.4 / 1.0;
        assert!(
            (player.vy - expected_hold).abs() < 0.0001,
            "pressed-thrust should add {expected_hold}, got {}",
            player.vy
        );

        player.vy = 0.0;
        update_ship(&mut player, false, flip_mod);
        // Released, !is_accelerating: v51 = 1.2, float_c =
        // usedGravity * gravity_mod = 0.9582 (gravity_mod = 1 in
        // normal gravity).
        let expected_release_falling =
            -1.2 * 0.9582 * SUBSTEP_TO_FRAME * VERTICAL_SLOW * 1.0 * 0.4 / 1.0;
        assert!(
            (player.vy - expected_release_falling).abs() < 0.0001,
            "release should add {expected_release_falling}, got {}",
            player.vy
        );

        player.vy = 2.0;
        update_ship(&mut player, false, flip_mod);
        // Released, !is_accelerating, vy > 0 (rising): same v51 = 1.2
        // path - GDP does not branch on vy direction outside the
        // `is_accelerating` gate.
        let expected_release_rising =
            2.0 + (-1.2 * 0.9582 * SUBSTEP_TO_FRAME * VERTICAL_SLOW * 1.0 * 0.4 / 1.0);
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
        // Same GDP path as above with v16 = 0.85 (mini-ship flyer
        // override at `updateJump.cpp` lines 128-130) and v52 = 0.4.
        let expected_hold =
            -(-1.0) * 0.9582 * SUBSTEP_TO_FRAME * VERTICAL_SLOW * 1.0 * 0.4 / 0.85;
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
        };

        let grounded = apply_implicit_bounds(&mut player);

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
        };
        let mut touched = HashSet::new();

        apply_orbs(&level, &mut player, &mut touched, 0, true);

        assert!(player.vy > 14.0);
        assert!(!player.state_ring_jump);
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
        };

        let grounded = apply_implicit_bounds(&mut player);

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



