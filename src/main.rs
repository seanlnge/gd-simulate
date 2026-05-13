use std::{
    fs::{self, OpenOptions},
    path::{Path, PathBuf},
};

use clap::{Parser, ValueEnum};
use flate2::{Compression, write::GzEncoder};
use gd_real_sim::{
    collision::slope_local_to_world_point,
    input::ClickTape,
    level::Level,
    object_data::ObjectDatabase,
    save::{decode_level_payload, read_local_levels, select_local_level},
    sim::{DT, LiveSimulationSession, SimulationConfig, SimulationOutcome, simulate_with_trace},
};
use minifb::{Key, KeyRepeat, MouseButton, MouseMode, Window, WindowOptions};
use serde::Serialize;
use std::io::Write;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

#[derive(Debug, Parser)]
#[command(name = "simulate")]
#[command(about = "Parity-first Geometry Dash death/completion simulator")]
struct Args {
    #[arg(long, conflicts_with = "levelstring_file")]
    levelstring: Option<String>,
    #[arg(long, conflicts_with = "save")]
    levelstring_file: Option<PathBuf>,
    #[arg(long, conflicts_with_all = ["levelstring", "levelstring_file"])]
    save: Option<PathBuf>,
    #[arg(long, requires = "save")]
    level: Option<String>,
    #[arg(long, conflicts_with_all = ["clicks_file", "clicks_bin"])]
    clicks: Option<String>,
    #[arg(long, conflicts_with = "clicks_bin")]
    clicks_file: Option<PathBuf>,
    /// SP240BIN packed-bit recording from bitstring/get_bitstring.py.
    #[arg(long)]
    clicks_bin: Option<PathBuf>,
    /// Override source sampling rate for --clicks-bin, then resample to 240 Hz.
    /// Useful when a recording was exported at e.g. 60 Hz by accident.
    #[arg(long, requires = "clicks_bin")]
    clicks_bin_hz: Option<u32>,
    /// List local levels in --save and exit (no simulation).
    #[arg(long, requires = "save")]
    list_levels: bool,
    #[arg(long, default_value_t = 240 * 60 * 5)]
    max_ticks: usize,
    /// Signed click-tape offset in ticks.
    /// Positive: skip the first N input ticks.
    /// Negative: prepend N zero ticks (push clicks forward in time).
    /// Useful for realigning audio-triggered recordings whose t=0 does not
    /// match attempt start.
    #[arg(
        long,
        visible_alias = "tick-offset",
        default_value_t = 0,
        allow_hyphen_values = true
    )]
    start_tick_offset: i32,
    #[arg(long, default_value = "error")]
    unsupported_policy: UnsupportedPolicy,
    #[arg(long)]
    trace_out: Option<PathBuf>,
    /// Write one fixed-width text line per simulated tick (plus partner rows in dual mode).
    #[arg(long)]
    tick_log_out: Option<PathBuf>,
    /// Open a simple debug window showing path + hitboxes.
    #[arg(long)]
    visualize: bool,
    /// Run the native visualizer as a live 240 Hz playable session.
    #[arg(long, requires = "visualize")]
    play_live: bool,
    /// JSONL file where live play attempts should be appended.
    #[arg(long, requires = "play_live")]
    live_attempt_history: Option<PathBuf>,
    /// DEV ONLY: overlay canonical tick log CSV in visualizer.
    #[arg(long, hide = true, requires = "visualize")]
    dev_canon_ticklog_csv: Option<PathBuf>,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum UnsupportedPolicy {
    Error,
    Report,
}

fn read_sp240bin(path: &PathBuf, override_hz: Option<u32>) -> anyhow::Result<String> {
    const SIM_HZ: u32 = 240;
    let bytes = fs::read(path)?;
    anyhow::ensure!(bytes.len() >= 8 + 13, "SP240BIN file too small");
    anyhow::ensure!(&bytes[0..8] == b"SP240BIN", "missing SP240BIN magic");
    let header_hz = u32::from_le_bytes(bytes[8..12].try_into().unwrap());
    let _duration = u32::from_le_bytes(bytes[12..16].try_into().unwrap());
    let header_total = u32::from_le_bytes(bytes[16..20].try_into().unwrap()) as usize;
    let _unused_bits = bytes[20] as i8;
    let source_hz = override_hz.unwrap_or(header_hz);
    anyhow::ensure!(source_hz > 0, "click input Hz must be > 0");
    let payload = &bytes[21..];
    // The header tells us the intended length. If the payload is short
    // (e.g. the recording was Ctrl+C'd early), pad the missing tail with
    // zeros instead of truncating - per user, the simulator should
    // continue playing the level with no input rather than ending the
    // tape prematurely.
    let payload_samples = payload.len() * 8;
    let mut source_bits = Vec::with_capacity(header_total);
    for i in 0..header_total {
        let bit = if i < payload_samples {
            (payload[i / 8] >> (i % 8)) & 1
        } else {
            0
        };
        source_bits.push(bit);
    }
    if header_total > payload_samples {
        eprintln!(
            "note: SP240BIN header says {header_total} samples, payload only has {payload_samples} (padding tail with zeros)"
        );
    }

    if source_hz == SIM_HZ {
        let mut out = String::with_capacity(source_bits.len());
        for bit in source_bits {
            out.push(if bit == 1 { '1' } else { '0' });
        }
        return Ok(out);
    }

    eprintln!(
        "note: resampling click tape from {source_hz} Hz to {SIM_HZ} Hz ({} source samples)",
        source_bits.len()
    );
    Ok(resample_click_bits_linear(&source_bits, source_hz, SIM_HZ))
}

fn resample_click_bits_linear(bits: &[u8], src_hz: u32, dst_hz: u32) -> String {
    if src_hz == 0 || dst_hz == 0 {
        return String::new();
    }
    if bits.is_empty() {
        return String::new();
    }
    if src_hz == dst_hz {
        let mut out = String::with_capacity(bits.len());
        for &b in bits {
            out.push(if b == 1 { '1' } else { '0' });
        }
        return out;
    }
    let out_len_u128 =
        (bits.len() as u128 * dst_hz as u128 + (src_hz as u128 / 2)) / src_hz as u128;
    let out_len = out_len_u128 as usize;
    let mut out = String::with_capacity(out_len);
    for i in 0..out_len {
        // Center-to-center resampling: map destination sample center into
        // source sample space to avoid a half-frame phase shift.
        let src_pos = ((i as f64 + 0.5) * src_hz as f64 / dst_hz as f64) - 0.5;
        let v = if src_pos <= 0.0 {
            bits[0] as f64
        } else if src_pos >= (bits.len() - 1) as f64 {
            bits[bits.len() - 1] as f64
        } else {
            let i0 = src_pos.floor() as usize;
            let i1 = i0 + 1;
            let frac = src_pos - i0 as f64;
            let v0 = bits[i0] as f64;
            let v1 = bits[i1] as f64;
            v0 + (v1 - v0) * frac
        };
        out.push(if v >= 0.5 { '1' } else { '0' });
    }
    out
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    if args.list_levels {
        let save_path = args.save.as_ref().expect("guarded by clap requires");
        let levels = read_local_levels(save_path)?;
        for (i, level) in levels.iter().enumerate() {
            println!("[{i}] {}", level.name);
        }
        return Ok(());
    }

    let levelstring = match (&args.levelstring, &args.levelstring_file) {
        (Some(levelstring), None) => levelstring.clone(),
        (None, Some(path)) => read_levelstring_file(path)?,
        (None, None) if args.save.is_some() => {
            let save_path = args.save.as_ref().expect("checked by guard");
            let levels = read_local_levels(save_path)?;
            select_local_level(&levels, args.level.as_deref())?
                .levelstring
                .clone()
        }
        _ => anyhow::bail!("provide exactly one of --levelstring, --levelstring-file, or --save"),
    };
    let db = ObjectDatabase::load_embedded()?;
    let level = Level::parse(levelstring.trim(), &db)?;

    if args.play_live {
        launch_live_visualizer(&level, args.live_attempt_history.as_deref())?;
        return Ok(());
    }

    let clicks = match (&args.clicks, &args.clicks_file, &args.clicks_bin) {
        (Some(clicks), None, None) => clicks.clone(),
        (None, Some(path), None) => fs::read_to_string(path)?,
        (None, None, Some(path)) => read_sp240bin(path, args.clicks_bin_hz)?,
        _ => anyhow::bail!("provide exactly one of --clicks, --clicks-file, or --clicks-bin"),
    };

    let trimmed_clicks = clicks.trim();
    let aligned_clicks = apply_tick_offset(trimmed_clicks, args.start_tick_offset);
    let tape = ClickTape::from_bits(&aligned_clicks)?;
    let result = simulate_with_trace(
        &level,
        &tape,
        SimulationConfig {
            max_ticks: args.max_ticks,
        },
    );

    match (result, args.unsupported_policy) {
        (Ok(run), _) => {
            if let Some(trace_out) = args.trace_out {
                let mut encoder = GzEncoder::new(Vec::new(), Compression::default());
                encoder.write_all(serde_json::to_vec(&run.trace)?.as_slice())?;
                fs::write(trace_out, encoder.finish()?)?;
            }
            if let Some(tick_log_out) = args.tick_log_out {
                fs::write(tick_log_out, format_tick_log(&run.trace))?;
            }
            println!("{}", serde_json::to_string_pretty(&run.outcome)?);
            if args.visualize {
                let canon_trace = if let Some(path) = args.dev_canon_ticklog_csv.as_ref() {
                    Some(read_canon_ticklog_csv(path)?)
                } else {
                    None
                };
                launch_visualizer(&level, &run.trace, canon_trace.as_deref())?;
            }
        }
        (Err(error), UnsupportedPolicy::Report) => println!(
            "{}",
            serde_json::json!({
                "outcome": "unsupported",
                "error": error.to_string()
            })
        ),
        (Err(error), UnsupportedPolicy::Error) => return Err(error.into()),
    }

    Ok(())
}

#[derive(Debug, Clone, Copy)]
struct CanonTracePoint {
    tick: usize,
    x: f32,
    y: f32,
}

#[derive(Debug, Clone, Serialize)]
struct LiveAttemptRecord {
    id: String,
    created_at_ms: u64,
    outcome: String,
    percent: f32,
    processed_clicks: usize,
    bitstring: String,
    tick: usize,
}

fn read_canon_ticklog_csv(path: &PathBuf) -> anyhow::Result<Vec<CanonTracePoint>> {
    let raw = fs::read_to_string(path)?;
    let mut out = Vec::new();
    for (line_idx, line) in raw.lines().enumerate() {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with("tick,") {
            continue;
        }
        let cols: Vec<&str> = trimmed.split(',').collect();
        if cols.len() < 4 {
            continue;
        }
        let tick = cols[0]
            .trim()
            .parse::<usize>()
            .map_err(|e| anyhow::anyhow!("invalid tick at line {}: {}", line_idx + 1, e))?;
        let x = cols[1]
            .trim()
            .parse::<f32>()
            .map_err(|e| anyhow::anyhow!("invalid x at line {}: {}", line_idx + 1, e))?;
        let y_text = cols[2].trim();
        if y_text.eq_ignore_ascii_case("nan") {
            continue;
        }
        let y = y_text
            .parse::<f32>()
            .map_err(|e| anyhow::anyhow!("invalid y at line {}: {}", line_idx + 1, e))?;
        out.push(CanonTracePoint { tick, x, y });
    }
    Ok(out)
}

fn read_levelstring_file(path: &PathBuf) -> anyhow::Result<String> {
    let raw = fs::read_to_string(path)?;
    let trimmed = raw.trim();
    if trimmed.contains(';') || trimmed.contains(',') {
        Ok(trimmed.to_owned())
    } else {
        Ok(decode_level_payload(trimmed)?)
    }
}

fn format_tick_log(trace: &[gd_real_sim::sim::TraceFrame]) -> String {
    let mut out = String::new();
    out.push_str(
        "clk tick   time      plr mode    grav   mini on_gnd on_slp x           y           vx          vy          p_spd   s_mult  g        y_start   size\n",
    );
    for frame in trace {
        append_player_line(
            &mut out,
            frame.tick,
            frame.time,
            0,
            frame.pressed,
            &frame.state,
        );
        if let Some(partner) = frame.partner {
            append_player_line(&mut out, frame.tick, frame.time, 1, frame.pressed, &partner);
        }
    }
    out
}

fn append_player_line(
    out: &mut String,
    tick: usize,
    time: f32,
    player_index: u8,
    pressed: bool,
    state: &gd_real_sim::sim::PlayerState,
) {
    let press = if pressed { "1" } else { "0" };
    let mini = if state.mini { "1" } else { "0" };
    let on_ground = if state.on_ground { "1" } else { "0" };
    let on_slope = if state.on_slope { "1" } else { "0" };
    out.push_str(&format!(
        "{press:>3} {tick:>6} {time:>8.5} {player_index:>4} {mode:<7} {grav:>6.1} {mini:>6} {on_ground:>6} {on_slope:>6} \
{x:>11.5} {y:>11.5} {vx:>11.5} {vy:>11.5} {player_speed:>7.3} {speed_multiplier:>7.3} {gravity:>8.5} {y_start:>9.5} {vehicle_size:>6.2}\n",
        mode = format!("{:?}", state.mode),
        grav = state.gravity_sign,
        x = state.x,
        y = state.y,
        vx = state.vx,
        vy = state.vy,
        player_speed = state.player_speed,
        speed_multiplier = state.speed_multiplier,
        gravity = state.gravity,
        y_start = state.y_start,
        vehicle_size = state.vehicle_size,
    ));
}

fn apply_tick_offset(clicks: &str, offset: i32) -> String {
    if offset >= 0 {
        let trim = (offset as usize).min(clicks.len());
        clicks[trim..].to_owned()
    } else {
        let pad = (-offset) as usize;
        let mut shifted = String::with_capacity(pad + clicks.len());
        shifted.extend(std::iter::repeat_n('0', pad));
        shifted.push_str(clicks);
        shifted
    }
}

fn live_attempt_bitstring(trace: &[gd_real_sim::sim::TraceFrame]) -> String {
    trace
        .iter()
        .map(|frame| if frame.pressed { '1' } else { '0' })
        .collect()
}

fn level_progress_percent(level: &Level, x: f32) -> f32 {
    let Some(finish_x) = visualizer_finish_x(level) else {
        return 0.0;
    };
    if finish_x <= 0.0 {
        return if x >= finish_x { 100.0 } else { 0.0 };
    }
    ((x.max(0.0) / finish_x) * 100.0).clamp(0.0, 100.0)
}

fn visualizer_finish_x(level: &Level) -> Option<f32> {
    if let Some(finish_portal_x) = level
        .objects
        .iter()
        .filter(|object| object.object_id == 3607)
        .map(|object| object.x)
        .min_by(|a, b| a.total_cmp(b))
    {
        return Some(finish_portal_x);
    }

    level
        .objects
        .iter()
        .filter(|object| {
            matches!(
                object.kind,
                gd_real_sim::level::ObjectKind::Solid
                    | gd_real_sim::level::ObjectKind::Slope
                    | gd_real_sim::level::ObjectKind::Hazard
            )
        })
        .map(|object| object.x)
        .max_by(|a, b| a.total_cmp(b))
        .map(|last_block_x| last_block_x + 60.0)
}

fn live_attempt_record(
    level: &Level,
    trace: &[gd_real_sim::sim::TraceFrame],
    outcome: &SimulationOutcome,
    attempt_index: u64,
) -> LiveAttemptRecord {
    let bitstring = live_attempt_bitstring(trace);
    let created_at_ms = current_time_ms();
    let (outcome_label, tick, x) = outcome_summary(outcome);
    LiveAttemptRecord {
        id: format!("{created_at_ms}-{attempt_index}"),
        created_at_ms,
        outcome: outcome_label.to_owned(),
        percent: level_progress_percent(level, x),
        processed_clicks: bitstring.len(),
        bitstring,
        tick,
    }
}

fn outcome_summary(outcome: &SimulationOutcome) -> (&'static str, usize, f32) {
    match outcome {
        SimulationOutcome::Completed { tick, state, .. } => ("completed", *tick, state.x),
        SimulationOutcome::Died { tick, state, .. } => ("died", *tick, state.x),
        SimulationOutcome::Timeout { tick, state, .. } => ("timeout", *tick, state.x),
    }
}

fn current_time_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_millis().min(u64::MAX as u128) as u64)
        .unwrap_or(0)
}

fn append_live_attempt(path: &Path, record: &LiveAttemptRecord) -> anyhow::Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    let mut file = OpenOptions::new().create(true).append(true).open(path)?;
    serde_json::to_writer(&mut file, record)?;
    writeln!(file)?;
    Ok(())
}

const BG_COLOR: u32 = 0x101018;
const PATH_COLOR: u32 = 0x20C8FF;
const PRESS_PATH_COLOR: u32 = 0xFF4040;
const CUBE_COLOR: u32 = 0xFFFFFF;
const CUBE_ALPHA: f32 = 0.20;
const PARTNER_PATH_COLOR: u32 = 0xAA66FF;
const PRESS_BAR_PRESSED_COLOR: u32 = 0x32C950;
const PRESS_BAR_RELEASED_COLOR: u32 = 0xD64545;
const PRESS_BAR_UNKNOWN_COLOR: u32 = 0x3A3A48;
const PRESS_BAR_HEIGHT: usize = 18;
const SPEED_BAR_HEIGHT: usize = 16;
const MODE_BAR_HEIGHT: usize = 16;
const SPEED_BAR_BG_COLOR: u32 = 0x1B1F2E;
const SPEED_BAR_UNKNOWN_COLOR: u32 = 0x3A3A48;
const SPEED_BAR_200_COLOR: u32 = 0xFF9900; // orange
const SPEED_BAR_201_COLOR: u32 = 0x3399FF; // blue
const SPEED_BAR_202_COLOR: u32 = 0x32C950; // green
const SPEED_BAR_203_COLOR: u32 = 0xAA66FF; // purple
const SPEED_BAR_1334_COLOR: u32 = 0xFF4040; // red
const MODE_BAR_BG_COLOR: u32 = 0x1B1F2E;
const MODE_BAR_CUBE_COLOR: u32 = 0x32C950; // green
const MODE_BAR_SHIP_COLOR: u32 = 0xB06BFF; // purple
const MODE_BAR_BALL_COLOR: u32 = 0xFF4040; // red
const MODE_BAR_UFO_COLOR: u32 = 0xFF9900; // orange
const MODE_BAR_WAVE_COLOR: u32 = 0x3399FF; // blue
const MODE_BAR_SPIDER_COLOR: u32 = 0x7D3EC7; // dark purple
const MODE_BAR_SWING_COLOR: u32 = 0xFFE066; // yellow
const VY_BAR_HEIGHT: usize = 28;
const VY_BAR_BG_COLOR: u32 = 0x1B1F2E;
const VY_BAR_ZERO_COLOR: u32 = 0x3A3A48;
const VY_BAR_UP_COLOR: u32 = 0x32C950;
const VY_BAR_DOWN_COLOR: u32 = 0xD64545;
const SCRUBBER_HEIGHT: usize = 18;
const SCRUBBER_BG_COLOR: u32 = 0x1B1F2E;
const SCRUBBER_FILL_COLOR: u32 = 0x40556B;
const SCRUBBER_HANDLE_COLOR: u32 = 0xF4F4FA;
const PLAYHEAD_COLOR: u32 = 0xFFE066;
const SETTINGS_BTN_SIZE: i32 = 22;
const SETTINGS_BTN_GAP: i32 = 4;
const SETTINGS_BTN_BG_OFF: u32 = 0x2A3342;
const SETTINGS_BTN_BG_ON: u32 = 0x32C950;
const SETTINGS_BTN_BORDER: u32 = 0xF4F4FA;
const SETTINGS_PANEL_BG: u32 = 0x1B1F2E;
const SETTINGS_PANEL_WIDTH: i32 = 188;
const SETTINGS_PANEL_ROW_H: i32 = 20;
const SETTINGS_PANEL_ROW_GAP: i32 = 4;
const SETTINGS_PANEL_PAD: i32 = 6;
const HITBOX_MAIN_COLOR: u32 = CUBE_COLOR;
const HITBOX_ROTATED_COLOR: u32 = 0x66CCFF;
const HITBOX_CORE_COLOR: u32 = 0xFFAA44;
const CANON_PATH_COLOR: u32 = 0xFF66CC;
const HITBOX_CURRENT_ALPHA: f32 = 0.90;
const VIEW_HEIGHT_BLOCKS: f32 = 20.0;
const BLOCK_SIZE: f32 = 30.0;
/// Sim editor-style grid: lines every `BLOCK_SIZE` units, cell corner at this world point.
const GRID_ORIGIN_X: f32 = 0.0;
const GRID_ORIGIN_Y: f32 = 90.0;
const GRID_COLOR: u32 = 0x2A3342;
const SCROLL_STEP_WORLD: f32 = 12.0;
const ZOOM_MIN: f32 = 0.5;
const ZOOM_MAX: f32 = 4.0;
const ZOOM_STEP: f32 = 1.15;

struct VizState {
    scroll_x: f32,
    /// Vertical world-space scroll. Positive moves the camera up (scene
    /// floor visible higher in the window). 0 keeps the original auto-fit
    /// behavior where `view_min_y == scene.min_y`.
    scroll_y: f32,
    zoom: f32,
    show_clicks: bool,
    show_speed: bool,
    show_mode: bool,
    show_vy: bool,
    show_main_hitbox: bool,
    show_rotated_hitbox: bool,
    show_core_hitbox: bool,
    show_trail: bool,
    show_canon_trace: bool,
    has_canon_trace: bool,
    show_settings_panel: bool,
    follow_cube: bool,
    current_tick: usize,
    scrubbing: bool,
    last_mouse_down: bool,
    /// Right-mouse-button pan state: when `Some`, holds the world-space
    /// `(scroll_x, scroll_y)` that was active when the drag began plus
    /// the screen-space anchor `(mx, my)` from that frame. The current
    /// scroll values are then computed each frame as `start + (mx-anchor) /
    /// scale * sign`. Cleared when RMB is released.
    pan_anchor: Option<(f32, f32, f32, f32)>,
    last_rmouse_down: bool,
}

struct Viewport {
    view_min_x: f32,
    view_min_y: f32,
    scale: f32,
    margin: f32,
    plot_bottom: f32,
    width: usize,
    height: usize,
}

struct SceneBounds {
    min_x: f32,
    min_y: f32,
    max_x: f32,
}

/// Pixel y-bands used by the bar overlays. Computed each frame from the
/// current toggle state so bars can be hidden without leaving empty space
/// in the world view.
struct Layout {
    scrubber_top: usize,
    scrubber_bottom: usize,
    plot_top: usize,
    plot_bottom: usize,
    press_top: Option<usize>,
    press_bottom: Option<usize>,
    speed_top: Option<usize>,
    speed_bottom: Option<usize>,
    mode_top: Option<usize>,
    mode_bottom: Option<usize>,
    vy_top: Option<usize>,
    vy_bottom: Option<usize>,
}

fn compute_layout(
    height: usize,
    show_clicks: bool,
    show_speed: bool,
    show_mode: bool,
    show_vy: bool,
) -> Layout {
    let scrubber_top = 0;
    let scrubber_bottom = scrubber_top + SCRUBBER_HEIGHT;
    let mut bottom = height;
    let (vy_top, vy_bottom) = if show_vy {
        let t = bottom.saturating_sub(VY_BAR_HEIGHT);
        bottom = t;
        (Some(t), Some(t + VY_BAR_HEIGHT))
    } else {
        (None, None)
    };
    let (press_top, press_bottom) = if show_clicks {
        let t = bottom.saturating_sub(PRESS_BAR_HEIGHT);
        bottom = t;
        (Some(t), Some(t + PRESS_BAR_HEIGHT))
    } else {
        (None, None)
    };
    let (speed_top, speed_bottom) = if show_speed {
        let t = bottom.saturating_sub(SPEED_BAR_HEIGHT);
        bottom = t;
        (Some(t), Some(t + SPEED_BAR_HEIGHT))
    } else {
        (None, None)
    };
    let (mode_top, mode_bottom) = if show_mode {
        let t = bottom.saturating_sub(MODE_BAR_HEIGHT);
        bottom = t;
        (Some(t), Some(t + MODE_BAR_HEIGHT))
    } else {
        (None, None)
    };
    Layout {
        scrubber_top,
        scrubber_bottom,
        plot_top: scrubber_bottom,
        plot_bottom: bottom,
        press_top,
        press_bottom,
        speed_top,
        speed_bottom,
        mode_top,
        mode_bottom,
        vy_top,
        vy_bottom,
    }
}

fn launch_visualizer(
    level: &Level,
    trace: &[gd_real_sim::sim::TraceFrame],
    canon_trace: Option<&[CanonTracePoint]>,
) -> anyhow::Result<()> {
    if trace.is_empty() {
        return Ok(());
    }

    let width = 1120;
    let height = 720;
    let mut window = Window::new(
        "gd-real-sim visualizer",
        width,
        height,
        WindowOptions::default(),
    )?;
    window.set_target_fps(60);

    eprintln!(
        "visualizer keys: Left/Right or A/D = scroll, +/- or wheel = zoom, \
         F = toggle follow-cube, \
         Home/End = first/last tick, drag top bar = scrub time, \
         right-mouse drag = pan camera (X+Y), Esc = close"
    );

    let mut buffer = vec![BG_COLOR; width * height];
    let scene = scene_bounds(level, trace, canon_trace);
    let mut state = VizState {
        scroll_x: scene.min_x,
        scroll_y: 0.0,
        zoom: 1.0,
        show_clicks: true,
        show_speed: true,
        show_mode: true,
        show_vy: true,
        show_main_hitbox: true,
        show_rotated_hitbox: false,
        show_core_hitbox: false,
        show_trail: true,
        show_canon_trace: canon_trace.is_some(),
        has_canon_trace: canon_trace.is_some(),
        show_settings_panel: false,
        follow_cube: false,
        current_tick: trace.len() - 1,
        scrubbing: false,
        last_mouse_down: false,
        pan_anchor: None,
        last_rmouse_down: false,
    };

    while window.is_open() && !window.is_key_down(Key::Escape) {
        // ---- input ----
        if window.is_key_down(Key::Left) || window.is_key_down(Key::A) {
            state.scroll_x -= SCROLL_STEP_WORLD / state.zoom;
            state.follow_cube = false;
        }
        if window.is_key_down(Key::Right) || window.is_key_down(Key::D) {
            state.scroll_x += SCROLL_STEP_WORLD / state.zoom;
            state.follow_cube = false;
        }
        for key in window.get_keys_pressed(KeyRepeat::No) {
            match key {
                Key::F => state.follow_cube = !state.follow_cube,
                Key::Equal | Key::NumPadPlus => {
                    state.zoom = (state.zoom * ZOOM_STEP).min(ZOOM_MAX);
                }
                Key::Minus | Key::NumPadMinus => {
                    state.zoom = (state.zoom / ZOOM_STEP).max(ZOOM_MIN);
                }
                Key::Home => state.current_tick = 0,
                Key::End => state.current_tick = trace.len() - 1,
                _ => {}
            }
        }
        if let Some((_, dy)) = window.get_scroll_wheel() {
            if dy > 0.0 {
                state.zoom = (state.zoom * ZOOM_STEP).min(ZOOM_MAX);
            } else if dy < 0.0 {
                state.zoom = (state.zoom / ZOOM_STEP).max(ZOOM_MIN);
            }
        }

        // Mouse drag handling for the top scrubber bar + RMB-drag pan.
        let mouse_down = window.get_mouse_down(MouseButton::Left);
        let rmouse_down = window.get_mouse_down(MouseButton::Right);
        let mouse_pos = window.get_mouse_pos(MouseMode::Discard);
        let layout = compute_layout(
            height,
            state.show_clicks,
            state.show_speed,
            state.show_mode,
            state.show_vy,
        );

        // Right-mouse-drag pan: anchor screen<->world delta on press,
        // then accumulate (mx-anchor_mx)/scale into scroll_x and
        // (my-anchor_my)/scale into scroll_y. We disable follow_cube on
        // any pan so the camera stays where the user dragged it.
        if let Some((mx, my)) = mouse_pos {
            let scale_now = base_scale(height, &layout) * state.zoom;
            if rmouse_down && !state.last_rmouse_down {
                state.pan_anchor = Some((mx, my, state.scroll_x, state.scroll_y));
                state.follow_cube = false;
            } else if rmouse_down {
                if let Some((anchor_mx, anchor_my, start_sx, start_sy)) = state.pan_anchor {
                    let dx_world = (mx - anchor_mx) / scale_now;
                    // Screen Y grows downward; world Y grows upward
                    // (see `world_to_screen` Y-flip), so a downward drag
                    // (`my - anchor_my > 0`) should LOWER the camera
                    // (scroll_y decreases) - i.e. matches the convention
                    // "drag the world with the cursor".
                    let dy_world = (my - anchor_my) / scale_now;
                    state.scroll_x = start_sx - dx_world;
                    state.scroll_y = start_sy + dy_world;
                }
            }
            if !rmouse_down {
                state.pan_anchor = None;
            }
        }
        state.last_rmouse_down = rmouse_down;

        if let Some((mx, my)) = mouse_pos {
            let in_scrubber =
                (my as usize) >= layout.scrubber_top && (my as usize) < layout.scrubber_bottom;
            let mut consumed_left_click = false;
            if mouse_down && !state.last_mouse_down {
                if point_in_rect(mx, my, settings_button_rect(width)) {
                    state.show_settings_panel = !state.show_settings_panel;
                    consumed_left_click = true;
                } else if state.show_settings_panel
                    && let Some(toggle) = settings_panel_toggle_at(width, mx, my, &state, false)
                {
                    let _ = apply_settings_toggle(&mut state, toggle);
                    consumed_left_click = true;
                }
            }
            if !consumed_left_click && mouse_down && (in_scrubber || state.scrubbing) {
                state.scrubbing = true;
                let t = (mx / width as f32).clamp(0.0, 1.0);
                state.current_tick = ((trace.len() - 1) as f32 * t).round() as usize;
            }
            if !mouse_down {
                state.scrubbing = false;
            }
        }
        state.last_mouse_down = mouse_down;

        // Recompute layout after toggle changes from this tick.
        let layout = compute_layout(
            height,
            state.show_clicks,
            state.show_speed,
            state.show_mode,
            state.show_vy,
        );

        // Optional follow-cube: snap scroll_x so the scrubbed cube is centered.
        if state.follow_cube {
            let cube_x = trace[state.current_tick].state.x;
            let span = (width as f32 - 48.0) / (base_scale(height, &layout) * state.zoom);
            state.scroll_x = cube_x - span * 0.5;
        }

        let viewport = build_viewport(
            &scene,
            width,
            height,
            &layout,
            state.scroll_x,
            state.scroll_y,
            state.zoom,
        );
        state.scroll_x = clamp_scroll_x(&scene, &viewport, state.scroll_x);
        let viewport = build_viewport(
            &scene,
            width,
            height,
            &layout,
            state.scroll_x,
            state.scroll_y,
            state.zoom,
        );

        // ---- render ----
        let visible_trace = &trace[..=state.current_tick];
        render_scene(
            &mut buffer,
            &viewport,
            level,
            visible_trace,
            canon_trace,
            &state,
        );
        if state.show_clicks {
            draw_press_bar(&mut buffer, &viewport, &layout, trace, state.current_tick);
        }
        if state.show_speed {
            draw_speed_bar(&mut buffer, &viewport, &layout, trace, state.current_tick);
        }
        if state.show_mode {
            draw_mode_bar(&mut buffer, &viewport, &layout, trace, state.current_tick);
        }
        if state.show_vy {
            draw_vy_bar(&mut buffer, &viewport, &layout, trace, state.current_tick);
        }
        draw_scrubber_bar(&mut buffer, &viewport, &layout, trace, state.current_tick);
        draw_settings_ui(&mut buffer, width, height, &state, false);

        window.set_title(&format!(
            "gd-real-sim | tick {}/{} | zoom {:.2}x | follow:{}",
            state.current_tick,
            trace.len() - 1,
            state.zoom,
            on_off(state.follow_cube),
        ));

        window.update_with_buffer(&buffer, width, height)?;
        std::thread::sleep(Duration::from_millis(16));
    }
    Ok(())
}

fn launch_live_visualizer(level: &Level, attempt_history: Option<&Path>) -> anyhow::Result<()> {
    let width = 1120;
    let height = 720;
    let mut window = Window::new(
        "gd-real-sim live play",
        width,
        height,
        WindowOptions::default(),
    )?;
    window.set_target_fps(240);

    eprintln!(
        "live controls: Space/Up/left mouse = hold, F = toggle follow-cube, \
         Left/Right or A/D = scroll, +/- or wheel = zoom, right-mouse drag = pan, \
         Esc = settings, Esc in settings = close"
    );

    let mut buffer = vec![BG_COLOR; width * height];
    let mut scene = scene_bounds(level, &[], None);
    scene.min_x = scene.min_x.min(0.0);
    scene.max_x = scene.max_x.max(0.0);
    let mut state = VizState {
        scroll_x: 0.0,
        scroll_y: 0.0,
        zoom: 1.0,
        show_clicks: true,
        show_speed: true,
        show_mode: true,
        show_vy: true,
        show_main_hitbox: true,
        show_rotated_hitbox: false,
        show_core_hitbox: false,
        show_trail: true,
        show_canon_trace: false,
        has_canon_trace: false,
        show_settings_panel: false,
        follow_cube: true,
        current_tick: 0,
        scrubbing: false,
        last_mouse_down: false,
        pan_anchor: None,
        last_rmouse_down: false,
    };

    let mut session = LiveSimulationSession::new(level)?;
    let mut trace = Vec::new();
    let mut last_frame_at = Instant::now();
    let mut accumulator = Duration::ZERO;
    let tick_dt = Duration::from_secs_f32(DT);
    let mut restart_at: Option<Instant> = None;
    let mut stopped_outcome: Option<SimulationOutcome> = None;
    let mut attempt_index = 0_u64;

    'live: while window.is_open() {
        let now = Instant::now();
        accumulator += now.saturating_duration_since(last_frame_at);
        last_frame_at = now;

        if let Some(when) = restart_at
            && now >= when
        {
            session = LiveSimulationSession::new(level)?;
            trace.clear();
            state.current_tick = 0;
            accumulator = Duration::ZERO;
            restart_at = None;
            stopped_outcome = None;
        }

        if window.is_key_down(Key::Left) || window.is_key_down(Key::A) {
            state.scroll_x -= SCROLL_STEP_WORLD / state.zoom;
            state.follow_cube = false;
        }
        if window.is_key_down(Key::Right) || window.is_key_down(Key::D) {
            state.scroll_x += SCROLL_STEP_WORLD / state.zoom;
            state.follow_cube = false;
        }
        for key in window.get_keys_pressed(KeyRepeat::No) {
            match key {
                Key::F => state.follow_cube = !state.follow_cube,
                Key::Escape => {
                    if state.show_settings_panel {
                        break 'live;
                    }
                    state.show_settings_panel = true;
                }
                Key::Equal | Key::NumPadPlus => {
                    state.zoom = (state.zoom * ZOOM_STEP).min(ZOOM_MAX);
                }
                Key::Minus | Key::NumPadMinus => {
                    state.zoom = (state.zoom / ZOOM_STEP).max(ZOOM_MIN);
                }
                _ => {}
            }
        }
        if let Some((_, dy)) = window.get_scroll_wheel() {
            if dy > 0.0 {
                state.zoom = (state.zoom * ZOOM_STEP).min(ZOOM_MAX);
            } else if dy < 0.0 {
                state.zoom = (state.zoom / ZOOM_STEP).max(ZOOM_MIN);
            }
        }

        let mouse_down = window.get_mouse_down(MouseButton::Left);
        let rmouse_down = window.get_mouse_down(MouseButton::Right);
        let mouse_pos = window.get_mouse_pos(MouseMode::Discard);
        let layout = compute_layout(
            height,
            state.show_clicks,
            state.show_speed,
            state.show_mode,
            state.show_vy,
        );
        if let Some((mx, my)) = mouse_pos {
            let scale_now = base_scale(height, &layout) * state.zoom;
            if rmouse_down && !state.last_rmouse_down {
                state.pan_anchor = Some((mx, my, state.scroll_x, state.scroll_y));
                state.follow_cube = false;
            } else if rmouse_down
                && let Some((anchor_mx, anchor_my, start_sx, start_sy)) = state.pan_anchor
            {
                let dx_world = (mx - anchor_mx) / scale_now;
                let dy_world = (my - anchor_my) / scale_now;
                state.scroll_x = start_sx - dx_world;
                state.scroll_y = start_sy + dy_world;
            }
            if !rmouse_down {
                state.pan_anchor = None;
            }
        }
        state.last_rmouse_down = rmouse_down;

        let mut consumed_left_click = false;
        if let Some((mx, my)) = mouse_pos {
            if mouse_down && !state.last_mouse_down {
                if point_in_rect(mx, my, settings_button_rect(width)) {
                    state.show_settings_panel = !state.show_settings_panel;
                    consumed_left_click = true;
                } else if state.show_settings_panel
                    && let Some(toggle) = settings_panel_toggle_at(width, mx, my, &state, true)
                {
                    if apply_settings_toggle(&mut state, toggle) {
                        break 'live;
                    }
                    consumed_left_click = true;
                }
            }
        }
        state.last_mouse_down = mouse_down;

        if stopped_outcome.is_none() {
            let mut steps_this_frame = 0;
            while accumulator >= tick_dt && steps_this_frame < 8 {
                let held = !state.show_settings_panel
                    && ((mouse_down && !consumed_left_click)
                        || window.is_key_down(Key::Space)
                        || window.is_key_down(Key::Up));
                let step = session.step_live(held)?;
                let outcome = step.outcome.clone();
                trace.push(step.frame);
                state.current_tick = trace.len().saturating_sub(1);
                accumulator = accumulator.saturating_sub(tick_dt);
                steps_this_frame += 1;

                if let Some(outcome) = outcome {
                    if let Some(path) = attempt_history {
                        let record = live_attempt_record(level, &trace, &outcome, attempt_index);
                        attempt_index = attempt_index.saturating_add(1);
                        if let Err(error) = append_live_attempt(path, &record) {
                            eprintln!("failed to write live attempt history: {error}");
                        }
                    }
                    if matches!(outcome, SimulationOutcome::Died { .. }) {
                        restart_at = Some(Instant::now() + Duration::from_secs(1));
                    }
                    stopped_outcome = Some(outcome);
                    break;
                }
            }
        }

        let layout = compute_layout(
            height,
            state.show_clicks,
            state.show_speed,
            state.show_mode,
            state.show_vy,
        );

        if state.follow_cube
            && let Some(frame) = trace.get(state.current_tick)
        {
            let span = (width as f32 - 48.0) / (base_scale(height, &layout) * state.zoom);
            state.scroll_x = frame.state.x - span * 0.35;
        }

        let viewport = build_viewport(
            &scene,
            width,
            height,
            &layout,
            state.scroll_x,
            state.scroll_y,
            state.zoom,
        );
        state.scroll_x = clamp_scroll_x(&scene, &viewport, state.scroll_x);
        let viewport = build_viewport(
            &scene,
            width,
            height,
            &layout,
            state.scroll_x,
            state.scroll_y,
            state.zoom,
        );

        render_scene(&mut buffer, &viewport, level, &trace, None, &state);
        if state.show_clicks {
            draw_press_bar(&mut buffer, &viewport, &layout, &trace, state.current_tick);
        }
        if state.show_speed {
            draw_speed_bar(&mut buffer, &viewport, &layout, &trace, state.current_tick);
        }
        if state.show_mode {
            draw_mode_bar(&mut buffer, &viewport, &layout, &trace, state.current_tick);
        }
        if state.show_vy {
            draw_vy_bar(&mut buffer, &viewport, &layout, &trace, state.current_tick);
        }
        draw_scrubber_bar(&mut buffer, &viewport, &layout, &trace, state.current_tick);
        draw_settings_ui(&mut buffer, width, height, &state, true);

        let status = match &stopped_outcome {
            Some(SimulationOutcome::Died { .. }) => "dead - restarting",
            Some(SimulationOutcome::Completed { .. }) => "complete",
            Some(SimulationOutcome::Timeout { .. }) => "timeout",
            None => {
                if !state.show_settings_panel
                    && (mouse_down || window.is_key_down(Key::Space) || window.is_key_down(Key::Up))
                {
                    "holding"
                } else {
                    "released"
                }
            }
        };
        window.set_title(&format!(
            "gd-real-sim live | tick {} | {} | follow:{}",
            session.tick(),
            status,
            on_off(state.follow_cube)
        ));

        window.update_with_buffer(&buffer, width, height)?;
    }

    Ok(())
}

fn on_off(b: bool) -> &'static str {
    if b { "on" } else { "off" }
}

fn base_scale(_height: usize, layout: &Layout) -> f32 {
    let plot_height = (layout.plot_bottom.saturating_sub(layout.plot_top) as f32 - 48.0).max(1.0);
    (plot_height / (VIEW_HEIGHT_BLOCKS * BLOCK_SIZE)).max(0.01)
}

fn scene_bounds(
    level: &Level,
    trace: &[gd_real_sim::sim::TraceFrame],
    canon_trace: Option<&[CanonTracePoint]>,
) -> SceneBounds {
    let mut min_x = f32::INFINITY;
    let mut min_y = f32::INFINITY;
    let mut max_x = f32::NEG_INFINITY;

    let mut include = |x: f32, y: f32| {
        min_x = min_x.min(x);
        min_y = min_y.min(y);
        max_x = max_x.max(x);
    };

    for object in &level.objects {
        if let Some((ox1, oy1, ox2, oy2)) = object_bounds(object) {
            include(ox1, oy1);
            include(ox2, oy2);
        }
    }

    for frame in trace {
        let half = player_half(frame.state.mini);
        include(frame.state.x - half, frame.state.y - half);
        include(frame.state.x + half, frame.state.y + half);
        if let Some(partner) = frame.partner {
            let ph = player_half(partner.mini);
            include(partner.x - ph, partner.y - ph);
            include(partner.x + ph, partner.y + ph);
        }
    }
    if let Some(canon) = canon_trace {
        for point in canon {
            include(point.x, point.y);
        }
    }

    if !min_x.is_finite() {
        min_x = 0.0;
        min_y = 0.0;
        max_x = 100.0;
    }

    SceneBounds {
        min_x,
        min_y,
        max_x,
    }
}

fn build_viewport(
    scene: &SceneBounds,
    width: usize,
    height: usize,
    layout: &Layout,
    scroll_x: f32,
    scroll_y: f32,
    zoom: f32,
) -> Viewport {
    let margin = 24.0;
    let scale = (base_scale(height, layout) * zoom).max(0.01);

    Viewport {
        view_min_x: scroll_x,
        // `scroll_y` is an additive world-space offset on top of the
        // auto-fit `scene.min_y`. `0.0` reproduces the original viewport.
        view_min_y: scene.min_y + scroll_y,
        scale,
        margin,
        plot_bottom: layout.plot_bottom as f32 - 4.0,
        width,
        height,
    }
}

fn clamp_scroll_x(scene: &SceneBounds, viewport: &Viewport, scroll_x: f32) -> f32 {
    let span = ((viewport.width as f32 - 2.0 * viewport.margin) / viewport.scale).max(1.0);
    let max_scroll = (scene.max_x - span).max(scene.min_x);
    scroll_x.clamp(scene.min_x, max_scroll)
}

fn render_scene(
    buffer: &mut [u32],
    viewport: &Viewport,
    level: &Level,
    trace: &[gd_real_sim::sim::TraceFrame],
    canon_trace: Option<&[CanonTracePoint]>,
    state: &VizState,
) {
    buffer.fill(BG_COLOR);

    draw_world_grid(buffer, viewport);

    // Draw non-hazard objects first.
    for object in &level.objects {
        if object.kind == gd_real_sim::level::ObjectKind::Hazard {
            continue;
        }
        draw_object_hitbox(buffer, viewport, object);
    }
    // Draw hazards on top so ground-touching hazards remain visible.
    for object in &level.objects {
        if object.kind != gd_real_sim::level::ObjectKind::Hazard {
            continue;
        }
        draw_object_hitbox(buffer, viewport, object);
    }

    for pair in trace.windows(2) {
        let a = &pair[0];
        let b = &pair[1];
        let color = if a.pressed || b.pressed {
            PRESS_PATH_COLOR
        } else {
            PATH_COLOR
        };
        draw_line_world(
            buffer, viewport, a.state.x, a.state.y, b.state.x, b.state.y, color,
        );
        if let (Some(pa), Some(pb)) = (a.partner, b.partner) {
            draw_line_world(buffer, viewport, pa.x, pa.y, pb.x, pb.y, PARTNER_PATH_COLOR);
        }
    }
    if state.show_canon_trace {
        if let Some(canon) = canon_trace {
            draw_canon_trace(buffer, viewport, canon, state.current_tick);
        }
    }

    if state.show_trail {
        let step = (trace.len() / 800).max(1);
        for frame in trace.iter().step_by(step) {
            draw_player_hitboxes(buffer, viewport, frame.state, state, CUBE_ALPHA);
        }
    }
    if let Some(current) = trace.last() {
        draw_player_hitboxes(buffer, viewport, current.state, state, HITBOX_CURRENT_ALPHA);
    }

    // Bars (press / vy / scrubber) are drawn by `launch_visualizer` after
    // `render_scene` so they sit on top of everything regardless of state.
}

fn draw_canon_trace(
    buffer: &mut [u32],
    viewport: &Viewport,
    canon: &[CanonTracePoint],
    current_tick: usize,
) {
    let mut prev: Option<(f32, f32)> = None;
    for point in canon.iter().copied().filter(|p| p.tick <= current_tick) {
        if let Some((px, py)) = prev {
            draw_line_world(buffer, viewport, px, py, point.x, point.y, CANON_PATH_COLOR);
        }
        prev = Some((point.x, point.y));
    }
}

fn draw_object_hitbox(
    buffer: &mut [u32],
    viewport: &Viewport,
    object: &gd_real_sim::level::LevelObject,
) {
    if let Some(corners) = object_oriented_quad(object) {
        draw_quad_outline(buffer, viewport, corners, color_for_kind(object.kind));
    } else if let Some((x1, y1, x2, y2)) = object_bounds(object) {
        draw_rect_outline(
            buffer,
            viewport,
            x1,
            y1,
            x2,
            y2,
            color_for_kind(object.kind),
        );
    }
    if matches!(
        object.hitbox,
        Some(gd_real_sim::object_data::HitboxData::Circle { .. })
    ) && let Some((cx, cy, r)) = object_circle(object)
    {
        draw_circle_outline(buffer, viewport, cx, cy, r, color_for_kind(object.kind));
    }
    if matches!(object.kind, gd_real_sim::level::ObjectKind::Slope)
        && let Some((x1, y1, x2, y2)) = slope_surface_segment(object)
    {
        draw_line_world(buffer, viewport, x1, y1, x2, y2, 0x6BFF8A);
    }
    // Some near-ground "block-like" decoration objects in Pop have no explicit
    // hitbox data. Draw a faint proxy tile so they are still visible in the
    // visualizer and don't appear missing.
    if object.hitbox.is_none() && object.kind == gd_real_sim::level::ObjectKind::Decoration {
        draw_rect_outline_alpha(
            buffer,
            viewport,
            object.x - 15.0,
            object.y - 15.0,
            object.x + 15.0,
            object.y + 15.0,
            0x3A3A4A,
            0.45,
        );
    }
}

fn color_for_kind(kind: gd_real_sim::level::ObjectKind) -> u32 {
    match kind {
        gd_real_sim::level::ObjectKind::Solid => 0x606060,
        gd_real_sim::level::ObjectKind::Hazard => 0xFF8A00,
        gd_real_sim::level::ObjectKind::Slope => 0x3DAA5A,
        gd_real_sim::level::ObjectKind::Pad => 0xB66CFF,
        gd_real_sim::level::ObjectKind::Orb => 0xFFE866,
        gd_real_sim::level::ObjectKind::ModePortal
        | gd_real_sim::level::ObjectKind::SpeedPortal
        | gd_real_sim::level::ObjectKind::GravityPortal
        | gd_real_sim::level::ObjectKind::SizePortal
        | gd_real_sim::level::ObjectKind::MirrorPortal
        | gd_real_sim::level::ObjectKind::DualPortal
        | gd_real_sim::level::ObjectKind::TeleportPortal => 0x69A2FF,
        gd_real_sim::level::ObjectKind::Trigger => 0x5A5A7F,
        gd_real_sim::level::ObjectKind::Decoration => 0x2A2A36,
    }
}

fn object_bounds(object: &gd_real_sim::level::LevelObject) -> Option<(f32, f32, f32, f32)> {
    if let Some(corners) = object_oriented_quad(object) {
        return Some(bounds_from_corners(&corners));
    }
    if let gd_real_sim::object_data::HitboxData::Circle { radius: _ } = object.hitbox? {
        let (cx, cy, r) = object_circle(object)?;
        return Some((cx - r, cy - r, cx + r, cy + r));
    }
    None
}

fn object_oriented_quad(object: &gd_real_sim::level::LevelObject) -> Option<[(f32, f32); 4]> {
    let hb = object.hitbox?;
    match hb {
        gd_real_sim::object_data::HitboxData::Box { .. } => render_box_corners(object),
        gd_real_sim::object_data::HitboxData::Slope { half_extents } => {
            Some(transformed_center_box_corners(
                object,
                -half_extents[0],
                -half_extents[1],
                half_extents[0] * 2.0,
                half_extents[1] * 2.0,
            ))
        }
        gd_real_sim::object_data::HitboxData::Circle { .. } => None,
    }
}

fn render_box_corners(object: &gd_real_sim::level::LevelObject) -> Option<[(f32, f32); 4]> {
    // Use the shared collision helper so visualizer rendering is
    // guaranteed to match the simulator's collision footprint exactly.
    // (Both now use `object.x, object.y` as the cell center, with no
    // hidden `+15, +15` shift - see `collision::opengd_box_transform`.)
    gd_real_sim::collision::opengd_box_transform(object).map(|transform| transform.corners)
}

fn object_circle(object: &gd_real_sim::level::LevelObject) -> Option<(f32, f32, f32)> {
    let gd_real_sim::object_data::HitboxData::Circle { radius } = object.hitbox? else {
        return None;
    };
    // Match `sim::circle_hazard_center`: object position is already the
    // cell center, no hidden `+15, +15` translation.
    Some((object.x, object.y, radius))
}

fn transformed_center_box_corners(
    object: &gd_real_sim::level::LevelObject,
    x: f32,
    y: f32,
    w: f32,
    h: f32,
) -> [(f32, f32); 4] {
    let mut out = [(0.0_f32, 0.0_f32); 4];
    for (i, (px, py)) in [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
        .into_iter()
        .enumerate()
    {
        out[i] = slope_local_to_world_point(
            object.x,
            object.y,
            object.rotation,
            object.scale_x,
            object.scale_y,
            px,
            py,
        );
    }
    out
}

fn bounds_from_corners(corners: &[(f32, f32); 4]) -> (f32, f32, f32, f32) {
    let mut min_x = f32::INFINITY;
    let mut min_y = f32::INFINITY;
    let mut max_x = f32::NEG_INFINITY;
    let mut max_y = f32::NEG_INFINITY;
    for (x, y) in corners {
        min_x = min_x.min(*x);
        min_y = min_y.min(*y);
        max_x = max_x.max(*x);
        max_y = max_y.max(*y);
    }
    (min_x, min_y, max_x, max_y)
}

fn slope_surface_segment(object: &gd_real_sim::level::LevelObject) -> Option<(f32, f32, f32, f32)> {
    let gd_real_sim::object_data::HitboxData::Slope { half_extents } = object.hitbox? else {
        return None;
    };
    let hx = half_extents[0];
    let hy = half_extents[1];
    let (x1, y1) = slope_local_to_world_point(
        object.x,
        object.y,
        object.rotation,
        object.scale_x,
        object.scale_y,
        -hx,
        -hy,
    );
    let (x2, y2) = slope_local_to_world_point(
        object.x,
        object.y,
        object.rotation,
        object.scale_x,
        object.scale_y,
        hx,
        hy,
    );
    Some((x1, y1, x2, y2))
}

fn visible_world_rect(view: &Viewport) -> (f32, f32, f32, f32) {
    let xmin = view.view_min_x;
    let xmax = view.view_min_x + ((view.width as f32 - 2.0 * view.margin) / view.scale).max(0.0);
    let ymin = view.view_min_y;
    let ymax = view.view_min_y + ((view.plot_bottom - view.margin) / view.scale).max(0.0);
    (xmin, ymin, xmax, ymax)
}

fn draw_world_grid(buffer: &mut [u32], view: &Viewport) {
    let (xmin, ymin, xmax, ymax) = visible_world_rect(view);
    if !(xmin.is_finite() && ymin.is_finite() && xmax.is_finite() && ymax.is_finite())
        || xmax <= xmin
        || ymax <= ymin
    {
        return;
    }

    let mut gx = GRID_ORIGIN_X + ((xmin - GRID_ORIGIN_X) / BLOCK_SIZE).floor() * BLOCK_SIZE;
    while gx <= xmax + 1e-4 {
        draw_line_world(buffer, view, gx, ymin, gx, ymax, GRID_COLOR);
        gx += BLOCK_SIZE;
    }

    let mut gy = GRID_ORIGIN_Y + ((ymin - GRID_ORIGIN_Y) / BLOCK_SIZE).floor() * BLOCK_SIZE;
    while gy <= ymax + 1e-4 {
        draw_line_world(buffer, view, xmin, gy, xmax, gy, GRID_COLOR);
        gy += BLOCK_SIZE;
    }
}

fn world_to_screen(view: &Viewport, x: f32, y: f32) -> (i32, i32) {
    let sx = view.margin + (x - view.view_min_x) * view.scale;
    let sy = view.plot_bottom - (y - view.view_min_y) * view.scale;
    (sx.round() as i32, sy.round() as i32)
}

/// Press / clicks bar. Maps each tick of `trace` linearly across the window
/// width (so it represents *time*, not world X). A vertical playhead line
/// marks `current_tick`.
fn draw_press_bar(
    buffer: &mut [u32],
    view: &Viewport,
    layout: &Layout,
    trace: &[gd_real_sim::sim::TraceFrame],
    current_tick: usize,
) {
    let (Some(y_start), Some(y_end)) = (layout.press_top, layout.press_bottom) else {
        return;
    };
    fill_band(buffer, view.width, y_start, y_end, PRESS_BAR_UNKNOWN_COLOR);

    if trace.len() < 2 {
        return;
    }
    let total = (trace.len() - 1) as f32;
    for (i, frame) in trace.iter().enumerate() {
        let t0 = i as f32 / total;
        let t1 = ((i + 1) as f32 / total).min(1.0);
        let x0 = (t0 * view.width as f32) as i32;
        let x1 = (t1 * view.width as f32) as i32;
        let color = if frame.pressed {
            PRESS_BAR_PRESSED_COLOR
        } else {
            PRESS_BAR_RELEASED_COLOR
        };
        for y in y_start..y_end {
            let row = y * view.width;
            for x in x0.max(0)..x1.min(view.width as i32) {
                buffer[row + x as usize] = color;
            }
        }
    }
    draw_playhead(
        buffer,
        view.width,
        view.height,
        y_start,
        y_end,
        trace.len(),
        current_tick,
    );
}

/// Y-velocity bar. Each column = one tick (linearly spread across width).
/// Bar is split horizontally at the zero line; positive vy fills upward in
/// green, negative downward in red. Heights are normalized to the trace's
/// max |vy|.
fn draw_vy_bar(
    buffer: &mut [u32],
    view: &Viewport,
    layout: &Layout,
    trace: &[gd_real_sim::sim::TraceFrame],
    current_tick: usize,
) {
    let (Some(y_start), Some(y_end)) = (layout.vy_top, layout.vy_bottom) else {
        return;
    };
    fill_band(buffer, view.width, y_start, y_end, VY_BAR_BG_COLOR);

    let max_abs = trace
        .iter()
        .map(|f| f.state.vy.abs())
        .fold(1.0_f32, f32::max);
    let mid_y = (y_start + y_end) / 2;
    let half_h = (y_end - y_start) as f32 * 0.5 - 1.0;

    // Zero line.
    let row = mid_y * view.width;
    for x in 0..view.width {
        buffer[row + x] = VY_BAR_ZERO_COLOR;
    }

    if trace.is_empty() {
        return;
    }

    let total = trace.len().saturating_sub(1).max(1) as f32;
    for (i, frame) in trace.iter().enumerate() {
        let x0 = ((i as f32 / total) * view.width as f32) as i32;
        let x1 = (((i + 1) as f32 / total) * view.width as f32) as i32;
        let h = ((frame.state.vy.abs() / max_abs) * half_h).round() as i32;
        if h <= 0 {
            continue;
        }
        let (color, top, bot) = if frame.state.vy > 0.0 {
            // Up against gravity: green block above zero line.
            (VY_BAR_UP_COLOR, mid_y as i32 - h, mid_y as i32)
        } else {
            (VY_BAR_DOWN_COLOR, mid_y as i32, mid_y as i32 + h)
        };
        for y in top.max(y_start as i32)..bot.min(y_end as i32) {
            let row = y as usize * view.width;
            for x in x0.max(0)..x1.min(view.width as i32) {
                buffer[row + x as usize] = color;
            }
        }
    }
    draw_playhead(
        buffer,
        view.width,
        view.height,
        y_start,
        y_end,
        trace.len(),
        current_tick,
    );
}

/// Speed portal bar. Colors track canonical speed-portal tiers:
/// 200=orange, 201=blue, 202=green, 203=purple, 1334=red.
/// Any non-tier speed (e.g. start speed) is shown in unknown gray.
fn draw_speed_bar(
    buffer: &mut [u32],
    view: &Viewport,
    layout: &Layout,
    trace: &[gd_real_sim::sim::TraceFrame],
    current_tick: usize,
) {
    let (Some(y_start), Some(y_end)) = (layout.speed_top, layout.speed_bottom) else {
        return;
    };
    fill_band(buffer, view.width, y_start, y_end, SPEED_BAR_BG_COLOR);

    if trace.len() < 2 {
        return;
    }
    let total = (trace.len() - 1) as f32;
    for (i, frame) in trace.iter().enumerate() {
        let t0 = i as f32 / total;
        let t1 = ((i + 1) as f32 / total).min(1.0);
        let x0 = (t0 * view.width as f32) as i32;
        let x1 = (t1 * view.width as f32) as i32;
        let color = speed_color_for_player_speed(frame.state.player_speed);
        for y in y_start..y_end {
            let row = y * view.width;
            for x in x0.max(0)..x1.min(view.width as i32) {
                buffer[row + x as usize] = color;
            }
        }
    }
    draw_playhead(
        buffer,
        view.width,
        view.height,
        y_start,
        y_end,
        trace.len(),
        current_tick,
    );
}

/// Gamemode bar. Colors: cube=green, ship=purple, ball=red, ufo=orange,
/// wave=blue, spider=dark purple, swing=yellow.
fn draw_mode_bar(
    buffer: &mut [u32],
    view: &Viewport,
    layout: &Layout,
    trace: &[gd_real_sim::sim::TraceFrame],
    current_tick: usize,
) {
    let (Some(y_start), Some(y_end)) = (layout.mode_top, layout.mode_bottom) else {
        return;
    };
    fill_band(buffer, view.width, y_start, y_end, MODE_BAR_BG_COLOR);

    if trace.len() < 2 {
        return;
    }
    let total = (trace.len() - 1) as f32;
    for (i, frame) in trace.iter().enumerate() {
        let t0 = i as f32 / total;
        let t1 = ((i + 1) as f32 / total).min(1.0);
        let x0 = (t0 * view.width as f32) as i32;
        let x1 = (t1 * view.width as f32) as i32;
        let color = mode_color(frame.state.mode);
        for y in y_start..y_end {
            let row = y * view.width;
            for x in x0.max(0)..x1.min(view.width as i32) {
                buffer[row + x as usize] = color;
            }
        }
    }
    draw_playhead(
        buffer,
        view.width,
        view.height,
        y_start,
        y_end,
        trace.len(),
        current_tick,
    );
}

fn speed_color_for_player_speed(player_speed: f32) -> u32 {
    const EPS: f32 = 0.01;
    if (player_speed - gd_real_sim::consts::PLAYER_SPEED_0_5X).abs() <= EPS {
        return SPEED_BAR_200_COLOR;
    }
    if (player_speed - gd_real_sim::consts::PLAYER_SPEED_1X).abs() <= EPS {
        return SPEED_BAR_201_COLOR;
    }
    if (player_speed - gd_real_sim::consts::PLAYER_SPEED_2X).abs() <= EPS {
        return SPEED_BAR_202_COLOR;
    }
    if (player_speed - gd_real_sim::consts::PLAYER_SPEED_3X).abs() <= EPS {
        return SPEED_BAR_203_COLOR;
    }
    if (player_speed - gd_real_sim::consts::PLAYER_SPEED_4X).abs() <= EPS {
        return SPEED_BAR_1334_COLOR;
    }
    SPEED_BAR_UNKNOWN_COLOR
}

fn mode_color(mode: gd_real_sim::sim::GameMode) -> u32 {
    match mode {
        gd_real_sim::sim::GameMode::Cube => MODE_BAR_CUBE_COLOR,
        gd_real_sim::sim::GameMode::Ship => MODE_BAR_SHIP_COLOR,
        gd_real_sim::sim::GameMode::Ball => MODE_BAR_BALL_COLOR,
        gd_real_sim::sim::GameMode::Ufo => MODE_BAR_UFO_COLOR,
        gd_real_sim::sim::GameMode::Wave => MODE_BAR_WAVE_COLOR,
        gd_real_sim::sim::GameMode::Spider => MODE_BAR_SPIDER_COLOR,
        gd_real_sim::sim::GameMode::Swing => MODE_BAR_SWING_COLOR,
        gd_real_sim::sim::GameMode::Robot => MODE_BAR_CUBE_COLOR,
    }
}

/// Top scrubber bar. Click-and-drag to seek through `current_tick`.
fn draw_scrubber_bar(
    buffer: &mut [u32],
    view: &Viewport,
    layout: &Layout,
    trace: &[gd_real_sim::sim::TraceFrame],
    current_tick: usize,
) {
    let (y_start, y_end) = (layout.scrubber_top, layout.scrubber_bottom);
    fill_band(buffer, view.width, y_start, y_end, SCRUBBER_BG_COLOR);

    if trace.len() < 2 {
        return;
    }
    let frac = current_tick as f32 / (trace.len() - 1) as f32;
    let fill_x = (frac * view.width as f32) as usize;
    for y in (y_start + 2)..(y_end - 2) {
        let row = y * view.width;
        for x in 0..fill_x.min(view.width) {
            buffer[row + x] = SCRUBBER_FILL_COLOR;
        }
    }
    // Handle: a vertical bar 3px wide.
    let handle_x = fill_x.min(view.width.saturating_sub(1));
    for y in y_start..y_end {
        let row = y * view.width;
        for dx in -1i32..=1 {
            let x = handle_x as i32 + dx;
            if x >= 0 && (x as usize) < view.width {
                buffer[row + x as usize] = SCRUBBER_HANDLE_COLOR;
            }
        }
    }
}

fn draw_playhead(
    buffer: &mut [u32],
    width: usize,
    _height: usize,
    y_start: usize,
    y_end: usize,
    trace_len: usize,
    current_tick: usize,
) {
    if trace_len < 2 {
        return;
    }
    let frac = current_tick as f32 / (trace_len - 1) as f32;
    let x = (frac * width as f32).round() as usize;
    let x = x.min(width.saturating_sub(1));
    for y in y_start..y_end {
        let row = y * width;
        buffer[row + x] = PLAYHEAD_COLOR;
    }
}

fn fill_band(buffer: &mut [u32], width: usize, y_start: usize, y_end: usize, color: u32) {
    for y in y_start..y_end {
        let row = y * width;
        for x in 0..width {
            buffer[row + x] = color;
        }
    }
}

#[derive(Clone, Copy)]
enum SettingsToggle {
    MainHitbox,
    RotatedHitbox,
    CoreHitbox,
    Trail,
    ClickBar,
    VelocityBar,
    SpeedBar,
    ModeBar,
    CanonTrace,
    Exit,
}

struct SettingsRow {
    glyph: char,
    on: bool,
}

fn draw_settings_ui(
    buffer: &mut [u32],
    width: usize,
    _height: usize,
    state: &VizState,
    include_exit: bool,
) {
    let (bx, by, bw, bh) = settings_button_rect(width);
    fill_rect(buffer, width, bx, by, bw, bh, SETTINGS_BTN_BG_OFF);
    draw_glyph_border(buffer, width, bx, by, bw, SETTINGS_BTN_BORDER);
    draw_toggle_glyph(buffer, width, bx + 6, by + 5, 'S', SETTINGS_BTN_BORDER);
    if !state.show_settings_panel {
        return;
    }
    let rows = settings_rows(state, include_exit);
    let (px, py, pw, ph) = settings_panel_rect(width, rows.len());
    fill_rect(buffer, width, px, py, pw, ph, SETTINGS_PANEL_BG);
    draw_rect_border(buffer, width, px, py, pw, ph, SETTINGS_BTN_BORDER);
    for (i, (_, row)) in rows.iter().enumerate() {
        let ry =
            py + SETTINGS_PANEL_PAD + i as i32 * (SETTINGS_PANEL_ROW_H + SETTINGS_PANEL_ROW_GAP);
        let row_bg = if row.on {
            SETTINGS_BTN_BG_ON
        } else {
            SETTINGS_BTN_BG_OFF
        };
        fill_rect(
            buffer,
            width,
            px + SETTINGS_PANEL_PAD,
            ry,
            pw - 2 * SETTINGS_PANEL_PAD,
            SETTINGS_PANEL_ROW_H,
            row_bg,
        );
        draw_rect_border(
            buffer,
            width,
            px + SETTINGS_PANEL_PAD,
            ry,
            pw - 2 * SETTINGS_PANEL_PAD,
            SETTINGS_PANEL_ROW_H,
            SETTINGS_BTN_BORDER,
        );
        draw_toggle_glyph(
            buffer,
            width,
            px + SETTINGS_PANEL_PAD + 6,
            ry + 4,
            row.glyph,
            SETTINGS_BTN_BORDER,
        );
        draw_toggle_glyph(
            buffer,
            width,
            px + pw - SETTINGS_PANEL_PAD - 16,
            ry + 4,
            if row.on { '1' } else { '0' },
            SETTINGS_BTN_BORDER,
        );
    }
}

fn settings_rows(state: &VizState, include_exit: bool) -> Vec<(SettingsToggle, SettingsRow)> {
    let mut rows = vec![
        (
            SettingsToggle::MainHitbox,
            SettingsRow {
                glyph: 'M',
                on: state.show_main_hitbox,
            },
        ),
        (
            SettingsToggle::RotatedHitbox,
            SettingsRow {
                glyph: 'R',
                on: state.show_rotated_hitbox,
            },
        ),
        (
            SettingsToggle::CoreHitbox,
            SettingsRow {
                glyph: 'I',
                on: state.show_core_hitbox,
            },
        ),
        (
            SettingsToggle::Trail,
            SettingsRow {
                glyph: 'T',
                on: state.show_trail,
            },
        ),
        (
            SettingsToggle::ClickBar,
            SettingsRow {
                glyph: 'C',
                on: state.show_clicks,
            },
        ),
        (
            SettingsToggle::VelocityBar,
            SettingsRow {
                glyph: 'V',
                on: state.show_vy,
            },
        ),
        (
            SettingsToggle::SpeedBar,
            SettingsRow {
                glyph: 'P',
                on: state.show_speed,
            },
        ),
        (
            SettingsToggle::ModeBar,
            SettingsRow {
                glyph: 'G',
                on: state.show_mode,
            },
        ),
    ];
    if state.has_canon_trace {
        rows.push((
            SettingsToggle::CanonTrace,
            SettingsRow {
                glyph: 'N',
                on: state.show_canon_trace,
            },
        ));
    }
    if include_exit {
        rows.push((
            SettingsToggle::Exit,
            SettingsRow {
                glyph: 'X',
                on: false,
            },
        ));
    }
    rows
}

fn settings_button_rect(width: usize) -> (i32, i32, i32, i32) {
    let x = width as i32 - 4 - SETTINGS_BTN_SIZE;
    let y = 4 + SCRUBBER_HEIGHT as i32;
    (x, y, SETTINGS_BTN_SIZE, SETTINGS_BTN_SIZE)
}

fn settings_panel_rect(width: usize, row_count: usize) -> (i32, i32, i32, i32) {
    let rows = row_count as i32;
    let panel_h =
        SETTINGS_PANEL_PAD * 2 + rows * SETTINGS_PANEL_ROW_H + (rows - 1) * SETTINGS_PANEL_ROW_GAP;
    let (bx, by, _, bh) = settings_button_rect(width);
    let x = bx - (SETTINGS_PANEL_WIDTH - SETTINGS_BTN_SIZE);
    let y = by + bh + SETTINGS_BTN_GAP;
    (x, y, SETTINGS_PANEL_WIDTH, panel_h)
}

fn settings_panel_toggle_at(
    width: usize,
    mx: f32,
    my: f32,
    state: &VizState,
    include_exit: bool,
) -> Option<SettingsToggle> {
    let rows = settings_rows(state, include_exit);
    let (px, py, pw, ph) = settings_panel_rect(width, rows.len());
    if !point_in_rect(mx, my, (px, py, pw, ph)) {
        return None;
    }
    for i in 0..rows.len() {
        let ry =
            py + SETTINGS_PANEL_PAD + i as i32 * (SETTINGS_PANEL_ROW_H + SETTINGS_PANEL_ROW_GAP);
        let row_rect = (
            px + SETTINGS_PANEL_PAD,
            ry,
            pw - 2 * SETTINGS_PANEL_PAD,
            SETTINGS_PANEL_ROW_H,
        );
        if point_in_rect(mx, my, row_rect) {
            return Some(rows[i].0);
        }
    }
    None
}

fn apply_settings_toggle(state: &mut VizState, toggle: SettingsToggle) -> bool {
    match toggle {
        SettingsToggle::MainHitbox => {
            state.show_main_hitbox = !state.show_main_hitbox;
        }
        SettingsToggle::RotatedHitbox => {
            state.show_rotated_hitbox = !state.show_rotated_hitbox;
        }
        SettingsToggle::CoreHitbox => {
            state.show_core_hitbox = !state.show_core_hitbox;
        }
        SettingsToggle::Trail => {
            state.show_trail = !state.show_trail;
        }
        SettingsToggle::ClickBar => {
            state.show_clicks = !state.show_clicks;
        }
        SettingsToggle::VelocityBar => {
            state.show_vy = !state.show_vy;
        }
        SettingsToggle::SpeedBar => {
            state.show_speed = !state.show_speed;
        }
        SettingsToggle::ModeBar => {
            state.show_mode = !state.show_mode;
        }
        SettingsToggle::CanonTrace => {
            state.show_canon_trace = !state.show_canon_trace;
        }
        SettingsToggle::Exit => return true,
    }
    false
}

fn point_in_rect(mx: f32, my: f32, rect: (i32, i32, i32, i32)) -> bool {
    let (x, y, w, h) = rect;
    mx >= x as f32 && mx < (x + w) as f32 && my >= y as f32 && my < (y + h) as f32
}

fn fill_rect(buffer: &mut [u32], width: usize, x: i32, y: i32, w: i32, h: i32, color: u32) {
    if w <= 0 || h <= 0 {
        return;
    }
    let x0 = x.max(0) as usize;
    let y0 = y.max(0) as usize;
    let x1 = (x + w).max(0) as usize;
    let y1 = (y + h).max(0) as usize;
    let x1 = x1.min(width);
    let height = buffer.len() / width;
    let y1 = y1.min(height);
    for yy in y0..y1 {
        let row = yy * width;
        for xx in x0..x1 {
            buffer[row + xx] = color;
        }
    }
}

fn draw_toggle_glyph(buffer: &mut [u32], width: usize, x: i32, y: i32, glyph: char, color: u32) {
    match glyph {
        'C' => {
            draw_h_line(buffer, width, x, x + 8, y, color);
            draw_h_line(buffer, width, x, x + 8, y + 10, color);
            draw_v_line(buffer, width, x, y, y + 10, color);
        }
        'V' => {
            stroke_segment(buffer, width, x, y, x + 4, y + 10, color);
            stroke_segment(buffer, width, x + 8, y, x + 4, y + 10, color);
        }
        'M' => {
            draw_v_line(buffer, width, x, y, y + 10, color);
            draw_v_line(buffer, width, x + 8, y, y + 10, color);
            stroke_segment(buffer, width, x, y, x + 4, y + 5, color);
            stroke_segment(buffer, width, x + 8, y, x + 4, y + 5, color);
        }
        'R' => {
            draw_v_line(buffer, width, x, y, y + 10, color);
            draw_h_line(buffer, width, x, x + 6, y, color);
            draw_h_line(buffer, width, x, x + 6, y + 5, color);
            draw_v_line(buffer, width, x + 6, y, y + 5, color);
            stroke_segment(buffer, width, x + 3, y + 5, x + 8, y + 10, color);
        }
        'I' => {
            draw_v_line(buffer, width, x + 4, y, y + 10, color);
            draw_h_line(buffer, width, x, x + 8, y, color);
            draw_h_line(buffer, width, x, x + 8, y + 10, color);
        }
        'T' => {
            draw_h_line(buffer, width, x, x + 8, y, color);
            draw_v_line(buffer, width, x + 4, y, y + 10, color);
        }
        'S' => {
            draw_h_line(buffer, width, x, x + 8, y, color);
            draw_h_line(buffer, width, x, x + 8, y + 5, color);
            draw_h_line(buffer, width, x, x + 8, y + 10, color);
            draw_v_line(buffer, width, x, y, y + 5, color);
            draw_v_line(buffer, width, x + 8, y + 5, y + 10, color);
        }
        'G' => {
            draw_h_line(buffer, width, x, x + 8, y, color);
            draw_h_line(buffer, width, x, x + 8, y + 10, color);
            draw_v_line(buffer, width, x, y, y + 10, color);
            draw_h_line(buffer, width, x + 4, x + 8, y + 6, color);
            draw_v_line(buffer, width, x + 8, y + 6, y + 10, color);
        }
        'N' => {
            draw_v_line(buffer, width, x, y, y + 10, color);
            draw_v_line(buffer, width, x + 8, y, y + 10, color);
            stroke_segment(buffer, width, x, y, x + 8, y + 10, color);
        }
        'X' => {
            stroke_segment(buffer, width, x, y, x + 8, y + 10, color);
            stroke_segment(buffer, width, x + 8, y, x, y + 10, color);
        }
        '1' => {
            draw_v_line(buffer, width, x + 4, y, y + 10, color);
        }
        '0' => {
            draw_h_line(buffer, width, x, x + 8, y, color);
            draw_h_line(buffer, width, x, x + 8, y + 10, color);
            draw_v_line(buffer, width, x, y, y + 10, color);
            draw_v_line(buffer, width, x + 8, y, y + 10, color);
        }
        _ => {}
    }
}

fn draw_glyph_border(buffer: &mut [u32], width: usize, x: i32, y: i32, size: i32, color: u32) {
    draw_h_line(buffer, width, x, x + size - 1, y, color);
    draw_h_line(buffer, width, x, x + size - 1, y + size - 1, color);
    draw_v_line(buffer, width, x, y, y + size - 1, color);
    draw_v_line(buffer, width, x + size - 1, y, y + size - 1, color);
}

fn draw_rect_border(buffer: &mut [u32], width: usize, x: i32, y: i32, w: i32, h: i32, color: u32) {
    if w <= 0 || h <= 0 {
        return;
    }
    draw_h_line(buffer, width, x, x + w - 1, y, color);
    draw_h_line(buffer, width, x, x + w - 1, y + h - 1, color);
    draw_v_line(buffer, width, x, y, y + h - 1, color);
    draw_v_line(buffer, width, x + w - 1, y, y + h - 1, color);
}

fn draw_h_line(buffer: &mut [u32], width: usize, x0: i32, x1: i32, y: i32, color: u32) {
    if y < 0 {
        return;
    }
    let row_start = y as usize * width;
    if row_start >= buffer.len() {
        return;
    }
    for x in x0.max(0)..=x1.min(width as i32 - 1) {
        buffer[row_start + x as usize] = color;
    }
}

fn draw_v_line(buffer: &mut [u32], width: usize, x: i32, y0: i32, y1: i32, color: u32) {
    if x < 0 || (x as usize) >= width {
        return;
    }
    for y in y0.max(0)..=y1 {
        let idx = y as usize * width + x as usize;
        if idx < buffer.len() {
            buffer[idx] = color;
        }
    }
}

fn stroke_segment(
    buffer: &mut [u32],
    width: usize,
    mut x0: i32,
    mut y0: i32,
    x1: i32,
    y1: i32,
    color: u32,
) {
    let dx = (x1 - x0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let dy = -(y1 - y0).abs();
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;
    loop {
        if x0 >= 0 && (x0 as usize) < width && y0 >= 0 {
            let idx = y0 as usize * width + x0 as usize;
            if idx < buffer.len() {
                buffer[idx] = color;
            }
        }
        if x0 == x1 && y0 == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x0 += sx;
        }
        if e2 <= dx {
            err += dx;
            y0 += sy;
        }
    }
}

fn draw_rect_outline(
    buffer: &mut [u32],
    view: &Viewport,
    x1: f32,
    y1: f32,
    x2: f32,
    y2: f32,
    color: u32,
) {
    let (sx1, sy1) = world_to_screen(view, x1, y1);
    let (sx2, sy2) = world_to_screen(view, x2, y2);
    let left = sx1.min(sx2);
    let right = sx1.max(sx2);
    let top = sy1.min(sy2);
    let bottom = sy1.max(sy2);
    draw_line_pixels(buffer, view, left, top, right, top, color);
    draw_line_pixels(buffer, view, right, top, right, bottom, color);
    draw_line_pixels(buffer, view, right, bottom, left, bottom, color);
    draw_line_pixels(buffer, view, left, bottom, left, top, color);
}

fn draw_rect_outline_alpha(
    buffer: &mut [u32],
    view: &Viewport,
    x1: f32,
    y1: f32,
    x2: f32,
    y2: f32,
    color: u32,
    alpha: f32,
) {
    let (sx1, sy1) = world_to_screen(view, x1, y1);
    let (sx2, sy2) = world_to_screen(view, x2, y2);
    let left = sx1.min(sx2);
    let right = sx1.max(sx2);
    let top = sy1.min(sy2);
    let bottom = sy1.max(sy2);
    draw_line_pixels_alpha(buffer, view, left, top, right, top, color, alpha);
    draw_line_pixels_alpha(buffer, view, right, top, right, bottom, color, alpha);
    draw_line_pixels_alpha(buffer, view, right, bottom, left, bottom, color, alpha);
    draw_line_pixels_alpha(buffer, view, left, bottom, left, top, color, alpha);
}

fn draw_quad_outline(buffer: &mut [u32], view: &Viewport, corners: [(f32, f32); 4], color: u32) {
    for i in 0..4 {
        let (x1, y1) = corners[i];
        let (x2, y2) = corners[(i + 1) % 4];
        draw_line_world(buffer, view, x1, y1, x2, y2, color);
    }
}

fn draw_quad_outline_alpha(
    buffer: &mut [u32],
    view: &Viewport,
    corners: [(f32, f32); 4],
    color: u32,
    alpha: f32,
) {
    for i in 0..4 {
        let (x1, y1) = corners[i];
        let (x2, y2) = corners[(i + 1) % 4];
        let (sx1, sy1) = world_to_screen(view, x1, y1);
        let (sx2, sy2) = world_to_screen(view, x2, y2);
        draw_line_pixels_alpha(buffer, view, sx1, sy1, sx2, sy2, color, alpha);
    }
}

fn rotated_square_corners(cx: f32, cy: f32, half: f32, deg: f32) -> [(f32, f32); 4] {
    let rad = deg.to_radians();
    let c = rad.cos();
    let s = rad.sin();
    let pts = [(-half, -half), (half, -half), (half, half), (-half, half)];
    let mut out = [(0.0, 0.0); 4];
    for (i, (x, y)) in pts.iter().enumerate() {
        out[i] = (cx + x * c - y * s, cy + x * s + y * c);
    }
    out
}

fn draw_player_hitboxes(
    buffer: &mut [u32],
    view: &Viewport,
    state: gd_real_sim::sim::PlayerState,
    viz: &VizState,
    alpha: f32,
) {
    let half = player_half(state.mini);
    if viz.show_main_hitbox {
        draw_rect_outline_alpha(
            buffer,
            view,
            state.x - half,
            state.y - half,
            state.x + half,
            state.y + half,
            HITBOX_MAIN_COLOR,
            alpha,
        );
    }
    if viz.show_rotated_hitbox {
        let rotation = if state.mode == gd_real_sim::sim::GameMode::Ball {
            0.0
        } else {
            state.rotation
        };
        let corners = rotated_square_corners(state.x, state.y, half, rotation);
        draw_quad_outline_alpha(buffer, view, corners, HITBOX_ROTATED_COLOR, alpha);
    }
    if viz.show_core_hitbox {
        let inner_half = if state.mode == gd_real_sim::sim::GameMode::Cube {
            if state.mini { 5.0 } else { 4.5 }
        } else {
            half * 0.3
        };
        draw_rect_outline_alpha(
            buffer,
            view,
            state.x - inner_half,
            state.y - inner_half,
            state.x + inner_half,
            state.y + inner_half,
            HITBOX_CORE_COLOR,
            alpha,
        );
    }
}

fn draw_circle_outline(
    buffer: &mut [u32],
    view: &Viewport,
    cx: f32,
    cy: f32,
    radius: f32,
    color: u32,
) {
    let (scx, scy) = world_to_screen(view, cx, cy);
    let r = (radius * view.scale).round().max(1.0) as i32;
    let mut x = r;
    let mut y = 0;
    let mut err = 0;

    while x >= y {
        for (dx, dy) in [
            (x, y),
            (y, x),
            (-y, x),
            (-x, y),
            (-x, -y),
            (-y, -x),
            (y, -x),
            (x, -y),
        ] {
            set_pixel(buffer, view.width, view.height, scx + dx, scy + dy, color);
        }
        y += 1;
        if err <= 0 {
            err += 2 * y + 1;
        } else {
            x -= 1;
            err -= 2 * x + 1;
        }
    }
}

fn draw_line_world(
    buffer: &mut [u32],
    view: &Viewport,
    x1: f32,
    y1: f32,
    x2: f32,
    y2: f32,
    color: u32,
) {
    let (sx1, sy1) = world_to_screen(view, x1, y1);
    let (sx2, sy2) = world_to_screen(view, x2, y2);
    draw_line_pixels(buffer, view, sx1, sy1, sx2, sy2, color);
}

fn draw_line_pixels(
    buffer: &mut [u32],
    view: &Viewport,
    mut x0: i32,
    mut y0: i32,
    x1: i32,
    y1: i32,
    color: u32,
) {
    let dx = (x1 - x0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let dy = -(y1 - y0).abs();
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;

    loop {
        set_pixel(buffer, view.width, view.height, x0, y0, color);
        if x0 == x1 && y0 == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x0 += sx;
        }
        if e2 <= dx {
            err += dx;
            y0 += sy;
        }
    }
}

fn draw_line_pixels_alpha(
    buffer: &mut [u32],
    view: &Viewport,
    mut x0: i32,
    mut y0: i32,
    x1: i32,
    y1: i32,
    color: u32,
    alpha: f32,
) {
    let dx = (x1 - x0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let dy = -(y1 - y0).abs();
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;

    loop {
        set_pixel_alpha(buffer, view.width, view.height, x0, y0, color, alpha);
        if x0 == x1 && y0 == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x0 += sx;
        }
        if e2 <= dx {
            err += dx;
            y0 += sy;
        }
    }
}

fn set_pixel(buffer: &mut [u32], width: usize, height: usize, x: i32, y: i32, color: u32) {
    if x < 0 || y < 0 || x >= width as i32 || y >= height as i32 {
        return;
    }
    let idx = y as usize * width + x as usize;
    buffer[idx] = color;
}

fn set_pixel_alpha(
    buffer: &mut [u32],
    width: usize,
    height: usize,
    x: i32,
    y: i32,
    color: u32,
    alpha: f32,
) {
    if x < 0 || y < 0 || x >= width as i32 || y >= height as i32 {
        return;
    }
    let idx = y as usize * width + x as usize;
    let dst = buffer[idx];
    buffer[idx] = blend_rgb(dst, color, alpha);
}

fn blend_rgb(dst: u32, src: u32, alpha: f32) -> u32 {
    let a = alpha.clamp(0.0, 1.0);
    let inv = 1.0 - a;
    let dr = ((dst >> 16) & 0xFF) as f32;
    let dg = ((dst >> 8) & 0xFF) as f32;
    let db = (dst & 0xFF) as f32;
    let sr = ((src >> 16) & 0xFF) as f32;
    let sg = ((src >> 8) & 0xFF) as f32;
    let sb = (src & 0xFF) as f32;
    let r = (dr * inv + sr * a).round() as u32;
    let g = (dg * inv + sg * a).round() as u32;
    let b = (db * inv + sb * a).round() as u32;
    (r << 16) | (g << 8) | b
}

fn player_half(mini: bool) -> f32 {
    if mini { 9.0 } else { 15.0 }
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use gd_real_sim::{
        level::{Level, LevelObject, ObjectKind},
        sim::{GameMode, PlayerState, TraceFrame},
    };

    use super::{
        apply_tick_offset, level_progress_percent, live_attempt_bitstring,
        resample_click_bits_linear,
    };

    #[test]
    fn apply_tick_offset_positive_trims_front() {
        assert_eq!(apply_tick_offset("001101", 2), "1101");
        assert_eq!(apply_tick_offset("001101", 99), "");
    }

    #[test]
    fn apply_tick_offset_negative_prepends_zeros() {
        assert_eq!(apply_tick_offset("101", -3), "000101");
    }

    #[test]
    fn resample_click_bits_linear_60_to_240_upsamples() {
        let source = vec![0_u8, 1_u8];
        let out = resample_click_bits_linear(&source, 60, 240);
        assert_eq!(out, "00001111");
    }

    #[test]
    fn resample_click_bits_linear_handles_empty() {
        let out = resample_click_bits_linear(&[], 60, 240);
        assert!(out.is_empty());
    }

    #[test]
    fn live_attempt_bitstring_preserves_full_attempt_timing() {
        let trace = vec![
            frame_at(-8.0, true),
            frame_at(-0.5, true),
            frame_at(0.2, true),
            frame_at(6.0, false),
            frame_at(12.0, true),
        ];

        assert_eq!(live_attempt_bitstring(&trace), "11101");
    }

    #[test]
    fn level_progress_percent_uses_finish_portal_when_present() {
        let level = Level {
            header: HashMap::new(),
            objects: vec![
                object(1, ObjectKind::Solid, 180.0),
                object(3607, ObjectKind::Trigger, 360.0),
            ],
        };

        assert_eq!(level_progress_percent(&level, -15.0), 0.0);
        assert!((level_progress_percent(&level, 180.0) - 50.0).abs() < 0.001);
        assert_eq!(level_progress_percent(&level, 500.0), 100.0);
    }

    fn frame_at(x: f32, pressed: bool) -> TraceFrame {
        TraceFrame {
            tick: 0,
            time: 0.0,
            pressed,
            state: PlayerState {
                x,
                y: 105.0,
                vx: 0.0,
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
                snapped_object: None,
                snap_distance: 0.0,
                dash_rotation_blocks_remaining: 0.0,
                dash_angle_deg: 0.0,
                hold_ticks: 0,
                pending_yvel_next_tick: 0.0,
            },
            partner: None,
        }
    }

    fn object(object_id: u32, kind: ObjectKind, x: f32) -> LevelObject {
        LevelObject {
            object_id,
            x,
            y: 90.0,
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
}
