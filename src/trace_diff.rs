use serde::Serialize;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DiffConfig {
    pub epsilon_x: f32,
    pub epsilon_y: f32,
    pub min_motion: f32,
}

impl Default for DiffConfig {
    fn default() -> Self {
        Self {
            epsilon_x: 0.25,
            epsilon_y: 0.25,
            min_motion: 0.001,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct TracePoint {
    pub source_tick: usize,
    pub time_seconds: f32,
    pub x: f32,
    pub y: f32,
    pub dx: f32,
    pub dy: f32,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Divergence {
    pub step: usize,
    pub real: TracePoint,
    pub sim: TracePoint,
    pub error_x: f32,
    pub error_y: f32,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct DiffReport {
    pub real_start: TracePoint,
    pub sim_start: TracePoint,
    pub compared_steps: usize,
    pub real_total_steps: usize,
    pub sim_total_steps: usize,
    pub sim_stride: usize,
    pub offset_x: f32,
    pub offset_y: f32,
    pub first_divergence: Option<Divergence>,
    pub max_error_x: f32,
    pub max_error_y: f32,
}

pub fn compare_logs(
    real_log: &str,
    sim_log: &str,
    config: DiffConfig,
) -> Result<DiffReport, String> {
    let real = normalize_real_log(real_log, config.min_motion)?;
    let sim = normalize_sim_log(sim_log)?;
    let real_start = *real
        .first()
        .ok_or_else(|| "real log has no meaningful motion rows".to_owned())?;
    let sim_start = *sim
        .first()
        .ok_or_else(|| "sim log has no player rows".to_owned())?;
    let sim_stride = infer_sim_stride(&real, &sim, config.min_motion);
    let sim_aligned = sim.iter().step_by(sim_stride).copied().collect::<Vec<_>>();
    let offset_x = real_start.x - sim_start.x;
    let offset_y = real_start.y - sim_start.y;
    let compared_steps = real.len().min(sim_aligned.len());
    let mut first_divergence = None;
    let mut max_error_x = 0.0_f32;
    let mut max_error_y = 0.0_f32;

    for step in 0..compared_steps {
        let real_point = real[step];
        let sim_point = sim_aligned[step];
        let aligned_sim_x = sim_point.x + offset_x;
        let aligned_sim_y = sim_point.y + offset_y;
        let error_x = (real_point.x - aligned_sim_x).abs();
        let error_y = (real_point.y - aligned_sim_y).abs();
        max_error_x = max_error_x.max(error_x);
        max_error_y = max_error_y.max(error_y);
        if first_divergence.is_none() && (error_x > config.epsilon_x || error_y > config.epsilon_y)
        {
            first_divergence = Some(Divergence {
                step,
                real: real_point,
                sim: sim_point,
                error_x,
                error_y,
            });
        }
    }

    Ok(DiffReport {
        real_start,
        sim_start,
        compared_steps,
        real_total_steps: real.len(),
        sim_total_steps: sim.len(),
        sim_stride,
        offset_x,
        offset_y,
        first_divergence,
        max_error_x,
        max_error_y,
    })
}

pub fn normalize_real_log(log: &str, min_motion: f32) -> Result<Vec<TracePoint>, String> {
    let mut rows = Vec::new();
    for line in log.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with("tick") {
            continue;
        }
        let cols = trimmed.split_whitespace().collect::<Vec<_>>();
        if cols.len() < 6 {
            continue;
        }
        let source_tick = parse_col::<usize>(&cols, 0, "real tick")?;
        let t_ms = parse_col::<f32>(&cols, 1, "real t_ms")?;
        let x = parse_col::<f32>(&cols, 2, "real x")?;
        let y = parse_col::<f32>(&cols, 3, "real y")?;
        let dx = parse_col::<f32>(&cols, 4, "real dx")?;
        let dy = parse_col::<f32>(&cols, 5, "real dy")?;
        rows.push(TracePoint {
            source_tick,
            time_seconds: t_ms / 1000.0,
            x,
            y,
            dx,
            dy,
        });
    }

    let start = rows
        .iter()
        .position(|row| row.x.abs() > min_motion || row.dx.abs() > min_motion)
        .unwrap_or(rows.len());

    let mut normalized = Vec::new();
    let mut last_position: Option<(f32, f32)> = None;
    for row in rows.into_iter().skip(start) {
        let duplicate_position = last_position
            .map(|(x, y)| (row.x - x).abs() <= min_motion && (row.y - y).abs() <= min_motion)
            .unwrap_or(false);
        let zero_delta = row.dx.abs() <= min_motion && row.dy.abs() <= min_motion;
        if duplicate_position && zero_delta {
            continue;
        }
        last_position = Some((row.x, row.y));
        normalized.push(row);
    }
    Ok(normalized)
}

pub fn normalize_sim_log(log: &str) -> Result<Vec<TracePoint>, String> {
    let mut rows = Vec::new();
    let mut previous: Option<(f32, f32)> = None;
    for line in log.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with("tick") {
            continue;
        }
        let cols = trimmed.split_whitespace().collect::<Vec<_>>();
        if cols.len() < 18 {
            continue;
        }
        let player_index = parse_col::<u8>(&cols, 2, "sim player index")?;
        if player_index != 0 {
            continue;
        }
        let source_tick = parse_col::<usize>(&cols, 0, "sim tick")?;
        let time_seconds = parse_col::<f32>(&cols, 1, "sim time")?;
        let x = parse_col::<f32>(&cols, 9, "sim x")?;
        let y = parse_col::<f32>(&cols, 10, "sim y")?;
        let (dx, dy) = previous
            .map(|(px, py)| (x - px, y - py))
            .unwrap_or((0.0, 0.0));
        previous = Some((x, y));
        rows.push(TracePoint {
            source_tick,
            time_seconds,
            x,
            y,
            dx,
            dy,
        });
    }
    Ok(rows)
}

fn infer_sim_stride(real: &[TracePoint], sim: &[TracePoint], min_motion: f32) -> usize {
    let Some(real_dx) = first_motion_dx(real, min_motion) else {
        return 1;
    };
    let Some(sim_dx) = first_motion_dx(sim, min_motion) else {
        return 1;
    };
    if sim_dx.abs() <= min_motion {
        return 1;
    }
    let stride = (real_dx.abs() / sim_dx.abs()).round() as usize;
    stride.max(1).min(16)
}

fn first_motion_dx(points: &[TracePoint], min_motion: f32) -> Option<f32> {
    points
        .iter()
        .find(|point| point.dx.abs() > min_motion)
        .map(|point| point.dx)
}

fn parse_col<T>(cols: &[&str], index: usize, name: &str) -> Result<T, String>
where
    T: std::str::FromStr,
    T::Err: std::fmt::Display,
{
    cols.get(index)
        .ok_or_else(|| format!("missing {name} column"))?
        .parse::<T>()
        .map_err(|error| format!("invalid {name}: {error}"))
}
