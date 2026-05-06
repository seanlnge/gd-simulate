pub const PHYSICS_TICKS_PER_SECOND: f32 = 240.0;

/// Mirrors the substep calculation shape in GJBaseGameLayer_update.cpp.
pub fn physics_steps(delta_seconds: f32, timewarp: f32) -> usize {
    let divisor = timewarp.min(1.0).max(f32::EPSILON);
    let steps = ((delta_seconds * PHYSICS_TICKS_PER_SECOND) / divisor).floor() as usize;
    steps.max(1)
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StepTiming {
    pub step_count: usize,
    pub physics_delta: f32,
    pub physics_second: f32,
    pub warped_physics_delta: f32,
}

impl StepTiming {
    pub fn from_frame_delta(delta_seconds: f32, timewarp: f32) -> Self {
        let step_count = physics_steps(delta_seconds, timewarp);
        Self {
            step_count,
            physics_delta: delta_seconds / step_count as f32,
            physics_second: (delta_seconds * 60.0) / step_count as f32,
            warped_physics_delta: (delta_seconds / timewarp.max(f32::EPSILON)) / step_count as f32,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpeedProfile {
    pub y_start: f32,
    pub gravity: f32,
    pub speed_multiplier: f32,
}

impl SpeedProfile {
    /// Port of the hardcoded thresholds in PlayerObject_updateTimeMod.cpp.
    pub fn for_player_speed(speed: f32) -> Self {
        if speed > 1.0 {
            Self {
                y_start: 11.420032,
                gravity: 0.957199,
                speed_multiplier: 5.870002,
            }
        } else if speed > 0.85 {
            Self {
                y_start: 11.1800318,
                gravity: 0.958199024,
                speed_multiplier: 5.77000189,
            }
        } else if speed == 0.7 {
            Self {
                y_start: 10.620032,
                gravity: 0.940199,
                speed_multiplier: 5.980002,
            }
        } else if speed == 1.3 || speed == 1.6 {
            Self {
                y_start: 11.230032,
                gravity: 0.961199,
                speed_multiplier: 6.000002,
            }
        } else {
            Self {
                y_start: 11.1800318,
                gravity: 0.958199024,
                speed_multiplier: 5.77000189,
            }
        }
    }
}
