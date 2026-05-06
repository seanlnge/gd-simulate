use crate::{SimError, SimResult};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClickTape {
    pressed: Vec<bool>,
}

impl ClickTape {
    pub fn from_bits(bits: &str) -> SimResult<Self> {
        let mut pressed = Vec::with_capacity(bits.len());
        for (idx, ch) in bits.chars().enumerate() {
            match ch {
                '0' => pressed.push(false),
                '1' => pressed.push(true),
                _ => {
                    return Err(SimError::InvalidClickTape(format!(
                        "character {ch:?} at index {idx} is not 0 or 1"
                    )));
                }
            }
        }
        Ok(Self { pressed })
    }

    pub fn len(&self) -> usize {
        self.pressed.len()
    }

    pub fn is_empty(&self) -> bool {
        self.pressed.is_empty()
    }

    pub fn is_pressed(&self, tick: usize) -> bool {
        self.pressed.get(tick).copied().unwrap_or(false)
    }

    pub fn is_press_start(&self, tick: usize) -> bool {
        self.is_pressed(tick) && (tick == 0 || !self.is_pressed(tick - 1))
    }

    pub fn is_release(&self, tick: usize) -> bool {
        !self.is_pressed(tick) && tick > 0 && self.is_pressed(tick - 1)
    }
}
