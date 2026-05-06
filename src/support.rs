use serde_json::Value;

use crate::SimResult;

const SUPPORT_MATRIX_JSON: &str = include_str!("../support_matrix.json");

#[derive(Debug, Clone)]
pub struct SupportMatrix {
    matrix: Value,
}

impl SupportMatrix {
    pub fn load_embedded() -> SimResult<Self> {
        Ok(Self {
            matrix: serde_json::from_str(SUPPORT_MATRIX_JSON)?,
        })
    }

    pub fn status(&self, path: &str) -> Option<&str> {
        path.split('.')
            .try_fold(&self.matrix, |current, segment| current.get(segment))
            .and_then(Value::as_str)
    }
}
