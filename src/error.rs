use thiserror::Error;

pub type SimResult<T> = Result<T, SimError>;

#[derive(Debug, Error)]
pub enum SimError {
    #[error("failed to parse object defaults: {0}")]
    ObjectData(String),
    #[error("failed to parse level: {0}")]
    LevelParse(String),
    #[error("invalid click tape: {0}")]
    InvalidClickTape(String),
    #[error("unsupported feature: {feature} on object id {object_id}")]
    UnsupportedFeature { feature: String, object_id: u32 },
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),
    #[error("json error: {0}")]
    Json(#[from] serde_json::Error),
}
