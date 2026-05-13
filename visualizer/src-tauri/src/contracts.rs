use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Deserialize)]
pub struct ParseLevelRequest {
    pub level_string: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct RenderableObject {
    pub object_id: u32,
    pub x: f32,
    pub y: f32,
    pub rotation: f32,
    pub scale_x: f32,
    pub scale_y: f32,
    pub kind: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct ParsedLevelResponse {
    pub object_count: usize,
    pub objects: Vec<RenderableObject>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct SimulateRequest {
    pub level_string: String,
    pub click_bitstring: Option<String>,
    pub max_ticks: Option<usize>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ListLocalLevelsRequest {
    pub path_override: Option<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct LocalLevelEntry {
    pub name: String,
    pub raw_payload: String,
    pub level_string: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct SearchOfficialLevelsRequest {
    pub query: String,
    pub page: Option<u32>,
}

#[derive(Debug, Clone, Serialize)]
pub struct OfficialLevelSearchItem {
    pub level_id: String,
    pub name: String,
    pub description: String,
    pub creator_name: String,
    pub creator_account_id: Option<String>,
    pub downloads: i64,
    pub likes: i64,
    pub difficulty: i64,
    pub length: i64,
    pub object_count: Option<usize>,
    pub song_id: Option<String>,
    pub custom_song_id: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct DownloadOfficialLevelRequest {
    pub level_id: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct OfficialLevelDownload {
    pub level_id: String,
    pub name: String,
    pub creator_name: String,
    pub level_string: String,
    pub description: String,
    pub length: Option<i64>,
    pub object_count: usize,
    pub song_id: Option<String>,
    pub custom_song_id: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BitstringEntry {
    pub id: String,
    pub name: String,
    pub bitstring: String,
    pub created_at: String,
    pub source_kind: String,
    pub notes: Option<String>,
    pub linked_level_id: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct UpsertBitstringRequest {
    pub id: Option<String>,
    pub name: String,
    pub bitstring: String,
    pub source_kind: String,
    pub notes: Option<String>,
    pub linked_level_id: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct DeleteBitstringRequest {
    pub id: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ListLiveAttemptsRequest {
    pub level_id: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LiveAttemptEntry {
    pub id: String,
    pub created_at_ms: u64,
    pub outcome: String,
    pub percent: f32,
    pub processed_clicks: usize,
    pub bitstring: String,
    pub tick: usize,
}

#[derive(Debug, Clone, Deserialize)]
pub struct LaunchNativeVisualizerRequest {
    pub level_string: String,
    pub click_bitstring: Option<String>,
    pub mode: Option<NativeVisualizerMode>,
    pub level_id: Option<String>,
}

#[derive(Debug, Clone, Copy, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum NativeVisualizerMode {
    Replay,
    Play,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ParseLocalLevelsBlobRequest {
    pub bytes_b64: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct DecodeClicksBinBlobRequest {
    pub bytes_b64: String,
    pub source_hz: Option<u32>,
}
