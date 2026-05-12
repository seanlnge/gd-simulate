use std::path::PathBuf;

use base64::{engine::general_purpose::STANDARD, Engine};
use gd_real_sim::save::{decode_local_levels_dat, parse_local_levels_xml, read_local_levels};

use crate::contracts::{ListLocalLevelsRequest, LocalLevelEntry, ParseLocalLevelsBlobRequest};

fn default_local_levels_path() -> Result<PathBuf, String> {
    let base = std::env::var("LOCALAPPDATA")
        .map_err(|_| "LOCALAPPDATA is not set on this machine".to_owned())?;
    Ok(PathBuf::from(base)
        .join("GeometryDash")
        .join("CCLocalLevels.dat"))
}

#[tauri::command]
pub fn list_local_levels(
    request: Option<ListLocalLevelsRequest>,
) -> Result<Vec<LocalLevelEntry>, String> {
    let path = request
        .and_then(|r| r.path_override.map(PathBuf::from))
        .map(Ok)
        .unwrap_or_else(default_local_levels_path)?;

    let levels = read_local_levels(&path).map_err(|e| format!("failed reading {:?}: {e}", path))?;
    Ok(to_entries(levels))
}

#[tauri::command]
pub fn parse_local_levels_blob(
    request: ParseLocalLevelsBlobRequest,
) -> Result<Vec<LocalLevelEntry>, String> {
    let bytes = STANDARD
        .decode(request.bytes_b64.trim())
        .map_err(|e| e.to_string())?;
    let xml = decode_local_levels_dat(&bytes).map_err(|e| e.to_string())?;
    let levels = parse_local_levels_xml(&xml).map_err(|e| e.to_string())?;
    Ok(to_entries(levels))
}

fn to_entries(levels: Vec<gd_real_sim::save::LocalLevel>) -> Vec<LocalLevelEntry> {
    levels
        .into_iter()
        .map(|level| LocalLevelEntry {
            name: level.name,
            raw_payload: level.raw_payload,
            level_string: level.levelstring,
        })
        .collect()
}
