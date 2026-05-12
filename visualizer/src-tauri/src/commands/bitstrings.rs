use std::{
    fs,
    path::{Path, PathBuf},
};

use chrono::Utc;
use uuid::Uuid;

use crate::contracts::{BitstringEntry, DeleteBitstringRequest, UpsertBitstringRequest};

fn bitstrings_dir() -> Result<PathBuf, String> {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .ok_or_else(|| "could not resolve visualizer directory".to_owned())?
        .join("bitstrings");
    fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    Ok(dir)
}

fn entry_path(id: &str) -> Result<PathBuf, String> {
    if id.is_empty() || id.contains(['\\', '/', ':', '*', '?', '"', '<', '>', '|']) {
        return Err("invalid bitstring id".to_owned());
    }
    Ok(bitstrings_dir()?.join(format!("{id}.json")))
}

#[tauri::command]
pub fn list_bitstrings() -> Result<Vec<BitstringEntry>, String> {
    let dir = bitstrings_dir()?;
    let mut items = Vec::new();
    for entry in fs::read_dir(&dir).map_err(|e| e.to_string())? {
        let entry = entry.map_err(|e| e.to_string())?;
        let path = entry.path();
        if path.extension().and_then(|ext| ext.to_str()) != Some("json") {
            continue;
        }
        let json = fs::read_to_string(&path).map_err(|e| e.to_string())?;
        let parsed = serde_json::from_str::<BitstringEntry>(&json).map_err(|e| e.to_string())?;
        items.push(parsed);
    }
    items.sort_by(|a, b| b.created_at.cmp(&a.created_at));
    Ok(items)
}

#[tauri::command]
pub fn upsert_bitstring(request: UpsertBitstringRequest) -> Result<BitstringEntry, String> {
    if request.bitstring.chars().any(|ch| ch != '0' && ch != '1') {
        return Err("bitstring can only contain 0 and 1".to_owned());
    }
    let id = request.id.unwrap_or_else(|| Uuid::new_v4().to_string());
    let existing = entry_path(&id)?;
    let created_at = if existing.exists() {
        let old = fs::read_to_string(&existing).map_err(|e| e.to_string())?;
        serde_json::from_str::<BitstringEntry>(&old)
            .map(|e| e.created_at)
            .unwrap_or_else(|_| Utc::now().to_rfc3339())
    } else {
        Utc::now().to_rfc3339()
    };

    let payload = BitstringEntry {
        id: id.clone(),
        name: request.name,
        bitstring: request.bitstring,
        created_at,
        source_kind: request.source_kind,
        notes: request.notes,
        linked_level_id: request.linked_level_id,
    };

    let json = serde_json::to_string_pretty(&payload).map_err(|e| e.to_string())?;
    fs::write(entry_path(&id)?, json).map_err(|e| e.to_string())?;
    Ok(payload)
}

#[tauri::command]
pub fn delete_bitstring(request: DeleteBitstringRequest) -> Result<(), String> {
    let path = entry_path(&request.id)?;
    if path.exists() {
        fs::remove_file(path).map_err(|e| e.to_string())?;
    }
    Ok(())
}
