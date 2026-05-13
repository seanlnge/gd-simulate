use std::{
    fs,
    path::{Path, PathBuf},
};

use crate::contracts::{ListLiveAttemptsRequest, LiveAttemptEntry};

const LIVE_ATTEMPT_LIMIT: usize = 20;

#[tauri::command]
pub fn list_live_attempts(request: ListLiveAttemptsRequest) -> Result<Vec<LiveAttemptEntry>, String> {
    let path = attempt_history_path_for_level_id(&request.level_id)?;
    read_live_attempts_from_path(&path)
}

pub fn attempt_history_path_for_level_id(level_id: &str) -> Result<PathBuf, String> {
    Ok(attempts_dir()?.join(format!("{}.jsonl", sanitize_level_id(level_id))))
}

fn attempts_dir() -> Result<PathBuf, String> {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .ok_or_else(|| "could not resolve visualizer directory".to_owned())?
        .join("attempts");
    fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    Ok(dir)
}

fn read_live_attempts_from_path(path: &Path) -> Result<Vec<LiveAttemptEntry>, String> {
    if !path.exists() {
        return Ok(Vec::new());
    }

    let raw = fs::read_to_string(path).map_err(|e| e.to_string())?;
    let mut attempts = Vec::new();
    for line in raw.lines().filter(|line| !line.trim().is_empty()) {
        attempts.push(serde_json::from_str::<LiveAttemptEntry>(line).map_err(|e| e.to_string())?);
    }
    attempts.sort_by(|a, b| b.created_at_ms.cmp(&a.created_at_ms));
    attempts.truncate(LIVE_ATTEMPT_LIMIT);
    Ok(attempts)
}

fn sanitize_level_id(level_id: &str) -> String {
    let sanitized: String = level_id
        .chars()
        .map(|ch| {
            if ch.is_ascii_alphanumeric() || ch == '-' || ch == '_' {
                ch
            } else {
                '_'
            }
        })
        .collect();
    if sanitized.is_empty() {
        "unknown".to_owned()
    } else {
        sanitized
    }
}

#[cfg(test)]
mod tests {
    use super::sanitize_level_id;

    #[test]
    fn sanitize_level_id_replaces_path_separators() {
        assert_eq!(sanitize_level_id("local:My/Level\\Name"), "local_My_Level_Name");
    }
}
