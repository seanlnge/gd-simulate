use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
};

use uuid::Uuid;

use crate::contracts::LaunchNativeVisualizerRequest;

#[tauri::command]
pub fn launch_native_visualizer(request: LaunchNativeVisualizerRequest) -> Result<(), String> {
    let gd_root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .ok_or_else(|| "failed to resolve gd-real-sim root".to_owned())?;

    let temp_dir = std::env::temp_dir().join("gd-real-sim-visualizer");
    fs::create_dir_all(&temp_dir).map_err(|e| e.to_string())?;

    let level_path = temp_dir.join(format!("level-{}.txt", Uuid::new_v4()));
    let clicks_path = temp_dir.join(format!("clicks-{}.txt", Uuid::new_v4()));
    fs::write(&level_path, request.level_string).map_err(|e| e.to_string())?;
    fs::write(
        &clicks_path,
        request.click_bitstring.unwrap_or_else(|| "0".to_owned()),
    )
    .map_err(|e| e.to_string())?;

    if let Some(exe_path) = discover_binary(gd_root) {
        Command::new(exe_path)
            .arg("--levelstring-file")
            .arg(level_path.as_os_str())
            .arg("--clicks-file")
            .arg(clicks_path.as_os_str())
            .arg("--visualize")
            .spawn()
            .map_err(|e| e.to_string())?;
    } else {
        Command::new("cargo")
            .current_dir(gd_root)
            .arg("run")
            .arg("--bin")
            .arg("gd-real-sim")
            .arg("--")
            .arg("--levelstring-file")
            .arg(level_path.as_os_str())
            .arg("--clicks-file")
            .arg(clicks_path.as_os_str())
            .arg("--visualize")
            .spawn()
            .map_err(|e| e.to_string())?;
    }
    Ok(())
}

fn discover_binary(gd_root: &Path) -> Option<PathBuf> {
    let candidates = [
        gd_root.join("target").join("debug").join("gd-real-sim.exe"),
        gd_root.join("target").join("debug").join("gd-real-sim"),
    ];
    candidates.into_iter().find(|path| path.exists())
}
