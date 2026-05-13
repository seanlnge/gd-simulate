use std::{
    fs,
    path::{Path, PathBuf},
    process::{Command, Stdio},
};

use uuid::Uuid;

use crate::contracts::{LaunchNativeVisualizerRequest, NativeVisualizerMode};

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
    let mode = request.mode.unwrap_or(NativeVisualizerMode::Replay);
    fs::write(&level_path, request.level_string).map_err(|e| e.to_string())?;
    fs::write(
        &clicks_path,
        request.click_bitstring.unwrap_or_else(|| "0".to_owned()),
    )
    .map_err(|e| e.to_string())?;

    let mut sim_args = vec![
        "--levelstring-file".to_owned(),
        level_path.to_string_lossy().into_owned(),
        "--visualize".to_owned(),
    ];
    if mode == NativeVisualizerMode::Play {
        sim_args.push("--play-live".to_owned());
    } else {
        sim_args.push("--clicks-file".to_owned());
        sim_args.push(clicks_path.to_string_lossy().into_owned());
    }

    if let Some(exe_path) = discover_binary(gd_root) {
        let mut command = Command::new(exe_path);
        spawn_detached(&mut command.args(&sim_args))
            .map_err(|e| format!("failed to launch native gd-real-sim binary: {e}"))?;
    } else {
        let mut command = Command::new("cargo");
        command
            .current_dir(gd_root)
            .arg("run")
            .arg("--bin")
            .arg("gd-real-sim")
            .arg("--");
        spawn_detached(&mut command.args(&sim_args))
            .map_err(|e| format!("failed to launch gd-real-sim through cargo: {e}"))?;
    }
    Ok(())
}

fn spawn_detached(command: &mut Command) -> std::io::Result<std::process::Child> {
    command
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
}

fn discover_binary(gd_root: &Path) -> Option<PathBuf> {
    let candidates = [
        gd_root.join("target").join("debug").join("gd-real-sim.exe"),
        gd_root.join("target").join("debug").join("gd-real-sim"),
    ];
    candidates.into_iter().find(|path| path.exists())
}
