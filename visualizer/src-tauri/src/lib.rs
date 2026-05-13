mod commands;
mod contracts;

use commands::{
    attempts::list_live_attempts,
    bitstrings::{delete_bitstring, list_bitstrings, upsert_bitstring},
    local_levels::{list_local_levels, parse_local_levels_blob},
    native_visualizer::launch_native_visualizer,
    official_levels::{download_official_level, search_official_levels},
    sim::{decode_clicks_bin_blob, parse_level, simulate, AppState},
};
use gd_real_sim::object_data::ObjectDatabase;

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    let object_db =
        ObjectDatabase::load_embedded().expect("failed to load embedded object database");
    tauri::Builder::default()
        .plugin(tauri_plugin_opener::init())
        .manage(AppState { object_db })
        .invoke_handler(tauri::generate_handler![
            parse_level,
            simulate,
            decode_clicks_bin_blob,
            list_local_levels,
            parse_local_levels_blob,
            search_official_levels,
            download_official_level,
            launch_native_visualizer,
            list_live_attempts,
            list_bitstrings,
            upsert_bitstring,
            delete_bitstring
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
