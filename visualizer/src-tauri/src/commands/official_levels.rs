use std::collections::HashMap;
use std::io::Read;

use base64::{engine::general_purpose::STANDARD, Engine};
use flate2::read::{GzDecoder, ZlibDecoder};
use gd_real_sim::save::decode_level_payload;
use reqwest::blocking::Client;
use reqwest::header::USER_AGENT;

use crate::contracts::{
    DownloadOfficialLevelRequest, OfficialLevelDownload, OfficialLevelSearchItem,
    SearchOfficialLevelsRequest,
};

const BASE_URL: &str = "https://www.boomlings.com/database";
const SECRET: &str = "Wmfd2893gb7";

#[tauri::command]
pub fn search_official_levels(
    request: SearchOfficialLevelsRequest,
) -> Result<Vec<OfficialLevelSearchItem>, String> {
    let page = request.page.unwrap_or(0);
    let client = api_client()?;
    let query = request.query.trim().to_owned();
    let response = fetch_levels_raw(&client, page, &query)?;
    parse_search_response(&response)
}

#[tauri::command]
pub fn download_official_level(
    request: DownloadOfficialLevelRequest,
) -> Result<OfficialLevelDownload, String> {
    let client = api_client()?;
    let response = client
        .post(format!("{BASE_URL}/downloadGJLevel22.php"))
        .form(&[
            ("secret", SECRET.to_owned()),
            ("gameVersion", "22".to_owned()),
            ("binaryVersion", "42".to_owned()),
            ("levelID", request.level_id),
        ])
        .send()
        .map_err(|e| e.to_string())?
        .error_for_status()
        .map_err(|e| e.to_string())?
        .text()
        .map_err(|e| e.to_string())?;

    parse_download_response(&response)
}

fn api_client() -> Result<Client, String> {
    let mut headers = reqwest::header::HeaderMap::new();
    headers.insert(USER_AGENT, reqwest::header::HeaderValue::from_static(""));
    Client::builder()
        .default_headers(headers)
        .build()
        .map_err(|e| e.to_string())
}

fn fetch_levels_raw(client: &Client, page: u32, query: &str) -> Result<String, String> {
    let mut form = vec![
        ("secret", SECRET.to_owned()),
        ("gameVersion", "22".to_owned()),
        ("binaryVersion", "42".to_owned()),
        ("type", "0".to_owned()),
        ("str", query.to_owned()),
        ("page", page.to_string()),
        ("total", "0".to_owned()),
    ];
    if query.is_empty() {
        // Bias empty-search requests toward curated popular/featured levels.
        form.push(("featured", "1".to_owned()));
        form.push(("star", "1".to_owned()));
    }
    client
        .post(format!("{BASE_URL}/getGJLevels21.php"))
        .form(&form)
        .send()
        .map_err(|e| e.to_string())?
        .error_for_status()
        .map_err(|e| e.to_string())?
        .text()
        .map_err(|e| e.to_string())
}

fn parse_search_response(response: &str) -> Result<Vec<OfficialLevelSearchItem>, String> {
    if response.trim() == "-1" {
        return Ok(Vec::new());
    }

    let mut split = response.split('#');
    let levels_segment = split
        .next()
        .ok_or_else(|| "invalid levels response: missing levels segment".to_owned())?;
    let creators_segment = split.next().unwrap_or_default();

    let creators_by_id = creators_segment
        .split('|')
        .filter(|row| !row.trim().is_empty())
        .filter_map(|row| {
            let parts = row.split(':').collect::<Vec<_>>();
            if parts.len() < 2 {
                return None;
            }
            let creator_id = parts[0].to_owned();
            let creator_name = parts[1].to_owned();
            let account_id = parts.get(2).map(|value| (*value).to_owned());
            Some((creator_id, (creator_name, account_id)))
        })
        .collect::<HashMap<_, _>>();

    levels_segment
        .split('|')
        .filter(|row| !row.trim().is_empty())
        .map(|row| {
            let map = parse_colon_map(row);
            let creator_id = map.get("6").cloned().unwrap_or_default();
            let (creator_name, creator_account_id) = creators_by_id
                .get(&creator_id)
                .cloned()
                .unwrap_or_else(|| ("unknown".to_owned(), None));
            Ok(OfficialLevelSearchItem {
                level_id: required(&map, "1", "level id")?,
                name: required(&map, "2", "name")?,
                description: decode_base64_or_raw(map.get("3").cloned().unwrap_or_default()),
                creator_name,
                creator_account_id,
                downloads: parse_i64_default(map.get("10"), 0),
                likes: parse_i64_default(map.get("14"), 0),
                difficulty: parse_i64_default(map.get("9"), 0),
                length: parse_i64_default(map.get("15"), 0),
                object_count: map.get("45").and_then(|v| v.parse::<usize>().ok()),
                song_id: map.get("12").cloned(),
                custom_song_id: map.get("35").cloned(),
            })
        })
        .collect()
}

fn parse_download_response(response: &str) -> Result<OfficialLevelDownload, String> {
    if response.trim() == "-1" {
        return Err("level not found".to_owned());
    }
    let mut split = response.split('#');
    let level_segment = split
        .next()
        .ok_or_else(|| "invalid download response: missing level segment".to_owned())?;
    let map = parse_colon_map(level_segment);
    let encoded_level = required(&map, "4", "level string")?;
    let decoded_level = decode_level_string_best_effort(&encoded_level)?;
    let object_count = decoded_level
        .split(';')
        .filter(|segment| !segment.trim().is_empty())
        .count()
        .saturating_sub(1);
    Ok(OfficialLevelDownload {
        level_id: required(&map, "1", "level id")?,
        name: required(&map, "2", "name")?,
        creator_name: map
            .get("5")
            .cloned()
            .unwrap_or_else(|| "unknown".to_owned()),
        level_string: decoded_level,
        description: decode_base64_or_raw(map.get("3").cloned().unwrap_or_default()),
        length: map.get("15").and_then(|v| v.parse::<i64>().ok()),
        object_count,
        song_id: map.get("12").cloned(),
        custom_song_id: map.get("35").cloned(),
    })
}

fn parse_colon_map(row: &str) -> HashMap<String, String> {
    let mut map = HashMap::new();
    let parts = row.split(':').collect::<Vec<_>>();
    for chunk in parts.chunks_exact(2) {
        map.insert(chunk[0].to_owned(), chunk[1].to_owned());
    }
    map
}

fn required(map: &HashMap<String, String>, key: &str, name: &str) -> Result<String, String> {
    map.get(key)
        .cloned()
        .ok_or_else(|| format!("response missing {name} (key {key})"))
}

fn parse_i64_default(value: Option<&String>, default: i64) -> i64 {
    value.and_then(|v| v.parse::<i64>().ok()).unwrap_or(default)
}

fn decode_base64_or_raw(input: String) -> String {
    if input.is_empty() {
        return input;
    }
    let normalized = input.replace('-', "+").replace('_', "/");
    STANDARD
        .decode(normalized)
        .ok()
        .and_then(|bytes| String::from_utf8(bytes).ok())
        .unwrap_or(input)
}

fn decode_level_string_best_effort(payload: &str) -> Result<String, String> {
    if let Ok(decoded) = decode_level_payload(payload) {
        return Ok(decoded);
    }

    let normalized = payload.replace('_', "/").replace('-', "+");
    let decoded = match STANDARD.decode(normalized.trim()) {
        Ok(bytes) => bytes,
        Err(error) => {
            if payload.contains(',') && payload.contains(';') {
                return Ok(payload.to_owned());
            }
            return Err(error.to_string());
        }
    };

    let mut text = String::new();
    if decoded.starts_with(&[0x1f, 0x8b]) {
        GzDecoder::new(decoded.as_slice())
            .read_to_string(&mut text)
            .map_err(|error| error.to_string())?;
        return Ok(text);
    }

    if ZlibDecoder::new(decoded.as_slice())
        .read_to_string(&mut text)
        .is_ok()
    {
        return Ok(text);
    }

    if payload.contains(',') && payload.contains(';') {
        return Ok(payload.to_owned());
    }

    Err("could not decode downloaded level payload".to_owned())
}

#[cfg(test)]
mod tests {
    use super::{parse_download_response, parse_search_response};

    #[test]
    fn parses_search_rows() {
        let response = "1:42:2:Demo:3:RGVtbw==:6:10:9:5:10:111:14:5:15:3|#10:Creator:999#";
        let rows = parse_search_response(response).expect("search parse");
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].level_id, "42");
        assert_eq!(rows[0].creator_name, "Creator");
    }

    #[test]
    fn parses_download_row() {
        let response = "1:42:2:Demo:3:RGVzYw==:4:kA11,0;1,2,3,4;:5:Creator:15:3:12:4:35:7#hash#";
        let level = parse_download_response(response).expect("download parse");
        assert_eq!(level.level_id, "42");
        assert_eq!(level.level_string, "kA11,0;1,2,3,4;");
        assert_eq!(level.description, "Desc");
    }
}
