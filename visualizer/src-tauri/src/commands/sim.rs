use base64::{engine::general_purpose::STANDARD, Engine};
use flate2::read::{GzDecoder, ZlibDecoder};
use gd_real_sim::{
    input::ClickTape,
    level::Level,
    object_data::ObjectDatabase,
    save::decode_level_payload,
    sim::{simulate_with_trace, SimulationConfig, SimulationRun},
};
use std::io::Read;
use tauri::State;

use crate::contracts::{
    DecodeClicksBinBlobRequest, ParseLevelRequest, ParsedLevelResponse, RenderableObject,
    SimulateRequest,
};

pub struct AppState {
    pub object_db: ObjectDatabase,
}

#[tauri::command]
pub fn parse_level(
    request: ParseLevelRequest,
    state: State<'_, AppState>,
) -> Result<ParsedLevelResponse, String> {
    let level = parse_level_with_decode(&request.level_string, &state.object_db)?;
    let objects = level
        .objects
        .iter()
        .map(|obj| RenderableObject {
            object_id: obj.object_id,
            x: obj.x,
            y: obj.y,
            rotation: obj.rotation,
            scale_x: obj.scale_x,
            scale_y: obj.scale_y,
            kind: format!("{:?}", obj.kind).to_lowercase(),
        })
        .collect::<Vec<_>>();

    Ok(ParsedLevelResponse {
        object_count: objects.len(),
        objects,
    })
}

#[tauri::command]
pub fn simulate(
    request: SimulateRequest,
    state: State<'_, AppState>,
) -> Result<SimulationRun, String> {
    let level = parse_level_with_decode(&request.level_string, &state.object_db)?;
    let bitstring = request.click_bitstring.unwrap_or_default();
    let clicks = ClickTape::from_bits(&bitstring).map_err(|e| e.to_string())?;
    let config = SimulationConfig {
        max_ticks: request
            .max_ticks
            .unwrap_or(SimulationConfig::default().max_ticks),
    };
    simulate_with_trace(&level, &clicks, config).map_err(|e| e.to_string())
}

fn parse_level_with_decode(input: &str, db: &ObjectDatabase) -> Result<Level, String> {
    match Level::parse(input, db) {
        Ok(level) => Ok(level),
        Err(primary_error) => {
            let decoded = decode_level_payload_best_effort(input)
                .map_err(|decode_error| format!("{primary_error}; decode error: {decode_error}"))?;
            Level::parse(&decoded, db).map_err(|decoded_error| {
                format!(
                    "failed to parse level. raw parse error: {primary_error}; decoded parse error: {decoded_error}"
                )
            })
        }
    }
}

fn decode_level_payload_best_effort(payload: &str) -> Result<String, String> {
    if let Ok(decoded) = decode_level_payload(payload) {
        return Ok(decoded);
    }

    let normalized = payload.replace('_', "/").replace('-', "+");
    let decoded = STANDARD
        .decode(normalized.trim())
        .map_err(|error| error.to_string())?;

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

    String::from_utf8(decoded).map_err(|error| error.to_string())
}

#[tauri::command]
pub fn decode_clicks_bin_blob(request: DecodeClicksBinBlobRequest) -> Result<String, String> {
    let bytes = STANDARD
        .decode(request.bytes_b64.trim())
        .map_err(|error| error.to_string())?;
    read_sp240bin_from_bytes(&bytes, request.source_hz)
}

fn read_sp240bin_from_bytes(bytes: &[u8], override_hz: Option<u32>) -> Result<String, String> {
    const SIM_HZ: u32 = 240;
    if bytes.len() < 21 {
        return Err("SP240BIN file too small".to_owned());
    }
    if &bytes[0..8] != b"SP240BIN" {
        return Err("missing SP240BIN magic header".to_owned());
    }
    let header_hz = u32::from_le_bytes(bytes[8..12].try_into().map_err(|_| "bad header")?);
    let header_total =
        u32::from_le_bytes(bytes[16..20].try_into().map_err(|_| "bad header")?) as usize;
    let source_hz = override_hz.unwrap_or(header_hz);
    if source_hz == 0 {
        return Err("click input Hz must be > 0".to_owned());
    }

    let payload = &bytes[21..];
    let payload_samples = payload.len() * 8;
    let mut source_bits = Vec::with_capacity(header_total);
    for i in 0..header_total {
        let bit = if i < payload_samples {
            (payload[i / 8] >> (i % 8)) & 1
        } else {
            0
        };
        source_bits.push(bit);
    }

    if source_hz == SIM_HZ {
        let mut out = String::with_capacity(source_bits.len());
        for bit in source_bits {
            out.push(if bit == 1 { '1' } else { '0' });
        }
        return Ok(out);
    }

    Ok(resample_click_bits_linear(&source_bits, source_hz, SIM_HZ))
}

fn resample_click_bits_linear(bits: &[u8], src_hz: u32, dst_hz: u32) -> String {
    if src_hz == 0 || dst_hz == 0 || bits.is_empty() {
        return String::new();
    }
    if src_hz == dst_hz {
        let mut out = String::with_capacity(bits.len());
        for &b in bits {
            out.push(if b == 1 { '1' } else { '0' });
        }
        return out;
    }
    let out_len_u128 =
        (bits.len() as u128 * dst_hz as u128 + (src_hz as u128 / 2)) / src_hz as u128;
    let out_len = out_len_u128 as usize;
    let mut out = String::with_capacity(out_len);
    for i in 0..out_len {
        let src_pos = ((i as f64 + 0.5) * src_hz as f64 / dst_hz as f64) - 0.5;
        let v = if src_pos <= 0.0 {
            bits[0] as f64
        } else if src_pos >= (bits.len() - 1) as f64 {
            bits[bits.len() - 1] as f64
        } else {
            let i0 = src_pos.floor() as usize;
            let i1 = i0 + 1;
            let frac = src_pos - i0 as f64;
            let v0 = bits[i0] as f64;
            let v1 = bits[i1] as f64;
            v0 + (v1 - v0) * frac
        };
        out.push(if v >= 0.5 { '1' } else { '0' });
    }
    out
}
