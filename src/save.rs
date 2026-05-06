use std::{fs, io::Read, path::Path};

use base64::{Engine, engine::general_purpose::STANDARD};
use flate2::read::{GzDecoder, ZlibDecoder};
use quick_xml::{Reader, events::Event};

use crate::{SimError, SimResult};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LocalLevel {
    pub name: String,
    pub raw_payload: String,
    pub levelstring: String,
}

pub fn read_local_levels(path: &Path) -> SimResult<Vec<LocalLevel>> {
    let bytes = fs::read(path)?;
    parse_local_levels_xml(&decode_local_levels_dat(&bytes)?)
}

pub fn decode_local_levels_dat(bytes: &[u8]) -> SimResult<String> {
    if bytes.starts_with(br#"<?xml version="1.0"?>"#) {
        return String::from_utf8(bytes.to_vec())
            .map_err(|error| SimError::LevelParse(error.to_string()));
    }

    let mut xored = bytes.iter().map(|byte| byte ^ 11).collect::<Vec<_>>();
    // GD writes a trailing 0x00 (-> 0x0B after XOR) which corrupts the base64
    // tail; strip any non-base64 bytes before decoding.
    while xored
        .last()
        .map(|b| !(b.is_ascii_alphanumeric() || matches!(*b, b'+' | b'/' | b'=' | b'-' | b'_')))
        .unwrap_or(false)
    {
        xored.pop();
    }
    let base64_text =
        String::from_utf8(xored).map_err(|error| SimError::LevelParse(error.to_string()))?;
    let normalized = base64_text.replace('-', "+").replace('_', "/");
    let compressed = STANDARD
        .decode(normalized.trim())
        .map_err(|error| SimError::LevelParse(error.to_string()))?;

    let mut xml = String::new();
    // CCLocalLevels.dat uses gzip (header 1f 8b). Some POC tooling produces
    // raw zlib instead (header 78 9c / 78 da); fall back if gzip fails.
    if compressed.starts_with(&[0x1f, 0x8b]) {
        GzDecoder::new(compressed.as_slice()).read_to_string(&mut xml)?;
    } else {
        ZlibDecoder::new(compressed.as_slice()).read_to_string(&mut xml)?;
    }
    Ok(xml)
}

pub fn decode_level_payload(payload: &str) -> SimResult<String> {
    let normalized = payload.replace('_', "/").replace('-', "+");
    let decoded = STANDARD
        .decode(normalized.trim())
        .map_err(|error| SimError::LevelParse(error.to_string()))?;

    let mut levelstring = String::new();
    GzDecoder::new(decoded.as_slice()).read_to_string(&mut levelstring)?;
    Ok(levelstring)
}

pub fn parse_local_levels_xml(xml: &str) -> SimResult<Vec<LocalLevel>> {
    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);

    let mut levels = Vec::new();
    let mut dict_stack = Vec::<PartialLevel>::new();
    let mut pending_key: Option<String> = None;

    loop {
        match reader.read_event() {
            Ok(Event::Start(element)) if element.name().as_ref() == b"d" => {
                dict_stack.push(PartialLevel::default());
            }
            Ok(Event::End(element)) if element.name().as_ref() == b"d" => {
                if let Some(partial) = dict_stack.pop()
                    && let (Some(name), Some(raw_payload)) = (partial.name, partial.raw_payload)
                {
                    levels.push(LocalLevel {
                        name,
                        levelstring: decode_level_payload(&raw_payload)?,
                        raw_payload,
                    });
                }
            }
            Ok(Event::Start(element)) if element.name().as_ref() == b"k" => {
                pending_key = read_text_until_end(&mut reader, b"k")?;
            }
            Ok(Event::Start(element)) if matches!(element.name().as_ref(), b"s" | b"t") => {
                let tag = element.name().as_ref().to_vec();
                let value = read_text_until_end(&mut reader, &tag)?.unwrap_or_default();
                if let (Some(dict), Some(key)) = (dict_stack.last_mut(), pending_key.take()) {
                    match key.as_str() {
                        "k2" => dict.name = Some(value),
                        "k4" => dict.raw_payload = Some(value),
                        _ => {}
                    }
                }
            }
            Ok(Event::Eof) => break,
            Ok(_) => {}
            Err(error) => return Err(SimError::LevelParse(error.to_string())),
        }
    }

    Ok(levels)
}

pub fn select_local_level<'a>(
    levels: &'a [LocalLevel],
    name_or_index: Option<&str>,
) -> SimResult<&'a LocalLevel> {
    if levels.is_empty() {
        return Err(SimError::LevelParse("no local levels found".to_owned()));
    }

    let Some(selector) = name_or_index else {
        return Ok(&levels[0]);
    };

    if let Ok(index) = selector.parse::<usize>()
        && let Some(level) = levels.get(index)
    {
        return Ok(level);
    }

    levels
        .iter()
        .find(|level| level.name == selector)
        .ok_or_else(|| SimError::LevelParse(format!("level {selector:?} not found")))
}

fn read_text_until_end(reader: &mut Reader<&[u8]>, end: &[u8]) -> SimResult<Option<String>> {
    let text = match reader.read_event() {
        Ok(Event::Text(text)) => Some(
            text.decode()
                .map_err(|error| SimError::LevelParse(error.to_string()))?
                .into_owned(),
        ),
        Ok(Event::End(element)) if element.name().as_ref() == end => return Ok(None),
        Ok(_) => None,
        Err(error) => return Err(SimError::LevelParse(error.to_string())),
    };

    loop {
        match reader.read_event() {
            Ok(Event::End(element)) if element.name().as_ref() == end => return Ok(text),
            Ok(Event::Eof) => return Ok(text),
            Ok(_) => {}
            Err(error) => return Err(SimError::LevelParse(error.to_string())),
        }
    }
}

#[derive(Debug, Default)]
struct PartialLevel {
    name: Option<String>,
    raw_payload: Option<String>,
}
