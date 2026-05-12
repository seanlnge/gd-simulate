import { invoke } from "@tauri-apps/api/core";
import type {
  BitstringEntry,
  LocalLevelEntry,
  OfficialLevelDownload,
  OfficialLevelSearchItem,
  ParsedLevelResponse,
  SimulationRun,
} from "../types/contracts";

export function parseLevel(levelString: string): Promise<ParsedLevelResponse> {
  return invoke("parse_level", { request: { level_string: levelString } });
}

export function simulateLevel(
  levelString: string,
  clickBitstring?: string | null,
  maxTicks?: number,
): Promise<SimulationRun> {
  return invoke("simulate", {
    request: {
      level_string: levelString,
      click_bitstring: clickBitstring ?? null,
      max_ticks: maxTicks ?? null,
    },
  });
}

export function listLocalLevels(pathOverride?: string): Promise<LocalLevelEntry[]> {
  return invoke("list_local_levels", {
    request: { path_override: pathOverride ?? null },
  });
}

export function parseLocalLevelsBlob(bytesB64: string): Promise<LocalLevelEntry[]> {
  return invoke("parse_local_levels_blob", {
    request: { bytes_b64: bytesB64 },
  });
}

export function searchOfficialLevels(
  query: string,
  page = 0,
): Promise<OfficialLevelSearchItem[]> {
  return invoke("search_official_levels", {
    request: { query, page },
  });
}

export function downloadOfficialLevel(levelId: string): Promise<OfficialLevelDownload> {
  return invoke("download_official_level", {
    request: { level_id: levelId },
  });
}

export function listBitstrings(): Promise<BitstringEntry[]> {
  return invoke("list_bitstrings");
}

export function upsertBitstring(payload: {
  id?: string;
  name: string;
  bitstring: string;
  source_kind: string;
  notes?: string | null;
  linked_level_id?: string | null;
}): Promise<BitstringEntry> {
  return invoke("upsert_bitstring", { request: payload });
}

export function deleteBitstring(id: string): Promise<void> {
  return invoke("delete_bitstring", { request: { id } });
}

export function launchNativeVisualizer(levelString: string, clickBitstring?: string | null): Promise<void> {
  return invoke("launch_native_visualizer", {
    request: {
      level_string: levelString,
      click_bitstring: clickBitstring ?? null,
    },
  });
}

export function decodeClicksBinBlob(bytesB64: string, sourceHz?: number): Promise<string> {
  return invoke("decode_clicks_bin_blob", {
    request: {
      bytes_b64: bytesB64,
      source_hz: sourceHz ?? null,
    },
  });
}
