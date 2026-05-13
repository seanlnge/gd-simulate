export interface RenderableObject {
  object_id: number;
  x: number;
  y: number;
  rotation: number;
  scale_x: number;
  scale_y: number;
  kind: string;
}

export interface ParsedLevelResponse {
  object_count: number;
  objects: RenderableObject[];
}

export interface PlayerState {
  x: number;
  y: number;
  vx: number;
  vy: number;
  mode: string;
  gravity_sign: number;
  mini: boolean;
  player_speed: number;
  speed_multiplier: number;
  gravity: number;
  y_start: number;
  vehicle_size: number;
  on_ground: boolean;
}

export interface TraceFrame {
  tick: number;
  time: number;
  pressed: boolean;
  state: PlayerState;
  partner?: PlayerState;
}

export type SimulationOutcome =
  | { outcome: "completed"; tick: number; time: number; state: PlayerState }
  | {
      outcome: "died";
      tick: number;
      time: number;
      state: PlayerState;
      object_id?: number;
      reason: string;
      which_player: number;
    }
  | { outcome: "timeout"; tick: number; time: number; state: PlayerState };

export interface SimulationRun {
  outcome: SimulationOutcome;
  trace: TraceFrame[];
}

export interface LocalLevelEntry {
  name: string;
  raw_payload: string;
  level_string: string;
}

export interface OfficialLevelSearchItem {
  level_id: string;
  name: string;
  description: string;
  creator_name: string;
  creator_account_id?: string | null;
  downloads: number;
  likes: number;
  difficulty: number;
  length: number;
  object_count?: number | null;
  song_id?: string | null;
  custom_song_id?: string | null;
}

export interface OfficialLevelDownload {
  level_id: string;
  name: string;
  creator_name: string;
  level_string: string;
  description: string;
  length?: number | null;
  object_count: number;
  song_id?: string | null;
  custom_song_id?: string | null;
}

export interface BitstringEntry {
  id: string;
  name: string;
  bitstring: string;
  created_at: string;
  source_kind: string;
  notes?: string | null;
  linked_level_id?: string | null;
}

export interface LiveAttemptEntry {
  id: string;
  created_at_ms: number;
  outcome: string;
  percent: number;
  processed_clicks: number;
  bitstring: string;
  tick: number;
}

export interface ViewerLevelSelection {
  source: "local" | "official";
  id: string;
  name: string;
  creatorName?: string;
  levelString?: string;
  description?: string;
  length?: number | null;
  objectCountHint?: number | null;
  songId?: string | null;
  customSongId?: string | null;
  downloads?: number;
  likes?: number;
}
