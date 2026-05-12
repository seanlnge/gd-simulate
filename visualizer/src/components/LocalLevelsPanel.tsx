import { useState } from "react";
import { listLocalLevels } from "../api/tauri";
import type { LocalLevelEntry, ViewerLevelSelection } from "../types/contracts";

interface LocalLevelsPanelProps {
  onOpenLevel: (level: ViewerLevelSelection) => void;
}

export function LocalLevelsPanel({ onOpenLevel }: LocalLevelsPanelProps) {
  const [levels, setLevels] = useState<LocalLevelEntry[]>([]);
  const [pathOverride, setPathOverride] = useState("");
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  async function refreshLevels() {
    setLoading(true);
    setError(null);
    try {
      const next = await listLocalLevels(pathOverride.trim() || undefined);
      setLevels(next);
    } catch (err) {
      setError(String(err));
    } finally {
      setLoading(false);
    }
  }

  return (
    <section className="panel">
      <h3>Local Levels</h3>
      <div className="row">
        <input
          value={pathOverride}
          placeholder="Optional CCLocalLevels.dat path override"
          onChange={(event) => setPathOverride(event.currentTarget.value)}
        />
        <button type="button" onClick={refreshLevels} disabled={loading}>
          {loading ? "Loading..." : "Load local levels"}
        </button>
      </div>
      {error ? <p className="error">{error}</p> : null}
      <div className="list">
        {levels.map((level) => (
          <button
            className="list-item"
            type="button"
            key={`${level.name}-${level.raw_payload.slice(0, 10)}`}
            onClick={() =>
              onOpenLevel({
                source: "local",
                id: `local-${level.name}`,
                name: level.name,
                levelString: level.level_string,
              })
            }
          >
            {level.name}
          </button>
        ))}
      </div>
    </section>
  );
}
