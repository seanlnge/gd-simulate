import { useState } from "react";
import { downloadOfficialLevel, searchOfficialLevels } from "../api/tauri";
import type { OfficialLevelSearchItem, ViewerLevelSelection } from "../types/contracts";

interface LevelSearchPanelProps {
  onOpenLevel: (level: ViewerLevelSelection) => void;
}

export function LevelSearchPanel({ onOpenLevel }: LevelSearchPanelProps) {
  const [query, setQuery] = useState("");
  const [results, setResults] = useState<OfficialLevelSearchItem[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  async function runSearch() {
    if (!query.trim()) return;
    setLoading(true);
    setError(null);
    try {
      const rows = await searchOfficialLevels(query.trim(), 0);
      setResults(rows);
    } catch (err) {
      setError(String(err));
    } finally {
      setLoading(false);
    }
  }

  async function openOfficialLevel(levelId: string) {
    setLoading(true);
    setError(null);
    try {
      const full = await downloadOfficialLevel(levelId);
      onOpenLevel({
        source: "official",
        id: full.level_id,
        name: full.name,
        creatorName: full.creator_name,
        levelString: full.level_string,
      });
    } catch (err) {
      setError(String(err));
    } finally {
      setLoading(false);
    }
  }

  return (
    <section className="panel">
      <h3>Official Search</h3>
      <div className="row">
        <input
          value={query}
          placeholder="Search official/online levels"
          onChange={(event) => setQuery(event.currentTarget.value)}
          onKeyDown={(event) => {
            if (event.key === "Enter") runSearch();
          }}
        />
        <button type="button" onClick={runSearch} disabled={loading}>
          {loading ? "Searching..." : "Search"}
        </button>
      </div>
      {error ? <p className="error">{error}</p> : null}
      <div className="list">
        {results.map((row) => (
          <button
            className="list-item"
            type="button"
            key={row.level_id}
            onClick={() => openOfficialLevel(row.level_id)}
          >
            <strong>{row.name}</strong> by {row.creator_name} - {row.downloads} dl
          </button>
        ))}
      </div>
    </section>
  );
}
