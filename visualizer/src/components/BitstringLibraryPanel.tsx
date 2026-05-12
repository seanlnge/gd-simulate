import { useEffect, useMemo, useState } from "react";
import { deleteBitstring, listBitstrings, upsertBitstring } from "../api/tauri";
import type { BitstringEntry } from "../types/contracts";

interface BitstringLibraryPanelProps {
  selectedId: string | null;
  onSelect: (entry: BitstringEntry | null) => void;
}

export function BitstringLibraryPanel({ selectedId, onSelect }: BitstringLibraryPanelProps) {
  const [entries, setEntries] = useState<BitstringEntry[]>([]);
  const [name, setName] = useState("");
  const [bitstring, setBitstring] = useState("");
  const [notes, setNotes] = useState("");
  const [error, setError] = useState<string | null>(null);

  async function reload() {
    try {
      setEntries(await listBitstrings());
    } catch (err) {
      setError(String(err));
    }
  }

  useEffect(() => {
    void reload();
  }, []);

  const selected = useMemo(
    () => entries.find((entry) => entry.id === selectedId) ?? null,
    [entries, selectedId],
  );

  async function createEntry() {
    setError(null);
    try {
      const created = await upsertBitstring({
        name: name.trim() || `Bitstring ${entries.length + 1}`,
        bitstring: bitstring.trim(),
        source_kind: "manual",
        notes: notes.trim() || null,
      });
      setName("");
      setBitstring("");
      setNotes("");
      await reload();
      onSelect(created);
    } catch (err) {
      setError(String(err));
    }
  }

  async function removeEntry(id: string) {
    setError(null);
    try {
      await deleteBitstring(id);
      await reload();
      if (selectedId === id) onSelect(null);
    } catch (err) {
      setError(String(err));
    }
  }

  return (
    <section className="panel">
      <h3>Stored Click Bitstrings</h3>
      <div className="column">
        <input
          value={name}
          placeholder="Name"
          onChange={(event) => setName(event.currentTarget.value)}
        />
        <textarea
          value={bitstring}
          placeholder="Bitstring (0/1 clicks at 240Hz)"
          rows={5}
          onChange={(event) => setBitstring(event.currentTarget.value)}
        />
        <input
          value={notes}
          placeholder="Notes (optional)"
          onChange={(event) => setNotes(event.currentTarget.value)}
        />
        <button type="button" onClick={createEntry} disabled={!bitstring.trim()}>
          Save bitstring
        </button>
      </div>
      {error ? <p className="error">{error}</p> : null}
      <div className="list">
        <button
          className={`list-item ${selectedId === null ? "active" : ""}`}
          type="button"
          onClick={() => onSelect(null)}
        >
          Render with no bitstring
        </button>
        {entries.map((entry) => (
          <div className="row split" key={entry.id}>
            <button
              className={`list-item ${entry.id === selected?.id ? "active" : ""}`}
              type="button"
              onClick={() => onSelect(entry)}
            >
              {entry.name} ({entry.bitstring.length} ticks)
            </button>
            <button type="button" onClick={() => removeEntry(entry.id)}>
              Delete
            </button>
          </div>
        ))}
      </div>
    </section>
  );
}
