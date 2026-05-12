import { useEffect, useMemo, useState } from "react";
import { Flame, Search } from "lucide-react";
import { listLocalLevels, parseLocalLevelsBlob, searchOfficialLevels } from "@/api/tauri";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { ScrollArea } from "@/components/ui/scroll-area";
import type { LocalLevelEntry, OfficialLevelSearchItem, ViewerLevelSelection } from "../types/contracts";

interface HomeProps {
  onOpenLevel: (level: ViewerLevelSelection) => void;
}

const POPULAR_CACHE_KEY = "gd_visualizer_popular_levels_v1";
const POPULAR_CACHE_TTL_MS = 1000 * 60 * 60 * 24 * 30;

function readPopularCache(): OfficialLevelSearchItem[] | null {
  try {
    const raw = window.localStorage.getItem(POPULAR_CACHE_KEY);
    if (!raw) return null;
    const parsed = JSON.parse(raw) as { ts: number; items: OfficialLevelSearchItem[] };
    if (!parsed || !Array.isArray(parsed.items) || typeof parsed.ts !== "number") {
      return null;
    }
    if (Date.now() - parsed.ts > POPULAR_CACHE_TTL_MS) {
      return null;
    }
    return parsed.items;
  } catch {
    return null;
  }
}

function writePopularCache(items: OfficialLevelSearchItem[]): void {
  try {
    window.localStorage.setItem(
      POPULAR_CACHE_KEY,
      JSON.stringify({
        ts: Date.now(),
        items,
      }),
    );
  } catch {
    // Ignore storage issues.
  }
}

export function Home({ onOpenLevel }: HomeProps) {
  const [localLevels, setLocalLevels] = useState<LocalLevelEntry[]>([]);
  const [localError, setLocalError] = useState<string | null>(null);
  const [search, setSearch] = useState("");
  const [onlineLevels, setOnlineLevels] = useState<OfficialLevelSearchItem[]>([]);
  const [onlineError, setOnlineError] = useState<string | null>(null);
  const [searchLoading, setSearchLoading] = useState(false);

  useEffect(() => {
    void listLocalLevels()
      .then(setLocalLevels)
      .catch((err) => setLocalError(String(err)));
  }, []);

  useEffect(() => {
    const query = search.trim();
    let cancelled = false;
    const timer = window.setTimeout(() => {
      if (query.length === 0) {
        const cached = readPopularCache();
        if (cached) {
          setOnlineLevels(cached);
          setSearchLoading(false);
          setOnlineError(null);
          return;
        }
      }

      setSearchLoading(true);
      setOnlineError(null);
      void searchOfficialLevels(query, 0)
        .then((rows) => {
          if (cancelled) return;
          setOnlineLevels(rows);
          if (query.length === 0) {
            writePopularCache(rows);
          }
        })
        .catch((err) => {
          if (cancelled) return;
          setOnlineLevels([]);
          setOnlineError(String(err));
        })
        .finally(() => {
          if (!cancelled) {
            setSearchLoading(false);
          }
        });
    }, query.length === 0 ? 0 : 300);
    return () => {
      cancelled = true;
      window.clearTimeout(timer);
    };
  }, [search]);

  async function importLocalLevelsFile(file: File) {
    try {
      const bytes = new Uint8Array(await file.arrayBuffer());
      const binary = Array.from(bytes, (b) => String.fromCharCode(b)).join("");
      const b64 = window.btoa(binary);
      const levels = await parseLocalLevelsBlob(b64);
      setLocalLevels(levels);
      setLocalError(null);
    } catch (error) {
      setLocalError(`Failed importing ${file.name}: ${String(error)}`);
    }
  }

  const title = useMemo(
    () => (search.trim().length === 0 ? "Popular Levels" : `Search: "${search.trim()}"`),
    [search],
  );

  return (
    <div className="grid h-[calc(100vh-130px)] grid-cols-[280px_1fr] gap-3">
      <Card className="h-full overflow-hidden rounded-none border-slate-900 bg-[#0a1328]">
        <CardHeader className="pb-3">
          <CardTitle className="text-base text-slate-100">Local Levels</CardTitle>
        </CardHeader>
        <CardContent className="h-[calc(100%-72px)] p-0">
          <ScrollArea className="h-full px-3 pb-3">
            <div className="space-y-2">
              <label
                className="block cursor-pointer rounded-sm border border-dashed border-slate-700 bg-[#111a33] px-3 py-2 text-xs text-slate-300"
                onDragOver={(event) => event.preventDefault()}
                onDrop={(event) => {
                  event.preventDefault();
                  const file = event.dataTransfer.files[0];
                  if (file) void importLocalLevelsFile(file);
                }}
              >
                Drop `CCLocalLevels.dat` here or click to pick file
                <input
                  type="file"
                  accept=".dat"
                  className="hidden"
                  onChange={(event) => {
                    const file = event.currentTarget.files?.[0];
                    if (file) void importLocalLevelsFile(file);
                    event.currentTarget.value = "";
                  }}
                />
              </label>
              {localError ? <p className="text-sm text-destructive">{localError}</p> : null}
              {localLevels.map((level) => (
                <button
                  key={`${level.name}-${level.raw_payload.slice(0, 16)}`}
                  type="button"
                  className="w-full rounded-sm border border-slate-800 bg-[#141e38] px-3 py-2 text-left text-sm text-slate-200 transition hover:bg-[#1b294c]"
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
          </ScrollArea>
        </CardContent>
      </Card>

      <Card className="h-full overflow-hidden rounded-none border-slate-900 bg-[#09152c]">
        <CardHeader className="space-y-3 pb-3">
          <div className="flex items-center justify-between gap-3">
            <CardTitle className="text-base text-slate-100">{title}</CardTitle>
            <Badge variant="secondary" className="gap-1.5 bg-slate-800 text-slate-200">
              <Flame className="size-3.5" />
              {searchLoading ? "Loading" : `${onlineLevels.length} results`}
            </Badge>
          </div>
          <div className="relative">
            <Search className="pointer-events-none absolute left-3 top-2.5 size-4 text-muted-foreground" />
            <Input
              value={search}
              onChange={(event) => setSearch(event.currentTarget.value)}
              placeholder="Search Geometry Dash levels..."
              className="border-slate-800 bg-[#141e38] pl-9 text-slate-100"
            />
          </div>
          {onlineError ? <p className="text-sm text-destructive">{onlineError}</p> : null}
        </CardHeader>
        <CardContent className="h-[calc(100%-132px)] p-0">
          <ScrollArea className="h-full px-4 pb-4">
            <div className="space-y-2">
              {onlineLevels.map((level) => (
                <button
                  key={level.level_id}
                  type="button"
                  className="w-full rounded-sm border border-slate-800 bg-[#141e38] px-3 py-2 text-left text-slate-100 transition hover:bg-[#1b294c]"
                  onClick={() => {
                    onOpenLevel({
                      source: "official",
                      id: level.level_id,
                      name: level.name,
                      creatorName: level.creator_name,
                      description: level.description,
                      length: level.length,
                      objectCountHint: level.object_count ?? null,
                      songId: level.song_id,
                      customSongId: level.custom_song_id,
                      downloads: level.downloads,
                      likes: level.likes,
                    });
                  }}
                >
                  <div className="flex items-center justify-between">
                    <p className="font-medium">{level.name}</p>
                    <Badge variant="outline" className="border-slate-700 text-slate-300">
                      ID {level.level_id}
                    </Badge>
                  </div>
                  <p className="text-sm text-slate-400">
                    by {level.creator_name} | {level.downloads} dl | {level.likes} likes
                  </p>
                </button>
              ))}
            </div>
          </ScrollArea>
        </CardContent>
      </Card>
    </div>
  );
}
