import { useEffect, useState, type ReactNode } from "react";
import { ArrowLeft, Binary, Disc3, Gauge, Play, Settings, Skull, Tv } from "lucide-react";
import {
  decodeClicksBinBlob,
  downloadOfficialLevel,
  launchNativeVisualizer,
  listBitstrings,
  upsertBitstring,
} from "@/api/tauri";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Checkbox } from "@/components/ui/checkbox";
import { ScrollArea } from "@/components/ui/scroll-area";
import type { BitstringEntry, ViewerLevelSelection } from "../types/contracts";

interface LevelViewProps {
  level: ViewerLevelSelection;
  attachedBitstring: BitstringEntry | null;
  onAttachBitstring: (entry: BitstringEntry | null) => void;
  onBack: () => void;
}

export function LevelView({ level, attachedBitstring, onAttachBitstring, onBack }: LevelViewProps) {
  const [resolvedLevelString, setResolvedLevelString] = useState<string | null>(level.levelString ?? null);
  const [previewLoading, setPreviewLoading] = useState(false);
  const [previewError, setPreviewError] = useState<string | null>(null);
  const [bitstrings, setBitstrings] = useState<BitstringEntry[]>([]);
  const [launchingNative, setLaunchingNative] = useState<"replay" | "play" | null>(null);
  const [launchError, setLaunchError] = useState<string | null>(null);
  const [settingsOpen, setSettingsOpen] = useState(false);
  const [showAllHitboxes, setShowAllHitboxes] = useState(true);
  const [showTrace, setShowTrace] = useState(true);
  const [showPlayer, setShowPlayer] = useState(true);
  const [showBottomPanels, setShowBottomPanels] = useState(true);

  useEffect(() => {
    void listBitstrings().then(setBitstrings).catch(() => setBitstrings([]));
  }, []);

  async function importBitstringBin(file: File) {
    try {
      const bytes = new Uint8Array(await file.arrayBuffer());
      const binary = Array.from(bytes, (b) => String.fromCharCode(b)).join("");
      const b64 = window.btoa(binary);
      const bitstring = await decodeClicksBinBlob(b64);
      const saved = await upsertBitstring({
        name: file.name.replace(/\.[^.]+$/, ""),
        bitstring,
        source_kind: "sp240bin_drop",
        linked_level_id: level.id,
      });
      const refreshed = await listBitstrings();
      setBitstrings(refreshed);
      onAttachBitstring(saved);
      setLaunchError(null);
    } catch (error) {
      setLaunchError(`Failed importing ${file.name}: ${String(error)}`);
    }
  }

  useEffect(() => {
    setSettingsOpen(false);
    setResolvedLevelString(level.levelString ?? null);
    setPreviewError(null);
    setLaunchError(null);
    if (level.source !== "official" || level.levelString) {
      setPreviewLoading(false);
      return;
    }

    let cancelled = false;
    setPreviewLoading(true);
    void downloadOfficialLevel(level.id)
      .then((full) => {
        if (cancelled) return;
        setResolvedLevelString(full.level_string);
      })
      .catch((error) => {
        if (cancelled) return;
        setPreviewError(String(error));
      })
      .finally(() => {
        if (!cancelled) setPreviewLoading(false);
      });

    return () => {
      cancelled = true;
    };
  }, [level.id, level.levelString, level.source]);

  const launchVisualizer = async (mode: "replay" | "play") => {
    if (!resolvedLevelString) {
      setLaunchError("Level payload has not loaded yet.");
      return;
    }
    setLaunchingNative(mode);
    setLaunchError(null);
    try {
      await launchNativeVisualizer(
        resolvedLevelString,
        mode === "replay" ? (attachedBitstring?.bitstring ?? null) : null,
        mode,
      );
    } catch (error) {
      setLaunchError(String(error));
    } finally {
      setLaunchingNative(null);
    }
  };

  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key !== "Escape") return;
      event.preventDefault();
      setSettingsOpen((prev) => !prev);
    };
    window.addEventListener("keydown", onKeyDown);
    return () => window.removeEventListener("keydown", onKeyDown);
  }, []);

  const musicLabel = level.customSongId
    ? `Custom song ${level.customSongId}`
    : level.songId
      ? `Main song ${level.songId}`
      : "Unknown";
  const lengthLabel = level.length == null ? "Unknown" : String(level.length);

  return (
    <div className="relative grid h-[calc(100vh-130px)] grid-cols-[1fr_320px] gap-3">
      <div className="space-y-3">
        <Card className="rounded-none border-slate-900 bg-[#09152c]">
          <CardHeader className="pb-3">
            <div className="flex items-center justify-between">
              <div className="space-y-1">
                <div className="flex items-center gap-2">
                  <Button variant="outline" size="sm" onClick={onBack}>
                    <ArrowLeft className="size-4" />
                    Back
                  </Button>
                  <CardTitle className="text-lg text-slate-100">{level.name}</CardTitle>
                </div>
                <p className="text-sm text-slate-400">
                  Source: {level.source}
                  {level.creatorName ? ` by ${level.creatorName}` : ""}
                </p>
              </div>
              <Badge variant="secondary" className="gap-1 bg-slate-800 text-slate-200">
                <Disc3 className="size-3.5" />
                {musicLabel}
              </Badge>
            </div>
          </CardHeader>
          <CardContent className="grid grid-cols-2 gap-3 text-sm md:grid-cols-4">
            <MetaItem label="Initial Speed" value="Native visualizer" icon={<Gauge className="size-4" />} />
            <MetaItem label="Initial Mode" value="Native visualizer" icon={<Play className="size-4" />} />
            <MetaItem label="Initial Mini" value="Native visualizer" icon={<Skull className="size-4" />} />
            <MetaItem label="Length" value={lengthLabel} />
            <MetaItem label="Obj Count" value={String(level.objectCountHint ?? "Unknown")} />
            <MetaItem label="Downloads" value={String(level.downloads ?? "Unknown")} />
            <MetaItem label="Likes" value={String(level.likes ?? "Unknown")} />
            <MetaItem label="Bitstring" value={attachedBitstring?.name ?? "None"} icon={<Binary className="size-4" />} />
          </CardContent>
          {level.description ? (
            <CardContent className="pt-0">
              <p className="rounded-sm border border-slate-800 bg-[#141e38] px-3 py-2 text-sm text-slate-300">
                {level.description}
              </p>
            </CardContent>
          ) : null}
        </Card>

        <Card className="rounded-none border-slate-900 bg-[#09152c]">
          <CardContent className="flex items-center justify-between p-4">
            <div className="space-y-1 text-sm text-slate-300">
              <p>Replay opens the native visualizer with the selected bitstring. Play opens a live 240 Hz session.</p>
              <p className="text-xs text-slate-400">
                Live controls: Space / Up / left mouse = hold. Death restarts after 1 second. Esc closes native window.
              </p>
              {previewLoading ? <p className="text-xs text-amber-300">Loading full level payload...</p> : null}
              {previewError ? <p className="text-xs text-red-300">{previewError}</p> : null}
              {launchError ? <p className="text-xs text-red-300">{launchError}</p> : null}
            </div>
            <div className="flex gap-2">
              <Button size="sm" variant="outline" onClick={() => setSettingsOpen(true)}>
                <Settings className="mr-2 size-4" />
                Settings (Esc)
              </Button>
              <Button
                onClick={() => void launchVisualizer("replay")}
                disabled={previewLoading || launchingNative != null || !resolvedLevelString}
                variant="outline"
              >
                <Tv className="mr-2 size-4" />
                {launchingNative === "replay" ? "Launching..." : "Replay Bitstring"}
              </Button>
              <Button
                onClick={() => void launchVisualizer("play")}
                disabled={previewLoading || launchingNative != null || !resolvedLevelString}
              >
                <Play className="mr-2 size-4" />
                {launchingNative === "play" ? "Launching..." : "Play Level"}
              </Button>
            </div>
          </CardContent>
        </Card>
      </div>

      <Card className="h-full rounded-none border-slate-900 bg-[#0a1328]">
        <CardHeader className="pb-3">
          <CardTitle className="text-base text-slate-100">Bitstrings</CardTitle>
          <p className="text-xs text-slate-400">Choose before replaying, or press Play Level for live input.</p>
        </CardHeader>
        <CardContent className="h-[calc(100%-88px)] p-0">
          <ScrollArea className="h-full px-3 pb-3">
            <div className="space-y-2">
              <label
                className="block cursor-pointer rounded-sm border border-dashed border-slate-700 bg-[#111a33] px-3 py-2 text-xs text-slate-300"
                onDragOver={(event) => event.preventDefault()}
                onDrop={(event) => {
                  event.preventDefault();
                  const file = event.dataTransfer.files[0];
                  if (file) void importBitstringBin(file);
                }}
              >
                Drop `.bin` here or click to import bitstring
                <input
                  type="file"
                  accept=".bin"
                  className="hidden"
                  onChange={(event) => {
                    const file = event.currentTarget.files?.[0];
                    if (file) void importBitstringBin(file);
                    event.currentTarget.value = "";
                  }}
                />
              </label>
              <button
                type="button"
                className={`w-full rounded-sm border border-slate-800 px-3 py-2 text-left text-sm text-slate-200 transition ${
                  attachedBitstring == null ? "bg-[#1b294c]" : "bg-[#141e38] hover:bg-[#1b294c]"
                }`}
                onClick={() => onAttachBitstring(null)}
              >
                No bitstring
              </button>
              {bitstrings.map((entry) => (
                <button
                  key={entry.id}
                  type="button"
                  className={`w-full rounded-sm border border-slate-800 px-3 py-2 text-left text-sm text-slate-200 transition ${
                    attachedBitstring?.id === entry.id ? "bg-[#1b294c]" : "bg-[#141e38] hover:bg-[#1b294c]"
                  }`}
                  onClick={() => onAttachBitstring(entry)}
                >
                  <p className="font-medium">{entry.name}</p>
                  <p className="text-xs text-slate-400">{entry.bitstring.length} ticks</p>
                </button>
              ))}
            </div>
          </ScrollArea>
        </CardContent>
      </Card>

      {settingsOpen ? (
        <div className="absolute inset-0 flex justify-end bg-black/45">
          <div className="h-full w-[360px] border-l border-slate-800 bg-[#0a1328] p-4">
            <h3 className="mb-3 text-base font-semibold text-slate-100">Launch Settings</h3>
            <div className="space-y-3 text-sm text-slate-200">
              <ToggleControl checked={showTrace} onCheckedChange={setShowTrace} label="Trace path (native toggle)" />
              <ToggleControl checked={showAllHitboxes} onCheckedChange={setShowAllHitboxes} label="All hitboxes (native toggle)" />
              <ToggleControl checked={showBottomPanels} onCheckedChange={setShowBottomPanels} label="Bottom panels (native toggle)" />
              <ToggleControl checked={showPlayer} onCheckedChange={setShowPlayer} label="Player marker (native toggle)" />
              <p className="text-xs text-slate-400">
                Native renderer controls these internally; this panel is for quick preset notes before launch.
              </p>
            </div>
            <div className="mt-4 flex gap-2">
              <Button
                className="flex-1"
                onClick={() => void launchVisualizer("play")}
                disabled={launchingNative != null || !resolvedLevelString}
              >
                Play Level
              </Button>
              <Button variant="outline" className="flex-1" onClick={() => setSettingsOpen(false)}>
                Close
              </Button>
            </div>
          </div>
        </div>
      ) : null}
    </div>
  );
}

function ToggleControl({
  checked,
  onCheckedChange,
  label,
}: {
  checked: boolean;
  onCheckedChange: (value: boolean) => void;
  label: string;
}) {
  return (
    <label className="flex items-center gap-2">
      <Checkbox checked={checked} onCheckedChange={(value) => onCheckedChange(Boolean(value))} />
      {label}
    </label>
  );
}

function MetaItem({ label, value, icon }: { label: string; value: string; icon?: ReactNode }) {
  return (
    <div className="rounded-md border bg-background p-2">
      <p className="text-xs text-muted-foreground">{label}</p>
      <p className="mt-1 flex items-center gap-2 font-medium">
        {icon}
        {value}
      </p>
    </div>
  );
}
