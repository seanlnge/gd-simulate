import { useState } from "react";
import { Binary, Sparkles } from "lucide-react";
import { Badge } from "@/components/ui/badge";
import { Home } from "./routes/Home";
import { LevelView } from "./routes/LevelView";
import type { BitstringEntry, ViewerLevelSelection } from "./types/contracts";

function App() {
  const [selectedLevel, setSelectedLevel] = useState<ViewerLevelSelection | null>(null);
  const [selectedBitstring, setSelectedBitstring] = useState<BitstringEntry | null>(null);

  return (
    <main className="dark min-h-screen bg-[#040c1d] text-foreground">
      <div className="mx-auto w-full max-w-[1800px] space-y-3 p-3">
        <header className="flex items-center justify-between border border-slate-900 bg-[#071327] px-4 py-3">
          <div className="flex items-center gap-3">
            <Sparkles className="size-5 text-primary" />
            <h1 className="text-xl font-semibold tracking-tight">gd-real-sim Visualizer</h1>
          </div>
          <div className="flex items-center gap-2">
            <Badge variant="secondary" className="gap-2">
              <Binary className="size-3.5" />
              Bitstring: {selectedBitstring ? selectedBitstring.name : "none"}
            </Badge>
          </div>
        </header>
        {selectedLevel ? (
          <LevelView
            level={selectedLevel}
            attachedBitstring={selectedBitstring}
            onAttachBitstring={setSelectedBitstring}
            onBack={() => setSelectedLevel(null)}
          />
        ) : (
          <Home onOpenLevel={setSelectedLevel} />
        )}
      </div>
    </main>
  );
}

export default App;
