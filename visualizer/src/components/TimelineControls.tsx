interface TimelineControlsProps {
  currentTick: number;
  maxTick: number;
  isPlaying: boolean;
  onTogglePlay: () => void;
  onTickChange: (tick: number) => void;
}

export function TimelineControls({
  currentTick,
  maxTick,
  isPlaying,
  onTogglePlay,
  onTickChange,
}: TimelineControlsProps) {
  return (
    <div className="timeline">
      <button type="button" onClick={onTogglePlay}>
        {isPlaying ? "Pause" : "Play"}
      </button>
      <button type="button" onClick={() => onTickChange(0)}>
        First
      </button>
      <button type="button" onClick={() => onTickChange(maxTick)}>
        Last
      </button>
      <input
        type="range"
        min={0}
        max={Math.max(0, maxTick)}
        value={Math.min(currentTick, Math.max(0, maxTick))}
        onChange={(event) => onTickChange(Number(event.currentTarget.value))}
      />
      <span>
        Tick {currentTick} / {maxTick}
      </span>
    </div>
  );
}
