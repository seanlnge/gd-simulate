import { useEffect, useMemo, useRef, useState } from "react";
import type { RenderableObject, TraceFrame } from "../types/contracts";
import { objectColor, objectSize } from "./LevelGeometry";
import { drawTracePath } from "./TraceOverlay";

interface CanvasViewportProps {
  objects: RenderableObject[];
  trace: TraceFrame[];
  currentTick: number;
  showObjects: boolean;
  showTrace: boolean;
  showPlayer: boolean;
}

const WORLD_SCALE = 0.4;

export function CanvasViewport({
  objects,
  trace,
  currentTick,
  showObjects,
  showTrace,
  showPlayer,
}: CanvasViewportProps) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const [zoom, setZoom] = useState(1);
  const [offset, setOffset] = useState({ x: 80, y: 420 });
  const [dragging, setDragging] = useState(false);
  const [dragAnchor, setDragAnchor] = useState({ x: 0, y: 0 });

  const clampedTick = useMemo(() => {
    if (trace.length === 0) return 0;
    return Math.max(0, Math.min(currentTick, trace.length - 1));
  }, [currentTick, trace.length]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "#14161c";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    const toScreen = (x: number, y: number): [number, number] => [
      x * WORLD_SCALE * zoom + offset.x,
      canvas.height - (y * WORLD_SCALE * zoom + offset.y),
    ];

    if (showObjects) {
      for (const obj of objects) {
        const [sx, sy] = toScreen(obj.x, obj.y);
        const size = objectSize(obj) * WORLD_SCALE * zoom;
        ctx.fillStyle = objectColor(obj.kind);
        ctx.fillRect(sx - size / 2, sy - size / 2, size, size);
      }
    }

    if (showTrace && trace.length > 0) {
      drawTracePath(ctx, trace, toScreen, clampedTick);
    }

    if (showPlayer && trace.length > 0) {
      const frame = trace[clampedTick];
      const [px, py] = toScreen(frame.state.x, frame.state.y);
      const radius = frame.state.mini ? 4 : 6;
      ctx.beginPath();
      ctx.arc(px, py, radius, 0, Math.PI * 2);
      ctx.fillStyle = "#59d7ff";
      ctx.fill();
      ctx.strokeStyle = "#d8f6ff";
      ctx.lineWidth = 2;
      ctx.stroke();
    }
  }, [objects, trace, clampedTick, offset, showObjects, showPlayer, showTrace, zoom]);

  return (
    <canvas
      ref={canvasRef}
      width={1200}
      height={640}
      className="w-full rounded-md border bg-zinc-950"
      onWheel={(event) => {
        event.preventDefault();
        const dir = event.deltaY > 0 ? -1 : 1;
        setZoom((prev) => Math.min(8, Math.max(0.15, prev + dir * 0.1)));
      }}
      onMouseDown={(event) => {
        if (event.button !== 1 && event.button !== 2) return;
        setDragging(true);
        setDragAnchor({ x: event.clientX, y: event.clientY });
      }}
      onMouseMove={(event) => {
        if (!dragging) return;
        const dx = event.clientX - dragAnchor.x;
        const dy = event.clientY - dragAnchor.y;
        setDragAnchor({ x: event.clientX, y: event.clientY });
        setOffset((prev) => ({ x: prev.x + dx, y: prev.y - dy }));
      }}
      onMouseUp={() => setDragging(false)}
      onMouseLeave={() => setDragging(false)}
      onContextMenu={(event) => event.preventDefault()}
    />
  );
}
