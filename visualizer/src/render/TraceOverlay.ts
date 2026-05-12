import type { TraceFrame } from "../types/contracts";

export function drawTracePath(
  ctx: CanvasRenderingContext2D,
  trace: TraceFrame[],
  toScreen: (x: number, y: number) => [number, number],
  tick: number,
): void {
  if (trace.length === 0) {
    return;
  }
  const end = Math.max(0, Math.min(tick, trace.length - 1));
  ctx.beginPath();
  for (let i = 0; i <= end; i += 1) {
    const [sx, sy] = toScreen(trace[i].state.x, trace[i].state.y);
    if (i === 0) {
      ctx.moveTo(sx, sy);
    } else {
      ctx.lineTo(sx, sy);
    }
  }
  ctx.strokeStyle = "#f4cf4c";
  ctx.lineWidth = 2;
  ctx.stroke();
}
