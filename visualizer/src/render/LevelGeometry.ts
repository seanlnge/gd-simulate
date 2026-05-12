import type { RenderableObject } from "../types/contracts";

export function objectColor(kind: string): string {
  switch (kind) {
    case "hazard":
      return "#d64a4a";
    case "slope":
      return "#d08d35";
    case "orb":
    case "pad":
      return "#7a62d6";
    case "modeportal":
    case "speedportal":
    case "gravityportal":
    case "sizeportal":
      return "#35a6d0";
    case "solid":
      return "#4db164";
    default:
      return "#72757e";
  }
}

export function objectSize(obj: RenderableObject): number {
  const base = obj.kind === "hazard" ? 24 : 30;
  return base * Math.max(Math.abs(obj.scale_x), Math.abs(obj.scale_y), 0.3);
}
