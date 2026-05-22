import { Color } from "cesium";

export const OVERLAY_SCENARIO_CAVEAT =
  "Explanatory topology overlay — not terrain truth or sensor coverage proof.";

const OVERLAY_STYLES: Record<
  string,
  { fill: string; fillAlpha: number; outline: string; outlineAlpha: number }
> = {
  ridge_mask: {
    fill: "#78716c",
    fillAlpha: 0.22,
    outline: "#a8a29e",
    outlineAlpha: 0.85,
  },
  los_blocked: {
    fill: "#dc2626",
    fillAlpha: 0.14,
    outline: "#f87171",
    outlineAlpha: 0.75,
  },
  degraded_visibility: {
    fill: "#d97706",
    fillAlpha: 0.12,
    outline: "#fbbf24",
    outlineAlpha: 0.65,
  },
  ingress_corridor: {
    fill: "#d97706",
    fillAlpha: 0.14,
    outline: "#fbbf24",
    outlineAlpha: 0.7,
  },
};

const DEFAULT_STYLE = {
  fill: "#64748b",
  fillAlpha: 0.12,
  outline: "#94a3b8",
  outlineAlpha: 0.5,
};

export function overlayFillColor(kind: string): Color {
  const s = OVERLAY_STYLES[kind] ?? DEFAULT_STYLE;
  return Color.fromCssColorString(s.fill).withAlpha(s.fillAlpha);
}

export function overlayOutlineColor(kind: string): Color {
  const s = OVERLAY_STYLES[kind] ?? DEFAULT_STYLE;
  return Color.fromCssColorString(s.outline).withAlpha(s.outlineAlpha);
}

export function overlayHighlightOutline(kind: string): Color {
  const s = OVERLAY_STYLES[kind] ?? DEFAULT_STYLE;
  return Color.fromCssColorString(s.outline).withAlpha(Math.min(1, s.outlineAlpha + 0.25));
}

export function isOverlayActiveAtT(
  activeRange: [number, number] | undefined,
  currentT: number,
): boolean {
  if (!activeRange) return true;
  return currentT >= activeRange[0] && currentT <= activeRange[1];
}
