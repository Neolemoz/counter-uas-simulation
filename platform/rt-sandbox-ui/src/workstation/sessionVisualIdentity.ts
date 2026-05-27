/** Per-session visual accent for multi-session chrome (PLAT-RT-V1). */

import { sessionAccentCss } from "@/cesium/visualStyle";

export const MAX_SESSION_SLOTS = 3;

export const SESSION_ACCENT_CLASSES = [
  "border-l-amber-400",
  "border-l-sky-400",
  "border-l-violet-400",
] as const;

export const SESSION_ACCENT_BG = [
  "bg-amber-400",
  "bg-sky-400",
  "bg-violet-400",
] as const;

/** Stable slot index from ordered session ids (workspace tab order). */
export function sessionSlotIndex(
  sessionId: string,
  orderedSessionIds: readonly string[],
): number {
  const idx = orderedSessionIds.indexOf(sessionId);
  return idx >= 0 ? idx : 0;
}

export function sessionAccentClass(
  sessionId: string,
  orderedSessionIds: readonly string[],
): string {
  return SESSION_ACCENT_CLASSES[
    sessionSlotIndex(sessionId, orderedSessionIds) % SESSION_ACCENT_CLASSES.length
  ];
}

export function sessionAccentBgClass(
  sessionId: string,
  orderedSessionIds: readonly string[],
): string {
  return SESSION_ACCENT_BG[
    sessionSlotIndex(sessionId, orderedSessionIds) % SESSION_ACCENT_BG.length
  ];
}

export function sessionAccentColor(
  sessionId: string,
  orderedSessionIds: readonly string[],
): string {
  return sessionAccentCss(sessionSlotIndex(sessionId, orderedSessionIds));
}

export function shortSessionId(sessionId: string): string {
  return sessionId.length > 8 ? sessionId.slice(0, 8) : sessionId;
}
