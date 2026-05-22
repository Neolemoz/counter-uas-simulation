import type { WorkspaceSegment } from "./types";

/** AUTHORING profile banner when authoring manifest mirror is loaded (PLAT-SA-A1). */
export const AUTHORING_SCENARIO_BANNER =
  "AUTHORING — fixture topology only; CLI promotes; not live configuration";

/** Governance-safe segment banners (PLAN-SA-H1 §14.1). */
export const SEGMENT_BANNERS: Record<WorkspaceSegment, string> = {
  scenario: "SCENARIO — fixture topology only; not live configuration",
  replay: "REPLAY — derived summary; not operational state",
  compare: "COMPARE — divergence review; not ranking",
  corpus: "CORPUS — index mirror; not deployment catalog",
  report: "REPORT — explanatory presentation; not certification",
};

export const SEGMENT_ACCENT: Record<WorkspaceSegment, string> = {
  scenario: "border-teal-700/50 text-teal-200",
  replay: "border-cyan-700/50 text-cyan-200",
  compare: "border-amber-700/50 text-amber-200",
  corpus: "border-slate-600 text-slate-300",
  report: "border-violet-700/50 text-violet-200",
};

export function segmentBanner(
  segment: WorkspaceSegment,
  opts?: { authoringMirror?: boolean },
): string {
  if (segment === "scenario" && opts?.authoringMirror) {
    return AUTHORING_SCENARIO_BANNER;
  }
  return SEGMENT_BANNERS[segment];
}
