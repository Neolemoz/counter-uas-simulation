import type { WorkspaceSegment } from "./types";

export type SegmentRuntimeFlags = {
  compareMode: boolean;
  presentationMode: boolean;
  filmstripMode: boolean;
  sweepMode: boolean;
  hasBundle: boolean;
};

function readUrlHints(): {
  corpusEntry: boolean;
  sweep: boolean;
  compare: boolean;
  presentation: boolean;
} {
  if (typeof window === "undefined") {
    return { corpusEntry: false, sweep: false, compare: false, presentation: false };
  }
  const p = new URLSearchParams(window.location.search);
  return {
    corpusEntry: Boolean(p.get("corpus_entry")),
    sweep: Boolean(p.get("sweep")),
    compare: Boolean(p.get("compare") || p.get("pair")),
    presentation: Boolean(p.get("presentation") || p.get("walkthrough")),
  };
}

/**
 * Derive effective workspace segment from runtime mode + URL (PLAN-SA-H1 §8.3).
 * User override applies only when not forced by compare/presentation modes.
 */
export function resolveWorkspaceSegment(
  flags: SegmentRuntimeFlags,
  userSegment: WorkspaceSegment | null,
): WorkspaceSegment {
  if (flags.compareMode) return "compare";
  if (flags.presentationMode) return "report";
  if (flags.filmstripMode) return "replay";

  const url = readUrlHints();
  if (url.compare) return "compare";
  if (url.presentation) return "report";

  if (userSegment) return userSegment;

  if (url.corpusEntry && !flags.hasBundle) return "corpus";
  if (url.corpusEntry) return "corpus";
  if (url.sweep) return "replay";

  return "replay";
}

export function segmentFromUrlOnly(): WorkspaceSegment | null {
  const url = readUrlHints();
  if (url.compare) return "compare";
  if (url.presentation) return "report";
  if (url.corpusEntry) return "corpus";
  if (url.sweep) return "replay";
  return null;
}
