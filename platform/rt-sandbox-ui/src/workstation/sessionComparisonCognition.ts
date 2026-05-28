import { shortSessionId } from "./sessionVisualIdentity";

export type SessionVisualRole = "selected" | "comparison" | "background";
export type SessionDisplayMode = "full" | "muted" | "dimmed" | "tab-only";

export interface SessionComparisonVisualRow {
  sessionId: string;
  shortId: string;
  role: SessionVisualRole;
  displayMode: SessionDisplayMode;
  commandable: boolean;
  explanatoryLabel: string;
}

export function deriveSessionComparisonVisualRows({
  activeSessionId,
  orderedSessionIds,
  comparisonGhostsEnabled,
  sessionContrastEnabled,
  compareEmphasisEnabled = false,
}: {
  activeSessionId: string | null | undefined;
  orderedSessionIds: readonly string[];
  comparisonGhostsEnabled: boolean;
  sessionContrastEnabled: boolean;
  compareEmphasisEnabled?: boolean;
}): SessionComparisonVisualRow[] {
  if (!activeSessionId) return [];

  return orderedSessionIds.map((sessionId) => {
    const selected = sessionId === activeSessionId;
    const role: SessionVisualRole = selected
      ? "selected"
      : comparisonGhostsEnabled
        ? "comparison"
        : "background";
    const displayMode: SessionDisplayMode = selected
      ? "full"
      : comparisonGhostsEnabled && sessionContrastEnabled
        ? compareEmphasisEnabled
          ? "dimmed"
          : "muted"
        : compareEmphasisEnabled && sessionContrastEnabled
          ? "dimmed"
          : "tab-only";
    return {
      sessionId,
      shortId: shortSessionId(sessionId),
      role,
      displayMode,
      commandable: selected,
      explanatoryLabel: selected
        ? "selected session - command target follows existing lock"
        : role === "comparison"
          ? "comparison visual only - no cross-session command"
          : compareEmphasisEnabled
            ? "background session dimmed visually - no cross-session command"
            : "background session - tab and diagnostics only",
    };
  });
}

export function sessionComparisonSummaryLine(
  rows: readonly SessionComparisonVisualRow[],
): string {
  if (rows.length <= 1) return "Session compare: single selected session.";
  const selected = rows.find((r) => r.role === "selected");
  const comparisons = rows.filter((r) => r.role === "comparison");
  const backgrounds = rows.filter((r) => r.role === "background");
  if (comparisons.length > 0) {
    return `Session compare: selected ${selected?.shortId ?? "unknown"} vs ${comparisons.length} comparison visual${comparisons.length === 1 ? "" : "s"} - explanatory only.`;
  }
  return `Session compare: selected ${selected?.shortId ?? "unknown"}; ${backgrounds.length} background session${backgrounds.length === 1 ? "" : "s"} tab-only.`;
}

export interface SessionComparisonChromeSummary {
  selected: number;
  comparison: number;
  background: number;
  dimmed: number;
  line: string;
}

export function sessionComparisonChromeSummary(
  rows: readonly SessionComparisonVisualRow[],
): SessionComparisonChromeSummary {
  const selected = rows.filter((r) => r.role === "selected").length;
  const comparison = rows.filter((r) => r.role === "comparison").length;
  const background = rows.filter((r) => r.role === "background").length;
  const dimmed = rows.filter((r) => r.displayMode === "dimmed").length;
  return {
    selected,
    comparison,
    background,
    dimmed,
    line: `Chrome: ${selected} selected · ${comparison} comparison · ${background} background · ${dimmed} dimmed - visual only.`,
  };
}
