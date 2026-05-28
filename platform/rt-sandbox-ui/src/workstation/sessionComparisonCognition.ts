import { shortSessionId } from "./sessionVisualIdentity";

export type SessionVisualRole = "selected" | "comparison" | "background";
export type SessionDisplayMode = "full" | "muted" | "tab-only";

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
}: {
  activeSessionId: string | null | undefined;
  orderedSessionIds: readonly string[];
  comparisonGhostsEnabled: boolean;
  sessionContrastEnabled: boolean;
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
        ? "muted"
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
