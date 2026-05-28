import {
  deriveSessionComparisonVisualRows,
  sessionComparisonChromeSummary,
  sessionComparisonSummaryLine,
} from "./sessionComparisonCognition";
import { sessionAccentBgClass } from "./sessionVisualIdentity";

export function SessionComparisonCognitionStrip({
  activeSessionId,
  orderedSessionIds,
  comparisonGhostsEnabled,
  sessionContrastEnabled,
  compareEmphasisEnabled = false,
  compact = false,
}: {
  activeSessionId?: string | null;
  orderedSessionIds: readonly string[];
  comparisonGhostsEnabled: boolean;
  sessionContrastEnabled: boolean;
  compareEmphasisEnabled?: boolean;
  compact?: boolean;
}) {
  const rows = deriveSessionComparisonVisualRows({
    activeSessionId,
    orderedSessionIds,
    comparisonGhostsEnabled,
    sessionContrastEnabled,
    compareEmphasisEnabled,
  });

  if (rows.length <= 1) return null;
  const chrome = sessionComparisonChromeSummary(rows);

  return (
    <div
      className="rounded border border-slate-700 bg-slate-950/40 p-2 text-xs"
      data-testid="session-comparison-cognition"
    >
      <p className="mb-2 text-slate-300">{sessionComparisonSummaryLine(rows)}</p>
      {compact && (
        <div className="mb-2 flex flex-wrap gap-1" data-testid="session-comparison-compact-chips">
          {rows.map((row) => (
            <span
              key={row.sessionId}
              className="rounded border border-slate-700/80 bg-slate-900/60 px-1.5 py-0.5 text-[10px] text-slate-400"
              title={row.explanatoryLabel}
            >
              <span className="font-mono text-slate-300">{row.shortId}</span> {row.role}/{row.displayMode}
            </span>
          ))}
        </div>
      )}
      {!compact && (
        <ul className="flex flex-wrap gap-2">
          {rows.map((row) => (
            <li
              key={row.sessionId}
              className="flex items-center gap-2 rounded border border-slate-700/80 bg-slate-900/60 px-2 py-1 text-[10px] text-slate-400"
              title={row.explanatoryLabel}
            >
              <span
                className={`h-2 w-2 rounded-full ${sessionAccentBgClass(row.sessionId, orderedSessionIds)}`}
                aria-hidden
              />
              <span className="font-mono text-slate-300">{row.shortId}</span>
              <span>{row.role}</span>
              <span className="text-slate-500">{row.displayMode}</span>
              {!row.commandable && <span className="text-amber-300/80">no commands</span>}
            </li>
          ))}
        </ul>
      )}
      <p className="mt-2 text-[10px] text-slate-500" data-testid="session-comparison-chrome-summary">
        {chrome.line}
      </p>
    </div>
  );
}
