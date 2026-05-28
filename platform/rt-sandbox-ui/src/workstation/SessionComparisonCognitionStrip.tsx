import {
  deriveSessionComparisonVisualRows,
  sessionComparisonSummaryLine,
} from "./sessionComparisonCognition";
import { sessionAccentBgClass } from "./sessionVisualIdentity";

export function SessionComparisonCognitionStrip({
  activeSessionId,
  orderedSessionIds,
  comparisonGhostsEnabled,
  sessionContrastEnabled,
  compact = false,
}: {
  activeSessionId?: string | null;
  orderedSessionIds: readonly string[];
  comparisonGhostsEnabled: boolean;
  sessionContrastEnabled: boolean;
  compact?: boolean;
}) {
  const rows = deriveSessionComparisonVisualRows({
    activeSessionId,
    orderedSessionIds,
    comparisonGhostsEnabled,
    sessionContrastEnabled,
  });

  if (rows.length <= 1) return null;

  return (
    <div
      className="rounded border border-slate-700 bg-slate-950/40 p-2 text-xs"
      data-testid="session-comparison-cognition"
    >
      <p className="mb-2 text-slate-300">{sessionComparisonSummaryLine(rows)}</p>
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
    </div>
  );
}
