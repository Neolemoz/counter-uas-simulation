import { useEffect, useState } from "react";
import type { SessionSlot } from "@/hooks/useRtSessionWorkspace";
import {
  backgroundPollPausedLabel,
  backgroundPullChipLabel,
  backgroundPullFaultLabel,
  backgroundPullingLabel,
  backgroundStaleChipLabels,
} from "@/workstation/backgroundDiagnosticChips";
import { deriveBackgroundSessionRow } from "@/workstation/backgroundSessionRowCognition";
import { sortSlotsBySessionOrder } from "@/workstation/sortSessionSlots";
import { StatusBadge } from "@/workstation/StatusBadge";
import {
  sessionAccentBgClass,
  shortSessionId,
} from "@/workstation/sessionVisualIdentity";

export function BackgroundDiagnosticsCompact({
  slots,
  orderedSessionIds,
  pollPaused = false,
  onExpandDetails,
  labelFor,
}: {
  slots: SessionSlot[];
  orderedSessionIds: readonly string[];
  pollPaused?: boolean;
  onExpandDetails?: () => void;
  labelFor: (sessionId: string) => string;
}) {
  const [nowMs, setNowMs] = useState(() => Date.now());

  useEffect(() => {
    const id = window.setInterval(() => setNowMs(Date.now()), 1000);
    return () => window.clearInterval(id);
  }, []);

  if (slots.length === 0) return null;

  const sortedSlots = sortSlotsBySessionOrder(slots, orderedSessionIds);
  const allStale = sortedSlots.every((slot) => {
    const row = deriveBackgroundSessionRow(slot, null, nowMs);
    return row.staleReasons.length > 0 || slot.lastError != null;
  });

  const handleActivate = () => {
    onExpandDetails?.();
  };

  return (
    <div
      role="status"
      data-testid="background-diagnostics-compact"
      className={`rounded-lg border px-3 py-2 ${
        allStale
          ? "border-amber-700/50 bg-amber-950/20"
          : "border-slate-700 bg-slate-900/50"
      }`}
    >
      <div className="mb-2 flex flex-wrap items-center gap-2 text-[10px] text-slate-500">
        <span className="font-medium uppercase tracking-wide text-slate-400">
          Background sessions
        </span>
        {pollPaused && <StatusBadge label={backgroundPollPausedLabel()} tone="warn" />}
        <button
          type="button"
          className="text-sky-400/90 underline-offset-2 hover:underline"
          onClick={handleActivate}
          onKeyDown={(e) => {
            if (e.key === "Enter" || e.key === " ") {
              e.preventDefault();
              handleActivate();
            }
          }}
        >
          Expand diagnostics
        </button>
      </div>
      <ul className="flex flex-wrap gap-2">
        {sortedSlots.map((slot) => {
          const row = deriveBackgroundSessionRow(slot, null, nowMs);
          const displayLabel = labelFor(slot.sessionId);
          const idSuffix = shortSessionId(slot.sessionId);
          const staleLabels = backgroundStaleChipLabels(row.staleReasons);
          return (
            <li
              key={slot.sessionId}
              className="flex flex-wrap items-center gap-x-2 gap-y-1 rounded border border-slate-700/80 bg-slate-950/60 px-2 py-1 text-[10px] text-slate-400"
              title={slot.sessionId}
            >
              <span
                className={`h-2 w-2 shrink-0 rounded-full ${sessionAccentBgClass(slot.sessionId, orderedSessionIds)}`}
                aria-hidden
              />
              <span className="font-mono text-slate-300">{displayLabel}</span>
              <span className="font-mono text-slate-600">({idSuffix})</span>
              <span>{backgroundPullChipLabel(row.pullAgeLabel)}</span>
              {slot.pulling && <StatusBadge label={backgroundPullingLabel()} tone="ok" />}
              {staleLabels.map((label) => (
                <StatusBadge key={label} label={label} tone="warn" />
              ))}
              {slot.lastError != null && (
                <span className="text-amber-400">
                  {backgroundPullFaultLabel(slot.lastError)}
                </span>
              )}
            </li>
          );
        })}
      </ul>
    </div>
  );
}
