import { useEffect, useState } from "react";
import type { CaptureHandoffRow } from "@/bridge/types";
import type { SessionSlot } from "@/hooks/useRtSessionWorkspace";
import { highestWorkflowPhase, phaseLabel } from "@/handoff/deriveHandoffPhase";
import {
  backgroundFidelityLabel,
  extractFidelityContext,
} from "@/fidelity/fidelityCognition";
import {
  deriveBackgroundSessionRow,
  staleReasonLabel,
} from "@/workstation/backgroundSessionRowCognition";
import { sortSlotsBySessionOrder } from "@/workstation/sortSessionSlots";
import { StatusBadge } from "@/workstation/StatusBadge";
import {
  sessionAccentBgClass,
  shortSessionId,
} from "@/workstation/sessionVisualIdentity";

export function BackgroundDiagnostics({
  slots,
  handoffBySession,
  orderedSessionIds,
  editingSessionId,
  terrainLayersOn = false,
  pollPaused = false,
  onOpenChange,
  labelFor,
}: {
  slots: SessionSlot[];
  handoffBySession: Map<string, CaptureHandoffRow[]>;
  orderedSessionIds: readonly string[];
  editingSessionId: string | null;
  terrainLayersOn?: boolean;
  pollPaused?: boolean;
  onOpenChange?: (open: boolean) => void;
  labelFor: (sessionId: string) => string;
}) {
  const [nowMs, setNowMs] = useState(() => Date.now());
  const [open, setOpen] = useState(false);

  useEffect(() => {
    const id = window.setInterval(() => setNowMs(Date.now()), 1000);
    return () => window.clearInterval(id);
  }, []);

  if (slots.length === 0) return null;

  const sortedSlots = sortSlotsBySessionOrder(slots, orderedSessionIds);

  return (
    <details
      className="rounded-lg border border-slate-700 bg-slate-900/50 p-3"
      open={open}
      onToggle={(e) => {
        const next = e.currentTarget.open;
        setOpen(next);
        onOpenChange?.(next);
      }}
    >
      <summary className="cursor-pointer text-sm font-medium text-slate-300">
        Background session diagnostics ({slots.length})
        {pollPaused && (
          <span className="ml-2 font-normal text-slate-500">· background poll paused</span>
        )}
        {pollPaused && <StatusBadge label="poll paused" tone="warn" />}
      </summary>
      {terrainLayersOn && (
        <p className="mt-2 text-[10px] text-slate-500">
          Fictional terrain active on active globe — background sessions use flat registry z
        </p>
      )}
      <ul className="mt-3 space-y-2">
        {sortedSlots.map((slot) => {
          const row = deriveBackgroundSessionRow(slot, editingSessionId, nowMs);
          const entityCount = slot.snapshots.world_summary?.payload?.entity_count;
          const handoffRows = handoffBySession.get(slot.sessionId) ?? [];
          const handoffPhase = highestWorkflowPhase(handoffRows);
          const healthPayload = slot.snapshots.session_health?.payload;
          const fidelityLabel = backgroundFidelityLabel(
            extractFidelityContext(healthPayload as Record<string, unknown> | undefined),
          );
          const displayLabel = labelFor(slot.sessionId);
          const idSuffix = shortSessionId(slot.sessionId);
          return (
            <li
              key={slot.sessionId}
              className="flex flex-wrap items-center gap-x-3 gap-y-1 rounded border border-slate-700/80 bg-slate-950/50 px-3 py-2 text-xs text-slate-400"
              title={slot.sessionId}
            >
              <span
                className={`h-2 w-2 shrink-0 rounded-full ${sessionAccentBgClass(slot.sessionId, orderedSessionIds)}`}
                aria-hidden
              />
              <span className="font-mono text-slate-300">{displayLabel}</span>
              <span className="font-mono text-[10px] text-slate-600">({idSuffix})</span>
              <StatusBadge label={`lifecycle: ${row.lifecycleLabel}`} tone="ok" />
              {row.isEditingLock && (
                <StatusBadge label="editing lock" tone="warn" />
              )}
              <span>entities: {String(entityCount ?? "—")}</span>
              <span>last pull: {row.pullAgeLabel}</span>
              {slot.pulling && <StatusBadge label="pulling" tone="ok" />}
              <span>
                captures: {handoffRows.length}
                {handoffRows.length > 0
                  ? ` · handoff: ${phaseLabel(handoffPhase)}`
                  : ""}
              </span>
              <span>fidelity: {fidelityLabel}</span>
              {row.healthBadges.map((b) => (
                <StatusBadge key={b.label} label={b.label} tone={b.tone} />
              ))}
              {row.staleReasons.map((reason) => (
                <StatusBadge
                  key={reason}
                  label={staleReasonLabel(reason)}
                  tone="warn"
                />
              ))}
              {slot.lastError != null && (
                <span className="text-amber-400">pull fault: {slot.lastError}</span>
              )}
            </li>
          );
        })}
      </ul>
    </details>
  );
}
