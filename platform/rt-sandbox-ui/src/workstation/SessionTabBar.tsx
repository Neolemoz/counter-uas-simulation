import { useRef, useState } from "react";
import type { CaptureHandoffRow } from "@/bridge/types";
import type { SessionSlot } from "@/hooks/useRtSessionWorkspace";
import { highestWorkflowPhase } from "@/handoff/deriveHandoffPhase";
import { sessionStateFromSnapshots } from "@/telemetry/channelIndex";
import { sortSlotsBySessionOrder } from "@/workstation/sortSessionSlots";
import {
  sessionAccentBgClass,
  sessionAccentClass,
} from "@/workstation/sessionVisualIdentity";

function reorderIds(
  orderedSessionIds: readonly string[],
  dragId: string,
  dropId: string,
): string[] {
  if (dragId === dropId) return [...orderedSessionIds];
  const next = orderedSessionIds.filter((id) => id !== dragId);
  const dropIdx = next.indexOf(dropId);
  if (dropIdx < 0) return [...orderedSessionIds];
  next.splice(dropIdx, 0, dragId);
  return next;
}

export function SessionTabBar({
  slots,
  orderedSessionIds,
  selectedSessionId,
  editingSessionId,
  atCapacity,
  busy,
  handoffBySession,
  labelFor,
  onRename,
  onSelect,
  onReorder,
  onNew,
  onClose,
}: {
  slots: SessionSlot[];
  orderedSessionIds: readonly string[];
  selectedSessionId: string | null;
  editingSessionId: string | null;
  atCapacity: boolean;
  busy: boolean;
  handoffBySession: Map<string, CaptureHandoffRow[]>;
  labelFor: (sessionId: string) => string;
  onRename: (sessionId: string) => void;
  onSelect: (sessionId: string) => void;
  onReorder: (nextOrder: string[]) => void;
  onNew: () => void;
  onClose: (sessionId: string) => void;
}) {
  const dragIdRef = useRef<string | null>(null);
  const [dragOverId, setDragOverId] = useState<string | null>(null);

  if (slots.length === 0) return null;

  const sortedSlots = sortSlotsBySessionOrder(slots, orderedSessionIds);

  return (
    <div
      className="flex flex-wrap items-center gap-2 rounded-lg border border-slate-700 bg-slate-900/60 p-2"
      role="tablist"
      aria-label="RT sandbox sessions"
    >
      {sortedSlots.map((slot) => {
        const selected = slot.sessionId === selectedSessionId;
        const editing = slot.sessionId === editingSessionId;
        const state = sessionStateFromSnapshots(slot.snapshots);
        const handoffRows = handoffBySession.get(slot.sessionId) ?? [];
        const handoffPhase = highestWorkflowPhase(handoffRows);
        const handoffBadge =
          handoffRows.length > 0
            ? handoffPhase === "ready"
              ? " ✓handoff"
              : ` ·${handoffRows.length}c`
            : "";
        const displayLabel = labelFor(slot.sessionId);
        const isDragOver = dragOverId === slot.sessionId;
        return (
          <div
            key={slot.sessionId}
            role="tab"
            aria-selected={selected}
            draggable={!busy}
            onDragStart={(e) => {
              if (busy) return;
              dragIdRef.current = slot.sessionId;
              e.dataTransfer.effectAllowed = "move";
              e.dataTransfer.setData("text/plain", slot.sessionId);
            }}
            onDragEnd={() => {
              dragIdRef.current = null;
              setDragOverId(null);
            }}
            onDragOver={(e) => {
              if (busy) return;
              e.preventDefault();
              e.dataTransfer.dropEffect = "move";
              setDragOverId(slot.sessionId);
            }}
            onDragLeave={() => {
              if (dragOverId === slot.sessionId) setDragOverId(null);
            }}
            onDrop={(e) => {
              e.preventDefault();
              const dragId = dragIdRef.current ?? e.dataTransfer.getData("text/plain");
              setDragOverId(null);
              if (!dragId || dragId === slot.sessionId) return;
              onReorder(reorderIds(orderedSessionIds, dragId, slot.sessionId));
            }}
            className={`flex items-center gap-2 rounded-md border border-l-4 px-3 py-1.5 text-sm ${sessionAccentClass(slot.sessionId, orderedSessionIds)} ${
              selected
                ? "border-amber-500/60 bg-amber-950/40 text-amber-100"
                : "border-slate-600 bg-slate-800/80 text-slate-300 hover:border-slate-500"
            } ${isDragOver ? "ring-1 ring-sky-500/60" : ""}`}
            title={`${slot.sessionId}\nDrag to reorder (RT UI only)`}
          >
            <span
              className={`h-2 w-2 shrink-0 rounded-full ${sessionAccentBgClass(slot.sessionId, orderedSessionIds)}`}
              aria-hidden
            />
            <button
              type="button"
              className="font-mono text-xs"
              onClick={() => onSelect(slot.sessionId)}
              onDoubleClick={(e) => {
                e.preventDefault();
                onRename(slot.sessionId);
              }}
              disabled={busy}
            >
              {displayLabel}
              {editing ? " 🔒" : ""}
              {selected ? " *" : ""}
              {handoffBadge}
            </button>
            <span className="text-xs uppercase text-slate-500">{state}</span>
            <button
              type="button"
              className="text-slate-500 hover:text-red-400"
              aria-label={`Disconnect session ${displayLabel}`}
              onClick={() => onClose(slot.sessionId)}
              disabled={busy}
            >
              ×
            </button>
          </div>
        );
      })}
      <button
        type="button"
        className="rounded-md border border-dashed border-slate-600 px-3 py-1.5 text-sm text-slate-400 hover:border-slate-400 hover:text-slate-200 disabled:opacity-40"
        onClick={onNew}
        disabled={busy || atCapacity}
        title={atCapacity ? "Maximum 3 concurrent sessions" : "Start new session"}
      >
        + New
      </button>
    </div>
  );
}
