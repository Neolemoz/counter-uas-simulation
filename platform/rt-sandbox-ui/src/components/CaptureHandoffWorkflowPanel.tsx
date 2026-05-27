import { useState } from "react";
import { PanelShell } from "@/components/GovernanceChrome";
import type { CaptureHandoffRow } from "@/bridge/types";
import {
  handoffChipsForRow,
  highestWorkflowPhase,
  phaseLabel,
  phaseTone,
} from "@/handoff/deriveHandoffPhase";
import { advisoryCountsSummary, deriveAdvisoryForRow } from "@/handoff/advisoryAggregate";
import { AdvisoryStateBadge } from "@/handoff/AdvisoryStateBadge";
import { HandoffAdvisoryMirrorStrip } from "@/handoff/HandoffAdvisoryMirrorStrip";
import { SaWorkflowAdvisoryPanel } from "@/handoff/SaWorkflowAdvisoryPanel";
import {
  HANDOFF_AUTHORITY_STATIC,
  handoffAuthorityDetail,
} from "@/handoff/handoffAuthorityCopy";
import { StatusBadge } from "@/workstation/StatusBadge";
import {
  captureReadinessFromLifecycle,
  CAPTURE_PIPELINE_STEPS,
  HANDOFF_DOC_CHIPS,
} from "@/workflow/captureHandoffCognition";

function shortCaptureId(id: string): string {
  return id.length > 12 ? `${id.slice(0, 8)}…` : id;
}

function MultiSessionHandoffOverview({
  slots,
  handoffBySession,
  sessionLifecycleById,
}: {
  slots: { sessionId: string; label: string }[];
  handoffBySession: Map<string, CaptureHandoffRow[]>;
  sessionLifecycleById?: Map<string, string>;
}) {
  if (slots.length === 0) return null;

  return (
    <div className="mb-4 rounded border border-slate-700 bg-slate-950/50 p-3">
      <h3 className="mb-2 text-xs font-semibold uppercase tracking-wide text-slate-400">
        Multi-session handoff overview
      </h3>
      <ul className="space-y-2">
        {slots.map(({ sessionId, label }) => {
          const rows = handoffBySession.get(sessionId) ?? [];
          const phase = highestWorkflowPhase(rows);
          const lifecycle = sessionLifecycleById?.get(sessionId);
          const advisorySummary = advisoryCountsSummary(rows, lifecycle);
          return (
            <li
              key={sessionId}
              className="flex flex-wrap items-center gap-2 rounded border border-slate-700/80 bg-slate-900/40 px-2 py-1.5 text-xs"
            >
              <span className="font-mono text-slate-300">{label}</span>
              <span className="text-slate-500">captures: {rows.length}</span>
              <StatusBadge label={phaseLabel(phase)} tone={phaseTone(phase)} />
              {advisorySummary && (
                <span className="text-slate-500">advisory: {advisorySummary}</span>
              )}
            </li>
          );
        })}
      </ul>
    </div>
  );
}

function SessionCaptureTable({
  rows,
  selectedCaptureId,
  onSelectCapture,
  sessionLifecycleState,
}: {
  rows: CaptureHandoffRow[];
  selectedCaptureId: string | null;
  onSelectCapture: (id: string | null) => void;
  sessionLifecycleState?: string;
}) {
  if (rows.length === 0) {
    return (
      <p className="mb-4 text-xs text-slate-500">
        No staged captures for this session_id. After capture_session, rows appear via
        read-only bridge mirror.
      </p>
    );
  }

  return (
    <div className="mb-4 overflow-x-auto">
      <h3 className="mb-2 text-xs font-semibold uppercase tracking-wide text-slate-400">
        Session captures (read-only mirror)
      </h3>
      <table className="w-full min-w-[32rem] border-collapse text-left text-xs">
        <thead>
          <tr className="border-b border-slate-700 text-slate-500">
            <th className="py-1 pr-2">ID</th>
            <th className="py-1 pr-2">Norm</th>
            <th className="py-1 pr-2">Approval</th>
            <th className="py-1 pr-2">Review</th>
            <th className="py-1 pr-2">Phase</th>
            <th className="py-1 pr-2">Advisory</th>
            <th className="py-1">Valid</th>
          </tr>
        </thead>
        <tbody>
          {rows.map((row) => {
            const selected = row.capture_candidate_id === selectedCaptureId;
            const advisory = deriveAdvisoryForRow(row, sessionLifecycleState);
            return (
              <tr
                key={row.capture_candidate_id}
                className={`cursor-pointer border-b border-slate-800/80 ${
                  selected ? "bg-amber-950/30" : "hover:bg-slate-900/60"
                }`}
                onClick={() =>
                  onSelectCapture(
                    selected ? null : row.capture_candidate_id,
                  )
                }
              >
                <td className="py-1.5 pr-2 font-mono text-slate-300">
                  {shortCaptureId(row.capture_candidate_id)}
                </td>
                <td className="py-1.5 pr-2">{row.normalization_status}</td>
                <td className="py-1.5 pr-2">{row.approval_status}</td>
                <td className="py-1.5 pr-2">{row.handoff_decision ?? "—"}</td>
                <td className="py-1.5 pr-2">
                  <StatusBadge
                    label={phaseLabel(row.workflow_phase)}
                    tone={phaseTone(row.workflow_phase)}
                  />
                </td>
                <td className="py-1.5 pr-2">
                  {advisory.terminal ? (
                    <StatusBadge label="committed" tone="ok" title={advisory.advisory_state_label} />
                  ) : advisory.advisory_state ? (
                    <AdvisoryStateBadge
                      state={advisory.advisory_state}
                      blocked={advisory.blocked}
                      label={advisory.advisory_state_label}
                    />
                  ) : (
                    <span className="text-slate-500">—</span>
                  )}
                </td>
                <td className="py-1.5">{row.validation_ok ? "ok" : "fail"}</td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}

function HandoffAuthorityStrip({ row }: { row: CaptureHandoffRow | null }) {
  return (
    <div className="mb-4 rounded border border-amber-900/40 bg-amber-950/20 p-3">
      <h3 className="mb-2 text-xs font-semibold uppercase tracking-wide text-amber-200/90">
        Handoff cognition (authority boundaries)
      </h3>
      <ul className="mb-2 list-inside list-disc text-xs text-slate-400">
        {HANDOFF_AUTHORITY_STATIC.map((line) => (
          <li key={line}>{line}</li>
        ))}
      </ul>
      <p className="text-xs text-slate-500">{handoffAuthorityDetail(row)}</p>
      {row && (
        <div className="mt-2 flex flex-wrap gap-1">
          {handoffChipsForRow(row).map((chip) => (
            <span
              key={chip.label}
              className="rounded border border-slate-700 bg-slate-900 px-2 py-0.5 text-[10px] text-slate-400"
            >
              {chip.label}
            </span>
          ))}
        </div>
      )}
    </div>
  );
}

export function CaptureHandoffWorkflowPanel({
  connected,
  sessionState,
  sessionId,
  handoffBySession,
  workspaceSessionIds,
}: {
  connected: boolean;
  sessionState: string;
  sessionId: string | null;
  handoffBySession: Map<string, CaptureHandoffRow[]>;
  workspaceSessionIds: readonly string[];
}) {
  const readiness = captureReadinessFromLifecycle(sessionState, connected);
  const activeRows = sessionId ? (handoffBySession.get(sessionId) ?? []) : [];
  const [selectedCaptureId, setSelectedCaptureId] = useState<string | null>(null);
  const selectedRow =
    activeRows.find((r) => r.capture_candidate_id === selectedCaptureId) ?? null;

  const overviewSlots = workspaceSessionIds.map((sid) => ({
    sessionId: sid,
    label: sid.slice(0, 8),
  }));

  const highlightPhase = selectedRow?.workflow_phase ?? highestWorkflowPhase(activeRows);

  return (
    <PanelShell title="Capture & handoff pipeline (read-only mirror)">
      <p className="mb-3 text-xs text-amber-200/90">
        Staging visibility via read-only bridge mirror — maintainer CLIs perform writes.
        Not SA replay authority.
      </p>

      <MultiSessionHandoffOverview
        slots={overviewSlots}
        handoffBySession={handoffBySession}
        sessionLifecycleById={
          sessionId ? new Map([[sessionId, sessionState]]) : undefined
        }
      />

      <div className="mb-4 rounded border border-slate-700 bg-slate-950/50 p-3">
        <div className="mb-1 flex flex-wrap items-center gap-2">
          <span className="text-sm font-semibold text-slate-300">Session capture readiness</span>
          <StatusBadge label={readiness.label} tone={readiness.tone} />
        </div>
        <p className="text-xs text-slate-400">{readiness.detail}</p>
        {sessionId && (
          <p className="mt-2 font-mono text-[10px] text-slate-500">
            session_id (correlation): {sessionId}
          </p>
        )}
      </div>

      {sessionId && (
        <SessionCaptureTable
          rows={activeRows}
          selectedCaptureId={selectedCaptureId}
          onSelectCapture={setSelectedCaptureId}
          sessionLifecycleState={sessionState}
        />
      )}

      <HandoffAuthorityStrip row={selectedRow ?? activeRows[0] ?? null} />

      <HandoffAdvisoryMirrorStrip
        status={
          selectedRow
            ? deriveAdvisoryForRow(selectedRow, sessionState)
            : activeRows[0]
              ? deriveAdvisoryForRow(activeRows[0], sessionState)
              : null
        }
        compact
      />

      <SaWorkflowAdvisoryPanel
        status={
          selectedRow
            ? deriveAdvisoryForRow(selectedRow, sessionState)
            : activeRows[0]
              ? deriveAdvisoryForRow(activeRows[0], sessionState)
              : null
        }
        selectedCaptureId={
          selectedRow?.capture_candidate_id ?? activeRows[0]?.capture_candidate_id
        }
      />

      <div className="mb-4">
        <h3 className="mb-2 text-xs font-semibold uppercase tracking-wide text-slate-400">
          Export audit vocabulary
        </h3>
        <div className="flex flex-wrap gap-1">
          {HANDOFF_DOC_CHIPS.map((chip) => (
            <span
              key={chip}
              className="rounded border border-slate-700 bg-slate-900 px-2 py-0.5 font-mono text-[10px] text-slate-500"
              title="Documentation — events written by maintainer CLIs"
            >
              {chip}
            </span>
          ))}
        </div>
      </div>

      <h3 className="mb-2 text-xs font-semibold uppercase tracking-wide text-slate-400">
        Maintainer pipeline
        {highlightPhase !== "none" && (
          <span className="ml-2 font-normal normal-case text-slate-500">
            (live phase hint: {phaseLabel(highlightPhase)})
          </span>
        )}
      </h3>
      <ol className="space-y-2">
        {CAPTURE_PIPELINE_STEPS.map((step) => (
          <li
            key={step.id}
            className="rounded border border-slate-700/80 bg-slate-950/30 px-3 py-2 text-xs"
          >
            <div className="flex flex-wrap items-baseline gap-2">
              <span className="font-mono text-slate-500">{step.phase}.</span>
              <span className="font-semibold text-slate-300">{step.title}</span>
            </div>
            <p className="mt-1 font-mono text-[11px] text-emerald-400/90">{step.cli}</p>
            <p className="mt-1 text-slate-500">{step.note}</p>
          </li>
        ))}
      </ol>
    </PanelShell>
  );
}
