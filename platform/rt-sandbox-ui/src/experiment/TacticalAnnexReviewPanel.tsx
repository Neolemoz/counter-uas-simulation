import { BANNER_ANNEX_REVIEW } from "@/governance/banners";
import { formatAuthorityChip } from "@/telemetry/cognition";
import type { TacticalCaptureAnnex } from "./tacticalAnnexSchema";
import { TacticalAnnexTimelineTable } from "./TacticalAnnexTimelineTable";

export function TacticalAnnexReviewPanel({
  annex,
  runLabel,
  captureStagingRef,
}: {
  annex: TacticalCaptureAnnex | null;
  runLabel: string;
  captureStagingRef?: string | null;
}) {
  if (!annex) {
    return (
      <p className="text-xs text-slate-500">
        No full annex loaded for <span className="text-slate-300">{runLabel}</span>. Import{" "}
        <code className="text-slate-400">tactical_annex.json</code> or an annex bundle from
        capture staging.
      </p>
    );
  }

  return (
    <section className="space-y-3" data-testid="tactical-annex-review-panel">
      <p className="text-[10px] text-amber-100/80">{BANNER_ANNEX_REVIEW}</p>
      {annex.governance_banner && (
        <p className="text-[10px] text-slate-500">{annex.governance_banner}</p>
      )}
      <dl className="grid gap-1 rounded border border-slate-800 bg-slate-950/50 p-2 text-xs text-slate-400">
        <div>
          <dt className="inline font-medium text-slate-500">run: </dt>
          <dd className="inline text-slate-300">{runLabel}</dd>
        </div>
        {captureStagingRef && (
          <div>
            <dt className="inline font-medium text-slate-500">staging: </dt>
            <dd className="inline font-mono text-slate-400">{captureStagingRef}</dd>
          </div>
        )}
        <div>
          <dt className="inline font-medium text-slate-500">final mode: </dt>
          <dd className="inline">{annex.final_tactical_mode ?? "—"}</dd>
        </div>
        <div>
          <dt className="inline font-medium text-slate-500">selected_id: </dt>
          <dd className="inline font-mono">{annex.selected_id ?? "—"}</dd>
        </div>
        <div>
          <dt className="inline font-medium text-slate-500">assigned_target: </dt>
          <dd className="inline font-mono">{annex.assigned_target ?? "—"}</dd>
        </div>
        <div>
          <dt className="inline font-medium text-slate-500">authority: </dt>
          <dd className="inline">{formatAuthorityChip(annex.authority_label)}</dd>
        </div>
        {annex.ephemeral_session_ref && (
          <div>
            <dt className="inline font-medium text-slate-500">session ref: </dt>
            <dd className="inline font-mono">{annex.ephemeral_session_ref}</dd>
          </div>
        )}
      </dl>
      <p className="text-[10px] text-slate-600">
        Timelines use UTC sandbox timestamps — not synchronized to log-line clock t.
      </p>
      <TacticalAnnexTimelineTable
        title="Mode switches"
        rows={annex.mode_switches ?? []}
        columns={[
          { key: "t_utc", label: "t_utc" },
          { key: "from_mode", label: "from" },
          { key: "to_mode", label: "to" },
          { key: "reason", label: "reason" },
        ]}
      />
      <TacticalAnnexTimelineTable
        title="Selected timeline"
        rows={annex.selected_timeline ?? []}
        columns={[
          { key: "t_utc", label: "t_utc" },
          { key: "candidate_id", label: "id" },
          { key: "role", label: "role" },
          { key: "source", label: "source" },
        ]}
      />
      <TacticalAnnexTimelineTable
        title="Assignment timeline"
        rows={annex.assignment_timeline ?? []}
        columns={[
          { key: "t_utc", label: "t_utc" },
          { key: "assigned_candidate_id", label: "assigned" },
          { key: "reason", label: "reason" },
        ]}
      />
      <TacticalAnnexTimelineTable
        title="TTI timeline"
        rows={annex.tti_timeline ?? []}
        columns={[
          { key: "t_utc", label: "t_utc" },
          { key: "candidate_id", label: "id" },
          { key: "tti_s", label: "tti_s" },
          { key: "feasible", label: "ok" },
        ]}
      />
      <TacticalAnnexTimelineTable
        title="Recommendation timeline"
        rows={annex.recommendation_timeline ?? []}
        columns={[
          { key: "t_utc", label: "t_utc" },
          { key: "event", label: "event" },
          { key: "recommendation_id", label: "rec_id" },
        ]}
      />
      <TacticalAnnexTimelineTable
        title="Pause / resume"
        rows={annex.pause_resume_transitions ?? []}
        columns={[
          { key: "t_utc", label: "t_utc" },
          { key: "action", label: "action" },
          { key: "initiator", label: "by" },
        ]}
      />
      <TacticalAnnexTimelineTable
        title="Assignment lock events"
        rows={annex.assignment_lock_events ?? []}
        columns={[
          { key: "t_utc", label: "t_utc" },
          { key: "assigned_candidate_id", label: "id" },
          { key: "duration_s", label: "dur_s" },
        ]}
      />
      <TacticalAnnexTimelineTable
        title="Target switches"
        rows={annex.target_switch_events ?? []}
        columns={[
          { key: "t_utc", label: "t_utc" },
          { key: "switch_kind", label: "kind" },
          { key: "reason", label: "reason" },
        ]}
      />
    </section>
  );
}
