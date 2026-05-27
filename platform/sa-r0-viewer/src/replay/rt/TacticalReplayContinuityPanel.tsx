import type { ReplaySaBundle } from "../bundleSchema";
import { hasTacticalContinuity } from "../tacticalReplayContinuitySchema";
import { TacticalTimelineTable } from "./TacticalTimelineTable";

type Props = { bundle: ReplaySaBundle };

export function TacticalReplayContinuityPanel({ bundle }: Props) {
  const block = bundle.rt_tactical_replay_continuity;
  if (!hasTacticalContinuity(block)) {
    return (
      <p className="text-sm text-slate-500">
        No RT tactical continuity embedded in this replay bundle.
      </p>
    );
  }

  const annex = block!.tactical_annex;

  return (
    <section className="space-y-3 text-sm">
      <p className="text-xs leading-relaxed text-amber-100/85">{block!.governance_banner}</p>
      {annex.governance_banner && (
        <p className="text-[10px] text-slate-500">{annex.governance_banner}</p>
      )}
      <dl className="grid gap-1 rounded border border-slate-800 bg-slate-950/50 p-2 text-xs text-slate-400">
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
          <dd className="inline font-mono">{annex.authority_label ?? "—"}</dd>
        </div>
      </dl>
      <p className="text-[10px] text-slate-600">
        Timelines use UTC sandbox timestamps — not synchronized to log-line clock t.
      </p>
      <TacticalTimelineTable
        title="Mode switches"
        rows={annex.mode_switches ?? []}
        columns={[
          { key: "t_utc", label: "t_utc" },
          { key: "from_mode", label: "from" },
          { key: "to_mode", label: "to" },
          { key: "reason", label: "reason" },
        ]}
      />
      <TacticalTimelineTable
        title="Selected timeline"
        rows={annex.selected_timeline ?? []}
        columns={[
          { key: "t_utc", label: "t_utc" },
          { key: "candidate_id", label: "id" },
          { key: "role", label: "role" },
          { key: "source", label: "source" },
        ]}
      />
      <TacticalTimelineTable
        title="Assignment timeline"
        rows={annex.assignment_timeline ?? []}
        columns={[
          { key: "t_utc", label: "t_utc" },
          { key: "assigned_candidate_id", label: "assigned" },
          { key: "reason", label: "reason" },
        ]}
      />
      <TacticalTimelineTable
        title="TTI timeline"
        rows={annex.tti_timeline ?? []}
        columns={[
          { key: "t_utc", label: "t_utc" },
          { key: "candidate_id", label: "id" },
          { key: "tti_s", label: "tti_s" },
          { key: "feasible", label: "ok" },
        ]}
      />
      <TacticalTimelineTable
        title="Recommendation timeline"
        rows={annex.recommendation_timeline ?? []}
        columns={[
          { key: "t_utc", label: "t_utc" },
          { key: "event", label: "event" },
          { key: "recommendation_id", label: "rec_id" },
        ]}
      />
      <TacticalTimelineTable
        title="Pause / resume"
        rows={annex.pause_resume_transitions ?? []}
        columns={[
          { key: "t_utc", label: "t_utc" },
          { key: "action", label: "action" },
          { key: "initiator", label: "by" },
        ]}
      />
      <TacticalTimelineTable
        title="Assignment lock events"
        rows={annex.assignment_lock_events ?? []}
        columns={[
          { key: "t_utc", label: "t_utc" },
          { key: "assigned_candidate_id", label: "id" },
          { key: "duration_s", label: "dur_s" },
        ]}
      />
      <TacticalTimelineTable
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
