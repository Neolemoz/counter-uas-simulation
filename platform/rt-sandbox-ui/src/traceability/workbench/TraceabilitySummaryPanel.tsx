import { formatNumber, labelFromToken, linkageStatusLabel } from "./formatters";
import type { LinkageStatus, TraceabilitySummary } from "./traceabilityWorkbenchTypes";

function linkageTone(status: LinkageStatus): string {
  switch (status) {
    case "linked":
      return "border-emerald-800/60 bg-emerald-950/30 text-emerald-100";
    case "partial":
      return "border-amber-800/60 bg-amber-950/35 text-amber-100";
    case "missing":
      return "border-slate-700 bg-slate-950/60 text-slate-400";
    case "stale":
      return "border-amber-800/60 bg-amber-950/35 text-amber-100";
    case "mismatch":
      return "border-rose-800/60 bg-rose-950/35 text-rose-100";
    default:
      return "border-slate-700 bg-slate-950/60 text-slate-300";
  }
}

function SummaryField({ label, value }: { label: string; value: string }) {
  return (
    <div className="rounded border border-slate-800 bg-slate-950/70 px-2 py-1">
      <dt className="text-[10px] uppercase text-slate-500">{label}</dt>
      <dd className="mt-0.5 font-mono text-slate-100">{value}</dd>
    </div>
  );
}

export function TraceabilitySummaryPanel({ summary }: { summary: TraceabilitySummary }) {
  const rankLabel =
    summary.threat_rank === null || !Number.isFinite(summary.threat_rank)
      ? "-"
      : `#${summary.threat_rank}`;

  return (
    <section
      className="rounded border border-slate-800 bg-slate-950/45 p-3 text-xs"
      data-testid="traceability-summary-panel"
    >
      <h3 className="mb-2 font-semibold uppercase tracking-wide text-slate-300">
        Traceability summary
      </h3>
      <div
        className={`mb-3 rounded border px-2 py-1 text-[10px] uppercase ${linkageTone(summary.linkage_status)}`}
        data-testid="traceability-linkage-status"
      >
        linkage_status: {linkageStatusLabel(summary.linkage_status)}
      </div>
      <dl className="grid gap-2 sm:grid-cols-2 lg:grid-cols-4">
        <SummaryField label="track_id" value={summary.track_id} />
        <SummaryField label="attacker_id" value={summary.attacker_id ?? "-"} />
        <SummaryField label="threat_rank" value={rankLabel} />
        <SummaryField label="threat_score" value={formatNumber(summary.threat_score, 1)} />
        <SummaryField label="advisory_id" value={summary.advisory_id ?? "-"} />
        <SummaryField
          label="recommended_defender"
          value={summary.recommended_defender ?? "-"}
        />
        <SummaryField
          label="freshness_alignment"
          value={labelFromToken(summary.freshness_alignment)}
        />
      </dl>
    </section>
  );
}
