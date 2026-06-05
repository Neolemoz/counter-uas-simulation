import { formatNumber, labelFromToken } from "./formatters";
import type { TrackLineage } from "./traceabilityWorkbenchTypes";

function LineageField({ label, value }: { label: string; value: string }) {
  return (
    <div>
      <dt className="text-[10px] uppercase text-slate-500">{label}</dt>
      <dd className="mt-0.5 font-mono text-slate-100">{value}</dd>
    </div>
  );
}

export function TrackLineagePanel({ lineage }: { lineage: TrackLineage }) {
  const stale = lineage.freshness === "stale";
  const ageLabel =
    lineage.track_age_s === null ? "-" : `${formatNumber(lineage.track_age_s, 0)} s`;

  return (
    <section
      className="rounded border border-slate-800 bg-slate-950/45 p-3 text-xs"
      data-testid="track-lineage-panel"
    >
      <h3 className="mb-2 font-semibold uppercase tracking-wide text-slate-300">
        Track lineage
      </h3>
      <div className="mb-3 grid gap-1 text-center text-[10px] uppercase text-slate-400 sm:grid-cols-3">
        <div className="rounded border border-slate-800 bg-slate-950/60 px-2 py-1">Track</div>
        <div className="rounded border border-slate-800 bg-slate-950/60 px-2 py-1">Entity</div>
        <div className="rounded border border-slate-800 bg-slate-950/60 px-2 py-1">Attacker</div>
      </div>
      <dl className="grid gap-2 sm:grid-cols-2">
        <LineageField label="track_id" value={lineage.track_id} />
        <LineageField label="linked_entity_id" value={lineage.linked_entity_id ?? "-"} />
        <LineageField label="attacker_id" value={lineage.attacker_id ?? "-"} />
        <LineageField label="track_state" value={lineage.track_state} />
        <LineageField label="track_age" value={ageLabel} />
        <div data-testid={stale ? "track-lineage-stale" : undefined}>
          <LineageField label="freshness" value={labelFromToken(lineage.freshness)} />
        </div>
        <LineageField label="confidence_level" value={lineage.confidence_level} />
        <LineageField
          label="confidence_score"
          value={formatNumber(lineage.confidence_score, 2)}
        />
      </dl>
      <p className="mt-2 text-slate-300">{lineage.confidence_basis}</p>
    </section>
  );
}
