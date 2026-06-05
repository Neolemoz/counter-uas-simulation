import { AdvisoryOriginPanel } from "./AdvisoryOriginPanel";
import { ThreatLineagePanel } from "./ThreatLineagePanel";
import { TrackLineagePanel } from "./TrackLineagePanel";
import { TraceabilitySummaryPanel } from "./TraceabilitySummaryPanel";
import { TRACEABILITY_WORKBENCH_GOVERNANCE } from "./traceabilityGovernance";
import type { TraceabilityWorkbenchModel } from "./traceabilityWorkbenchTypes";

export function TraceabilityWorkbench({ model }: { model: TraceabilityWorkbenchModel }) {
  const showStaleBanner =
    model.summary.linkage_status === "stale" || model.track_lineage.freshness === "stale";
  const showMismatchBanner = model.summary.linkage_status === "mismatch";

  return (
    <section
      className="space-y-3 rounded border border-slate-700/70 bg-slate-900/70 p-3"
      data-testid="traceability-workbench"
    >
      <div className="sticky top-0 z-10 space-y-2 border-b border-slate-800 bg-slate-900/95 pb-2">
        <p className="text-[10px] text-amber-100/80" data-testid="traceability-governance-banner">
          {TRACEABILITY_WORKBENCH_GOVERNANCE}
        </p>
        {showStaleBanner && (
          <p
            className="rounded border border-amber-800/60 bg-amber-950/35 px-2 py-1 text-[10px] text-amber-100"
            data-testid="traceability-stale-banner"
          >
            Lineage stale - explanation data preserved for review.
          </p>
        )}
        {showMismatchBanner && (
          <p
            className="rounded border border-rose-800/60 bg-rose-950/35 px-2 py-1 text-[10px] text-rose-100"
            data-testid="traceability-mismatch-banner"
          >
            Lineage mismatch - track and advisory correlation diverged; review as explanatory only.
          </p>
        )}
        <h2 className="text-xs font-semibold uppercase tracking-wide text-slate-200">
          Threat traceability workbench
        </h2>
      </div>
      <TraceabilitySummaryPanel summary={model.summary} />
      <TrackLineagePanel lineage={model.track_lineage} />
      <ThreatLineagePanel lineage={model.threat_lineage} />
      <AdvisoryOriginPanel origin={model.advisory_origin} />
    </section>
  );
}
