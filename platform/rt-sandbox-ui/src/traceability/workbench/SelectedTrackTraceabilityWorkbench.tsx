import { labelFromToken, linkageStatusLabel } from "./formatters";
import { TraceabilityWorkbench } from "./TraceabilityWorkbench";
import {
  TRACEABILITY_EMPTY_STATE_COPY,
  TRACEABILITY_INTEGRATION_GOVERNANCE_LINES,
  TRACEABILITY_WORKBENCH_GOVERNANCE,
} from "./traceabilityGovernance";
import type { TraceabilityAssemblyInput } from "./traceabilitySelectors";
import { getSelectedTraceabilityModel } from "./traceabilitySelectors";

function ContextChip({ label, value }: { label: string; value: string }) {
  return (
    <div className="rounded border border-slate-800 bg-slate-950/70 px-2 py-1">
      <dt className="text-[10px] uppercase text-slate-500">{label}</dt>
      <dd className="mt-0.5 font-mono text-slate-100">{value}</dd>
    </div>
  );
}

export function SelectedTrackTraceabilityWorkbench({
  selectedTrackId,
  inputs,
}: {
  selectedTrackId: string | null;
  inputs?: Record<string, TraceabilityAssemblyInput>;
}) {
  const model = getSelectedTraceabilityModel(selectedTrackId, inputs);

  if (model === null) {
    return (
      <section
        className="rounded border border-slate-800 bg-slate-950/45 p-4 text-xs text-slate-400"
        data-testid="selected-track-traceability-empty"
      >
        <p className="font-semibold uppercase tracking-wide text-slate-300">
          Threat traceability workbench
        </p>
        <p className="mt-2 text-[10px] text-amber-100/80">{TRACEABILITY_WORKBENCH_GOVERNANCE}</p>
        <ul className="mt-2 space-y-1 text-[10px] text-slate-500">
          {TRACEABILITY_INTEGRATION_GOVERNANCE_LINES.map((line) => (
            <li key={line}>{line}</li>
          ))}
        </ul>
        <p className="mt-3">{TRACEABILITY_EMPTY_STATE_COPY}</p>
      </section>
    );
  }

  const { summary } = model;

  return (
    <div className="space-y-2" data-testid="selected-track-traceability-workbench">
      <div
        className="sticky top-0 z-20 space-y-2 rounded border border-slate-800 bg-slate-900/95 p-2"
        data-testid="traceability-context-header"
      >
        <p className="text-[10px] text-amber-100/80" data-testid="traceability-integration-governance">
          {TRACEABILITY_WORKBENCH_GOVERNANCE}
        </p>
        <ul className="flex flex-wrap gap-1 text-[10px] text-slate-500">
          {TRACEABILITY_INTEGRATION_GOVERNANCE_LINES.map((line) => (
            <li
              key={line}
              className="rounded border border-slate-800 bg-slate-950/70 px-1.5 py-0.5"
            >
              {line}
            </li>
          ))}
        </ul>
        <dl className="flex flex-wrap gap-2 text-[10px] text-slate-300">
          <ContextChip label="track_id" value={summary.track_id} />
          <ContextChip label="attacker_id" value={summary.attacker_id ?? "-"} />
          <ContextChip
            label="linkage_status"
            value={linkageStatusLabel(summary.linkage_status)}
          />
          <ContextChip
            label="freshness_alignment"
            value={labelFromToken(summary.freshness_alignment)}
          />
        </dl>
      </div>
      <TraceabilityWorkbench model={model} />
    </div>
  );
}
