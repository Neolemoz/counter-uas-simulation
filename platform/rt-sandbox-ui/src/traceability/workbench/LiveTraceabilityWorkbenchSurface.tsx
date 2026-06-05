import type { RtIntelligenceAdvisoryTransportV1 } from "@/intelligence/intelligenceAdvisory";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import { TraceabilityWorkbench } from "./TraceabilityWorkbench";
import { labelFromToken } from "./formatters";
import type { LiveMirrorFreshness } from "./liveTraceabilityFreshness";
import { getLiveTraceabilityAssemblyInput } from "./liveTraceabilitySelectors";
import { SelectedTrackTraceabilityWorkbench } from "./SelectedTrackTraceabilityWorkbench";
import {
  TRACEABILITY_ENTITY_POSE_MIRROR_AUTHORITY,
  TRACEABILITY_INTELLIGENCE_ADVISORY_AUTHORITY,
  TRACEABILITY_LIVE_ADAPTER_AUTHORITY,
  TRACEABILITY_LIVE_GOVERNANCE_LINES,
} from "./traceabilityProvenance";
import { assembleTraceabilityWorkbenchModel } from "./traceabilitySelectors";
import type { TraceabilityAssemblyInput } from "./traceabilitySelectors";

function ProvenanceChip({ label, value }: { label: string; value: string }) {
  return (
    <div className="rounded border border-slate-800 bg-slate-950/70 px-2 py-1">
      <dt className="text-[10px] uppercase text-slate-500">{label}</dt>
      <dd className="mt-0.5 font-mono text-slate-100">{value}</dd>
    </div>
  );
}

function liveFreshnessDetail(freshnessAlignment: string): string {
  switch (freshnessAlignment) {
    case "fresh":
      return "Entity mirror fresh; advisory fresh.";
    case "entity_stale":
      return "Entity mirror stale; advisory is not marked stale.";
    case "advisory_stale":
      return "Entity mirror fresh; advisory is marked stale.";
    case "both_stale":
      return "Entity mirror stale; advisory is marked stale.";
    default:
      return "Live freshness unknown.";
  }
}

function LiveTraceabilityProvenance({
  assemblyInput,
}: {
  assemblyInput: TraceabilityAssemblyInput;
}) {
  const model = assembleTraceabilityWorkbenchModel(assemblyInput);
  const advisoryMissing = assemblyInput.advisory === null;

  return (
    <div className="space-y-2">
      <section
        className="rounded border border-cyan-900/70 bg-cyan-950/20 p-3 text-xs"
        data-testid="live-traceability-provenance"
      >
        <div className="flex flex-wrap items-center justify-between gap-2">
          <h3 className="text-xs font-semibold uppercase tracking-wide text-cyan-100">
            Live entity/advisory correlation
          </h3>
          <div className="flex flex-wrap gap-1">
            <span className="rounded border border-slate-700 bg-slate-950/70 px-2 py-1 text-[10px] uppercase text-slate-300">
              Not tracker lineage
            </span>
            <span className="rounded border border-slate-700 bg-slate-950/70 px-2 py-1 text-[10px] uppercase text-slate-300">
              {labelFromToken(model.summary.freshness_alignment)}
            </span>
          </div>
        </div>
        <ul className="mt-2 flex flex-wrap gap-1 text-[10px] text-amber-100/80">
          {TRACEABILITY_LIVE_GOVERNANCE_LINES.map((line) => (
            <li key={line} className="rounded border border-slate-800 bg-slate-950/70 px-1.5 py-0.5">
              {line}
            </li>
          ))}
        </ul>
        <p className="mt-2 rounded border border-slate-800 bg-slate-950/60 px-2 py-1 text-[10px] text-slate-300" data-testid="live-traceability-freshness-detail">
          {liveFreshnessDetail(model.summary.freshness_alignment)}
        </p>
        <dl className="mt-2 grid gap-2 text-[10px] sm:grid-cols-3">
          <ProvenanceChip
            label="entity source"
            value={TRACEABILITY_ENTITY_POSE_MIRROR_AUTHORITY}
          />
          <ProvenanceChip
            label="advisory source"
            value={TRACEABILITY_INTELLIGENCE_ADVISORY_AUTHORITY}
          />
          <ProvenanceChip
            label="adapter"
            value={TRACEABILITY_LIVE_ADAPTER_AUTHORITY}
          />
        </dl>
        {advisoryMissing && (
          <p
            className="mt-2 rounded border border-amber-800/60 bg-amber-950/35 px-2 py-1 text-[10px] text-amber-100"
            data-testid="live-traceability-missing-advisory"
          >
            No live advisory available for the selected entity.
          </p>
        )}
      </section>
      <TraceabilityWorkbench model={model} />
    </div>
  );
}

export function LiveTraceabilityWorkbenchSurface({
  selectedEntityId,
  entityPoseMirror,
  intelligenceAdvisory,
  mirrorFreshness = "unknown",
  fixtureInputs,
}: {
  selectedEntityId: string | null;
  entityPoseMirror?: ChannelSnapshot<"entity_pose_mirror"> | null;
  intelligenceAdvisory?: RtIntelligenceAdvisoryTransportV1 | null;
  mirrorFreshness?: LiveMirrorFreshness | null;
  fixtureInputs?: Record<string, TraceabilityAssemblyInput>;
}) {
  const liveAssemblyInput = getLiveTraceabilityAssemblyInput({
    selectedEntityId,
    entityPoseMirror,
    intelligenceAdvisory,
    mirrorFreshness,
  });

  if (liveAssemblyInput) {
    return <LiveTraceabilityProvenance assemblyInput={liveAssemblyInput} />;
  }

  return (
    <SelectedTrackTraceabilityWorkbench
      selectedTrackId={selectedEntityId}
      inputs={fixtureInputs}
    />
  );
}
