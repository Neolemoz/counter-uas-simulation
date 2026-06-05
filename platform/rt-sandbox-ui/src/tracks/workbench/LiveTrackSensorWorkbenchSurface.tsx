import type { RtIntelligenceAdvisoryTransportV1 } from "@/intelligence/intelligenceAdvisory";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import { TrackSensorWorkbench } from "./TrackSensorWorkbench";
import { SelectedTrackSensorWorkbench } from "./SelectedTrackSensorWorkbench";
import type { LiveTrackMirrorFreshness } from "./liveTrackFreshness";
import { getSelectedLiveTrackSensorWorkbenchModel } from "./liveTrackSensorWorkbenchSelectors";
import {
  TRACK_LIVE_ADAPTER_AUTHORITY,
  TRACK_LIVE_ENTITY_POSE_MIRROR_AUTHORITY,
  TRACK_LIVE_GOVERNANCE_LINES,
  TRACK_LIVE_INTELLIGENCE_ADVISORY_AUTHORITY,
  TRACK_LIVE_LIFECYCLE_UNAVAILABLE_NOTE,
  TRACK_LIVE_SENSOR_UNAVAILABLE_NOTE,
} from "./trackLiveProvenance";
import type { TrackSensorWorkbenchModel } from "./trackSensorWorkbenchTypes";

function ProvenanceChip({ label, value }: { label: string; value: string }) {
  return (
    <div className="rounded border border-slate-800 bg-slate-950/70 px-2 py-1">
      <dt className="text-[10px] uppercase text-slate-500">{label}</dt>
      <dd className="mt-0.5 font-mono text-slate-100">{value}</dd>
    </div>
  );
}

function liveTrackFreshnessDetail(freshnessAlignment: string): string {
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
      return "Live track freshness unknown.";
  }
}

function LiveTrackProvenance({
  model,
}: {
  model: TrackSensorWorkbenchModel;
}) {
  const advisoryMissing = model.advisory_link === null;

  return (
    <div className="space-y-2">
      <section
        className="rounded border border-cyan-900/70 bg-cyan-950/20 p-3 text-xs"
        data-testid="live-track-provenance"
      >
        <div className="flex flex-wrap items-center justify-between gap-2">
          <h3 className="text-xs font-semibold uppercase tracking-wide text-cyan-100">
            Live entity mirror workbench
          </h3>
          <div className="flex flex-wrap gap-1">
            <span className="rounded border border-slate-700 bg-slate-950/70 px-2 py-1 text-[10px] uppercase text-slate-300">
              Not tracker output
            </span>
            <span className="rounded border border-slate-700 bg-slate-950/70 px-2 py-1 text-[10px] uppercase text-slate-300">
              Entity/advisory correlation only
            </span>
          </div>
        </div>
        <ul className="mt-2 flex flex-wrap gap-1 text-[10px] text-amber-100/80">
          {TRACK_LIVE_GOVERNANCE_LINES.map((line) => (
            <li
              key={line}
              className="rounded border border-slate-800 bg-slate-950/70 px-1.5 py-0.5"
            >
              {line}
            </li>
          ))}
        </ul>
        <p className="mt-2 rounded border border-slate-800 bg-slate-950/60 px-2 py-1 text-[10px] text-slate-300" data-testid="live-track-freshness-detail">
          {liveTrackFreshnessDetail(model.advisory_link?.freshness_alignment ?? model.track.staleness)}
        </p>
        <dl className="mt-2 grid gap-2 text-[10px] sm:grid-cols-3">
          <ProvenanceChip
            label="entity source"
            value={TRACK_LIVE_ENTITY_POSE_MIRROR_AUTHORITY}
          />
          <ProvenanceChip
            label="advisory source"
            value={TRACK_LIVE_INTELLIGENCE_ADVISORY_AUTHORITY}
          />
          <ProvenanceChip label="adapter" value={TRACK_LIVE_ADAPTER_AUTHORITY} />
        </dl>
        <div className="mt-2 grid gap-2 text-[10px] sm:grid-cols-3">
          <p className="rounded border border-slate-800 bg-slate-950/60 px-2 py-1 text-slate-300">
            Track confidence unavailable.
          </p>
          <p className="rounded border border-slate-800 bg-slate-950/60 px-2 py-1 text-slate-300">
            Sensor contribution unavailable. {TRACK_LIVE_SENSOR_UNAVAILABLE_NOTE}
          </p>
          <p className="rounded border border-slate-800 bg-slate-950/60 px-2 py-1 text-slate-300">
            Tracker lifecycle unavailable. {TRACK_LIVE_LIFECYCLE_UNAVAILABLE_NOTE}
          </p>
        </div>
        {advisoryMissing && (
          <p
            className="mt-2 rounded border border-amber-800/60 bg-amber-950/35 px-2 py-1 text-[10px] text-amber-100"
            data-testid="live-track-missing-advisory"
          >
            No linked live advisory available for the selected entity.
          </p>
        )}
      </section>
      <TrackSensorWorkbench model={model} />
    </div>
  );
}

export function LiveTrackSensorWorkbenchSurface({
  selectedEntityId,
  entityPoseMirror,
  intelligenceAdvisory,
  mirrorFreshness = "unknown",
  fixtureModels,
}: {
  selectedEntityId: string | null;
  entityPoseMirror?: ChannelSnapshot<"entity_pose_mirror"> | null;
  intelligenceAdvisory?: RtIntelligenceAdvisoryTransportV1 | null;
  mirrorFreshness?: LiveTrackMirrorFreshness | null;
  fixtureModels?: readonly TrackSensorWorkbenchModel[];
}) {
  const liveModel = getSelectedLiveTrackSensorWorkbenchModel({
    selectedEntityId,
    entityPoseMirror,
    intelligenceAdvisory,
    mirrorFreshness,
  });

  if (liveModel) return <LiveTrackProvenance model={liveModel} />;

  return (
    <SelectedTrackSensorWorkbench
      selectedTrackId={selectedEntityId}
      models={fixtureModels}
    />
  );
}
