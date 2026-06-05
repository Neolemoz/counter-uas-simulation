import { SensorContributionPanel } from "./SensorContributionPanel";
import { TrackAdvisoryLinkPanel } from "./TrackAdvisoryLinkPanel";
import { TrackConfidencePanel } from "./TrackConfidencePanel";
import { TrackDetailPanel } from "./TrackDetailPanel";
import { TrackLifecyclePanel } from "./TrackLifecyclePanel";
import { formatNumber, labelFromToken } from "./formatters";
import type { TrackSensorWorkbenchModel } from "./trackSensorWorkbenchTypes";

export const TRACK_SENSOR_WORKBENCH_GOVERNANCE =
  "TRACK & SENSOR WORKBENCH - read-only explanation only; no assignment, engagement, or autonomy authority";

export function TrackSensorWorkbench({
  model,
}: {
  model: TrackSensorWorkbenchModel;
}) {
  const stale = model.track.staleness === "stale";
  const ageLabel = `${formatNumber(model.track.track_age_s, 0)} s`;

  return (
    <section
      className="space-y-3 rounded border border-slate-700/70 bg-slate-900/70 p-3"
      data-testid="track-sensor-workbench"
    >
      <div className="sticky top-0 z-10 space-y-2 border-b border-slate-800 bg-slate-900/95 pb-2">
        <p className="text-[10px] text-amber-100/80">
          {TRACK_SENSOR_WORKBENCH_GOVERNANCE}
        </p>
        {stale && (
          <p
            className="rounded border border-amber-800/60 bg-amber-950/35 px-2 py-1 text-[10px] text-amber-100"
            data-testid="track-stale-banner"
          >
            Track stale - explanation data preserved for review.
          </p>
        )}
        <div className="flex flex-wrap items-center justify-between gap-2">
          <h2 className="text-xs font-semibold uppercase tracking-wide text-slate-200">
            Track & sensor workbench
          </h2>
          <dl
            className="flex flex-wrap gap-2 text-[10px] text-slate-300"
            data-testid="track-context-header"
          >
            <div className="rounded border border-slate-800 bg-slate-950/70 px-2 py-1">
              <dt className="uppercase text-slate-500">track_id</dt>
              <dd className="font-mono text-slate-100">{model.track.track_id}</dd>
            </div>
            <div className="rounded border border-slate-800 bg-slate-950/70 px-2 py-1">
              <dt className="uppercase text-slate-500">linked_entity_id</dt>
              <dd className="font-mono text-slate-100">
                {model.track.linked_entity_id ?? "-"}
              </dd>
            </div>
            <div className="rounded border border-slate-800 bg-slate-950/70 px-2 py-1">
              <dt className="uppercase text-slate-500">track_state</dt>
              <dd className="text-slate-100">{model.track.track_state}</dd>
            </div>
            <div className="rounded border border-slate-800 bg-slate-950/70 px-2 py-1">
              <dt className="uppercase text-slate-500">track_age</dt>
              <dd className="font-mono text-slate-100">{ageLabel}</dd>
            </div>
            <div className="rounded border border-slate-800 bg-slate-950/70 px-2 py-1">
              <dt className="uppercase text-slate-500">freshness</dt>
              <dd className={stale ? "font-semibold text-amber-100" : "text-slate-100"}>
                {labelFromToken(model.track.staleness)}
              </dd>
            </div>
          </dl>
        </div>
      </div>
      <TrackDetailPanel track={model.track} />
      <SensorContributionPanel rows={model.sensor_contributions} />
      <TrackLifecyclePanel events={model.lifecycle_events} />
      <TrackConfidencePanel confidence={model.confidence} />
      <TrackAdvisoryLinkPanel link={model.advisory_link} />
    </section>
  );
}
