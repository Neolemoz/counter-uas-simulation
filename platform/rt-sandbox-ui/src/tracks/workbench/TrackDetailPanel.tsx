import {
  formatHeading,
  formatMeters,
  formatMps,
  formatNumber,
  formatTimestamp,
  labelFromToken,
} from "./formatters";
import type { TrackDetail } from "./trackSensorWorkbenchTypes";

function Field({ label, value }: { label: string; value: string }) {
  return (
    <div className="rounded border border-slate-800 bg-slate-950/55 px-2 py-1.5">
      <dt className="text-[10px] uppercase text-slate-500">{label}</dt>
      <dd className="mt-0.5 break-words font-mono text-slate-100">{value}</dd>
    </div>
  );
}

export function TrackDetailPanel({ track }: { track: TrackDetail }) {
  return (
    <section
      className="rounded border border-slate-800 bg-slate-950/45 p-3 text-xs"
      data-testid="track-detail-panel"
    >
      <div className="mb-2 flex flex-wrap items-center justify-between gap-2">
        <h3 className="font-semibold uppercase tracking-wide text-slate-300">
          Track detail
        </h3>
        <span className="rounded border border-slate-700 bg-slate-900 px-2 py-0.5 text-[10px] uppercase text-slate-200">
          {labelFromToken(track.staleness)}
        </span>
      </div>
      <dl className="grid gap-2 sm:grid-cols-2">
        <Field label="track_id" value={track.track_id} />
        <Field label="linked_entity_id" value={track.linked_entity_id ?? "-"} />
        <Field label="track_state" value={track.track_state} />
        <Field
          label="pose"
          value={`x ${formatMeters(track.pose.x)} / y ${formatMeters(track.pose.y)} / z ${formatMeters(track.pose.z)}`}
        />
        <Field
          label="velocity"
          value={`vx ${formatMps(track.velocity.vx)} / vy ${formatMps(track.velocity.vy)} / vz ${formatMps(track.velocity.vz)}`}
        />
        <Field label="heading" value={formatHeading(track.heading_deg)} />
        <Field label="speed" value={formatMps(track.speed_mps)} />
        <Field label="source_authority" value={track.source_authority} />
        <Field label="last_update" value={formatTimestamp(track.last_update_utc)} />
        <Field label="track_age" value={`${formatNumber(track.track_age_s, 0)} s`} />
        <Field label="staleness" value={track.staleness} />
      </dl>
    </section>
  );
}
