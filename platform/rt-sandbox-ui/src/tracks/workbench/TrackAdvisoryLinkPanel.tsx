import { formatNumber } from "./formatters";
import type { TrackAdvisoryLink } from "./trackSensorWorkbenchTypes";

function LinkField({ label, value }: { label: string; value: string }) {
  return (
    <div>
      <dt className="text-[10px] uppercase text-slate-500">{label}</dt>
      <dd className="mt-0.5 font-mono text-slate-100">{value}</dd>
    </div>
  );
}

export function TrackAdvisoryLinkPanel({
  link,
}: {
  link: TrackAdvisoryLink | null;
}) {
  return (
    <section
      className="rounded border border-slate-800 bg-slate-950/45 p-3 text-xs"
      data-testid="track-advisory-link-panel"
    >
      <h3 className="mb-2 font-semibold uppercase tracking-wide text-slate-300">
        Track advisory link
      </h3>
      <div className="mb-3 grid gap-1 text-center text-[10px] uppercase text-slate-400 sm:grid-cols-3">
        <div className="rounded border border-slate-800 bg-slate-950/60 px-2 py-1">
          Track
        </div>
        <div className="rounded border border-slate-800 bg-slate-950/60 px-2 py-1">
          Threat Evaluation
        </div>
        <div className="rounded border border-slate-800 bg-slate-950/60 px-2 py-1">
          Advisory
        </div>
      </div>
      {link === null ? (
        <p className="rounded border border-slate-800 bg-slate-950/60 px-2 py-2 text-slate-400">
          No advisory link available for this track.
        </p>
      ) : (
        <dl className="grid gap-2 sm:grid-cols-2">
          <LinkField label="track_id" value={link.track_id} />
          <LinkField label="attacker_id" value={link.attacker_id ?? "-"} />
          <LinkField
            label="threat_rank"
            value={link.threat_rank === null ? "-" : `#${link.threat_rank}`}
          />
          <LinkField
            label="threat_score"
            value={formatNumber(link.threat_score, 1)}
          />
          <LinkField
            label="recommended_defender"
            value={link.recommended_defender ?? "-"}
          />
          <LinkField
            label="freshness_alignment"
            value={link.freshness_alignment}
          />
        </dl>
      )}
      <p className="mt-2 text-[10px] text-slate-500">
        Advisory linkage is read-only explanation flow; it does not create a control path.
      </p>
    </section>
  );
}
