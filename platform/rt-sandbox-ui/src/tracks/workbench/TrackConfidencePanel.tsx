import { formatNumber, labelFromToken } from "./formatters";
import type { TrackConfidence } from "./trackSensorWorkbenchTypes";

export function TrackConfidencePanel({
  confidence,
}: {
  confidence: TrackConfidence;
}) {
  return (
    <section
      className="rounded border border-slate-800 bg-slate-950/45 p-3 text-xs"
      data-testid="track-confidence-panel"
    >
      <div className="mb-2 flex flex-wrap items-center justify-between gap-2">
        <h3 className="font-semibold uppercase tracking-wide text-slate-300">
          Track confidence
        </h3>
        <span className="rounded border border-slate-700 bg-slate-900 px-2 py-0.5 text-slate-200">
          {confidence.level}
        </span>
      </div>
      <dl className="grid grid-cols-2 gap-2 text-slate-300">
        <div>
          <dt className="text-[10px] uppercase text-slate-500">Confidence score</dt>
          <dd className="text-slate-200">{formatNumber(confidence.score, 2)}</dd>
        </div>
        <div>
          <dt className="text-[10px] uppercase text-slate-500">Confidence level</dt>
          <dd className="capitalize text-slate-200">{confidence.level}</dd>
        </div>
      </dl>
      <div className="mt-2">
        <p className="mb-1 text-[10px] uppercase text-slate-500">Factor chips</p>
        {confidence.factors.length === 0 ? (
          <p className="text-slate-500">No confidence factors available.</p>
        ) : (
          <ul className="flex flex-wrap gap-1" aria-label="Track confidence factors">
            {confidence.factors.map((factor) => (
              <li
                key={factor}
                className="rounded border border-slate-700 bg-slate-900 px-1.5 py-0.5 text-[10px] text-slate-300"
              >
                {labelFromToken(factor)}
              </li>
            ))}
          </ul>
        )}
      </div>
      <p className="mt-2 text-slate-300">{confidence.basis}</p>
      <p className="mt-2 text-[10px] text-amber-100/80">
        Track quality confidence only. Not mission success confidence. Not kill probability. Not engagement confidence.
      </p>
    </section>
  );
}
