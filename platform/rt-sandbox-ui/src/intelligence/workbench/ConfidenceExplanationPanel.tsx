import type { RtIntelligenceAdvisoryV1 } from "../intelligenceAdvisory";

function formatScore(score: number): string {
  return Number.isFinite(score) ? score.toFixed(2) : "-";
}

function formatBasisLabel(value: string): string {
  const label = value
    .split("_")
    .filter(Boolean)
    .join(" ");
  return label.charAt(0).toUpperCase() + label.slice(1);
}

export function ConfidenceExplanationPanel({ advisory }: { advisory: RtIntelligenceAdvisoryV1 }) {
  const confidence = advisory.confidence.heuristic_confidence;

  return (
    <section
      className="rounded border border-slate-800 bg-slate-950/45 p-3 text-xs"
      data-testid="confidence-explanation-panel"
    >
      <div className="mb-2 flex flex-wrap items-center justify-between gap-2">
        <h3 className="font-semibold uppercase tracking-wide text-slate-300">
          Confidence explanation
        </h3>
        <span className="rounded border border-slate-700 bg-slate-900 px-2 py-0.5 text-slate-200">
          {confidence.level}
        </span>
      </div>
      <dl className="grid grid-cols-2 gap-2 text-slate-300">
        <div>
          <dt className="text-[10px] uppercase text-slate-500">Confidence score</dt>
          <dd className="text-slate-200">{formatScore(confidence.score)}</dd>
        </div>
        <div>
          <dt className="text-[10px] uppercase text-slate-500">Confidence level</dt>
          <dd className="capitalize text-slate-200">{confidence.level}</dd>
        </div>
      </dl>
      <div className="mt-2">
        <p className="mb-1 text-[10px] uppercase text-slate-500">Basis</p>
        {confidence.basis.length === 0 ? (
          <p className="text-slate-500">No confidence basis available.</p>
        ) : (
          <ul className="flex flex-wrap gap-1" aria-label="Confidence basis">
            {confidence.basis.map((basis) => (
              <li
                key={basis}
                className="rounded border border-slate-700 bg-slate-900 px-1.5 py-0.5 text-[10px] text-slate-300"
              >
                {formatBasisLabel(basis)}
              </li>
            ))}
          </ul>
        )}
      </div>
      <p className="mt-2 text-[10px] text-amber-100/80">
        Heuristic advisory quality confidence only. Not mission success confidence. Not kill probability.
      </p>
    </section>
  );
}
