import { useSweepStore } from "../useSweepStore";
import { importanceWeightEntries } from "../cognition/replayStorytelling";

export function ReplayNarrativePanel() {
  const sweep = useSweepStore((s) => s.sweep);
  const narrative = sweep?.replay_narrative_summary;
  if (!narrative) return null;

  const weights = importanceWeightEntries(narrative.importance_weights);

  return (
    <section className="rounded border border-amber-900/40 bg-amber-950/20 p-3 text-sm">
      <h2 className="mb-1 font-semibold text-amber-100">Replay narrative summary</h2>
      <p className="mb-2 text-xs text-slate-400">
        Deterministic replay reasoning — explanatory only, not tactical guidance.
      </p>
      <p className="mb-2 text-slate-200">{narrative.headline}</p>
      {weights.length > 0 && (
        <div className="mb-2 flex flex-wrap gap-1">
          {weights.map(({ key, weight }) => (
            <span
              key={key}
              className="rounded bg-slate-800 px-1.5 py-0.5 text-[10px] text-slate-400"
              title="Viewer emphasis weight — not a score"
            >
              {key}: {(weight * 100).toFixed(0)}%
            </span>
          ))}
        </div>
      )}
      <ul className="list-inside list-disc space-y-1 text-xs text-slate-300">
        {narrative.bullets.map((b, i) => (
          <li key={i}>{b}</li>
        ))}
      </ul>
    </section>
  );
}
