import { useSweepStore } from "../useSweepStore";
import { buildVariabilitySummaries } from "../cognition/replayVariability";

export function ReplayVariabilityPanel() {
  const sweep = useSweepStore((s) => s.sweep);
  if (!sweep) return null;

  const lines = buildVariabilitySummaries(sweep);

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <h2 className="mb-1 font-semibold text-slate-200">Replay variability</h2>
      <ul className="list-inside list-disc space-y-1 text-xs text-slate-300">
        {lines.map((line) => (
          <li key={line}>{line}</li>
        ))}
      </ul>
      <p className="mt-2 text-[10px] text-slate-500">
        Descriptive replay-local summaries only — not validated tactical conclusions.
      </p>
    </section>
  );
}
