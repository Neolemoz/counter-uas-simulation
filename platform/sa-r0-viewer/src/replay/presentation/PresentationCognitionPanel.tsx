import { useClockStore } from "../clockStore";
import { useSweepStore } from "../useSweepStore";
import { computeCognitionIndicators } from "../cognition/replayStorytelling";

export function PresentationCognitionPanel() {
  const bundle = useClockStore((s) => s.bundle);
  const sweep = useSweepStore((s) => s.sweep);
  const indicators = computeCognitionIndicators(bundle, sweep);

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <h2 className="mb-1 font-semibold text-slate-200">Presentation cognition</h2>
      <p className="mb-2 text-xs text-slate-500">
        Reviewer support only — not performance metrics or reviewer scoring.
      </p>
      <ul className="flex flex-wrap gap-2">
        {Object.entries(indicators).map(([key, label]) => (
          <li
            key={key}
            className="rounded border border-slate-700 bg-slate-950/60 px-2 py-1 text-[11px] text-slate-300"
          >
            <span className="font-medium text-slate-400">{key.replace(/([A-Z])/g, " $1")}: </span>
            {label}
          </li>
        ))}
      </ul>
    </section>
  );
}
