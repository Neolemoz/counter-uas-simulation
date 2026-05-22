import { useSweepStore } from "../useSweepStore";

function BarRow({ label, value, max }: { label: string; value: number; max: number }) {
  const pct = max > 0 ? Math.round((value / max) * 100) : 0;
  return (
    <li className="flex items-center gap-2 text-xs">
      <span className="w-28 shrink-0 text-slate-500">{label}</span>
      <span className="w-6 text-right text-slate-400">{value}</span>
      <div className="h-2 flex-1 rounded bg-slate-800">
        <div className="h-2 rounded bg-violet-600/70" style={{ width: `${pct}%` }} />
      </div>
    </li>
  );
}

export function OutcomeDistributionPanel() {
  const sweep = useSweepStore((s) => s.sweep);
  if (!sweep?.replay_aggregation?.outcome_histogram) return null;

  const hist = sweep.replay_aggregation.outcome_histogram;
  const entries = Object.entries(hist).filter(([, vals]) => Array.isArray(vals) && vals.length);

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <h2 className="mb-1 font-semibold text-slate-200">Outcome distribution</h2>
      <p className="mb-2 text-[10px] text-slate-500">Replay-local histogram — not operational KPIs.</p>
      <ul className="space-y-2">
        {entries.map(([key, vals]) => {
          const nums = vals as number[];
          const counts = new Map<number, number>();
          for (const n of nums) counts.set(n, (counts.get(n) ?? 0) + 1);
          const max = Math.max(...counts.values(), 1);
          return (
            <li key={key}>
              <p className="mb-1 text-xs font-medium text-slate-400">{key}</p>
              <ul className="space-y-1">
                {[...counts.entries()].map(([v, c]) => (
                  <BarRow key={v} label={`t=${v}`} value={c} max={max} />
                ))}
              </ul>
            </li>
          );
        })}
      </ul>
    </section>
  );
}
