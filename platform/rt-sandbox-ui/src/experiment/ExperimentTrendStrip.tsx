import type { PerRunAnalytics } from "./experimentSchema";

export function ExperimentTrendStrip({ perRun }: { perRun: PerRunAnalytics[] }) {
  if (perRun.length === 0) return null;

  const ordered = [...perRun].sort((a, b) => {
    const t = a.recorded_at_utc.localeCompare(b.recorded_at_utc);
    return t !== 0 ? t : a.run_id.localeCompare(b.run_id);
  });

  return (
    <div className="rounded border border-slate-800 bg-slate-950/50 p-2" data-testid="experiment-trend-strip">
      <p className="mb-2 text-[10px] uppercase tracking-wide text-slate-500">Run trend (counts only)</p>
      <ul className="flex flex-wrap gap-2">
        {ordered.map((row) => (
          <li
            key={row.run_id}
            className="min-w-[7rem] rounded border border-slate-700 px-2 py-1 text-[10px] text-slate-400"
          >
            <div className="font-medium text-slate-300">{row.label}</div>
            <div>
              entities {row.entity_count ?? "—"} · mode {row.tactical_mode ?? "—"}
            </div>
          </li>
        ))}
      </ul>
    </div>
  );
}
