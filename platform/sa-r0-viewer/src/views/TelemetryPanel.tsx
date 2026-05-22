import type { ReplaySaBundle } from "@/replay/bundleSchema";
import { useClockStore } from "@/replay/clockStore";

type Props = {
  bundle?: ReplaySaBundle | null;
  currentT?: number;
};

export function TelemetryPanel({ bundle: bundleProp, currentT: tProp }: Props = {}) {
  const bundle = bundleProp ?? useClockStore((s) => s.bundle);
  const currentT = tProp ?? useClockStore((s) => s.currentT);
  if (!bundle) return null;

  const series = bundle.panels?.telemetry_series ?? [];
  const atT = series.filter((row) => Number(row.t) <= currentT);
  const latest = atT.at(-1);

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-2 text-xs">
      <h2 className="mb-1 font-semibold text-slate-200">Telemetry</h2>
      <p className="mb-2 text-[10px] text-slate-500">Sparse [METRICS] — not continuous authority.</p>
      {latest ? (
        <dl className="grid gap-0.5 text-slate-400">
          <div>
            <dt className="inline text-slate-500">id: </dt>
            <dd className="inline">{String(latest.interceptor_id)}</dd>
          </div>
          <div>
            <dt className="inline text-slate-500">dist: </dt>
            <dd className="inline">{String(latest.dist_m)} m</dd>
          </div>
          <div>
            <dt className="inline text-slate-500">t_go: </dt>
            <dd className="inline">{String(latest.t_go_s ?? "n/a")} s</dd>
          </div>
        </dl>
      ) : (
        <p className="text-slate-600">No telemetry at t={currentT}</p>
      )}
    </section>
  );
}
