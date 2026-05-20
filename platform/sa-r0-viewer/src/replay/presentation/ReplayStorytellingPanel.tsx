import type { ReplaySaBundle } from "../bundleSchema";
import { useSweepStore } from "../useSweepStore";
import { buildBundleStorytelling, buildSweepStorytelling } from "../cognition/replayStorytelling";

type Props = {
  bundle: ReplaySaBundle | null;
};

export function ReplayStorytellingPanel({ bundle }: Props) {
  const sweep = useSweepStore((s) => s.sweep);
  const sections = bundle ? buildBundleStorytelling(bundle) : [];
  const sweepSections = sweep ? buildSweepStorytelling(sweep) : [];
  const merged = [...sections];
  for (const s of sweepSections) {
    if (!merged.some((m) => m.id === s.id)) merged.push(s);
  }

  if (!merged.length) {
    return (
      <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm text-slate-500">
        No storytelling sections available for this replay.
      </section>
    );
  }

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <h2 className="mb-1 font-semibold text-slate-200">Replay storytelling</h2>
      <p className="mb-3 text-xs text-slate-500">
        Deterministic replay-derived summaries — explanatory only, not causal proof.
      </p>
      <ul className="space-y-3">
        {merged.map((s) => (
          <li key={s.id} className="rounded border border-slate-800 bg-slate-950/50 p-2">
            <h3 className="mb-1 text-xs font-semibold uppercase tracking-wide text-amber-200/90">
              {s.title}
            </h3>
            <p className="text-xs text-slate-300">{s.body}</p>
          </li>
        ))}
      </ul>
    </section>
  );
}
