import { useSweepStore } from "../useSweepStore";

export function ReplayAnomalyBanner() {
  const sweep = useSweepStore((s) => s.sweep);
  const emphasize = useSweepStore((s) => s.emphasizeAnomalies);
  const setEmphasize = useSweepStore((s) => s.setEmphasizeAnomalies);
  if (!sweep?.replay_cohorts?.length) return null;

  const anomalyIndices = new Set<number>();
  for (const c of sweep.replay_cohorts) {
    for (const i of c.anomaly_member_indices ?? []) anomalyIndices.add(i);
  }
  if (!anomalyIndices.size) return null;

  const labels = [...anomalyIndices].map((i) => sweep.members[i]?.member_id ?? `#${i}`);

  return (
    <section
      className={`rounded border p-2 text-xs ${
        emphasize
          ? "border-amber-600/60 bg-amber-950/40 text-amber-100"
          : "border-slate-600 bg-slate-800/60 text-slate-300"
      }`}
    >
      <label className="flex cursor-pointer items-start gap-2">
        <input
          type="checkbox"
          checked={emphasize}
          onChange={(e) => setEmphasize(e.target.checked)}
        />
        <span>
          Replay outliers flagged (explanatory): {labels.join(", ")} — timing or LOS
          concentration differs from cohort peers.
        </span>
      </label>
    </section>
  );
}
