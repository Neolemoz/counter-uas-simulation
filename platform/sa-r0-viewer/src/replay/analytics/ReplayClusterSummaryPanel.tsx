import { useSweepStore } from "../useSweepStore";

export function ReplayClusterSummaryPanel() {
  const sweep = useSweepStore((s) => s.sweep);
  if (!sweep) return null;

  const clusters = sweep.spatial_aggregate.layers.replay_event_clusters;
  if (!clusters?.centroids_enu_m?.length) return null;

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <h2 className="mb-1 font-semibold text-slate-200">Event concentration zones</h2>
      <ul className="space-y-1 text-xs text-slate-300">
        {(clusters.labels ?? []).map((label, i) => (
          <li key={i}>
            {label}
            {clusters.centroids_enu_m?.[i] && (
              <span className="text-slate-500">
                {" "}
                @ ENU [{clusters.centroids_enu_m[i]!.join(", ")}]
              </span>
            )}
          </li>
        ))}
      </ul>
    </section>
  );
}
