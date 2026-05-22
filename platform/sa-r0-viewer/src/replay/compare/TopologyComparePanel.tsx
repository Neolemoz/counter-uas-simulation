import { useMemo } from "react";
import { useCompareStore } from "../compareStore";
import { computeTopologyDiff } from "../topologyDiff";

export function TopologyComparePanel() {
  const slotA = useCompareStore((s) => s.slotA);
  const slotB = useCompareStore((s) => s.slotB);

  const diff = useMemo(() => {
    if (!slotA.bundle || !slotB.bundle) return null;
    return computeTopologyDiff(
      slotA.bundle,
      slotB.bundle,
      slotA.bundle.scenario.title,
      slotB.bundle.scenario.title,
    );
  }, [slotA.bundle, slotB.bundle]);

  if (!diff) return null;

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <h2 className="mb-2 font-semibold text-slate-200">Topology comparison</h2>
      <div className="mb-2 grid grid-cols-2 gap-2 text-xs text-slate-400">
        <div>
          <span className="text-slate-500">A layout</span>
          <p className="font-mono text-[10px] text-slate-500">
            {slotA.bundle?.comparison_hints?.sensor_layout_id ?? "—"}
          </p>
        </div>
        <div>
          <span className="text-slate-500">B layout</span>
          <p className="font-mono text-[10px] text-slate-500">
            {slotB.bundle?.comparison_hints?.sensor_layout_id ?? "—"}
          </p>
        </div>
      </div>
      {diff.tagIntersection.length > 0 && (
        <p className="mb-1 text-xs text-slate-500">
          Shared tags: {diff.tagIntersection.join(", ")}
        </p>
      )}
      {(diff.tagOnlyA.length > 0 || diff.tagOnlyB.length > 0) && (
        <p className="mb-2 text-xs text-slate-500">
          {diff.tagOnlyA.length > 0 && <>A-only: {diff.tagOnlyA.join(", ")}. </>}
          {diff.tagOnlyB.length > 0 && <>B-only: {diff.tagOnlyB.join(", ")}.</>}
        </p>
      )}
      <ul className="max-h-40 space-y-1 overflow-y-auto text-xs text-slate-300">
        {diff.bullets.map((b, i) => (
          <li key={i} className="list-inside list-disc">
            {b}
          </li>
        ))}
      </ul>
    </section>
  );
}
