import { useMemo } from "react";
import { useCompareStore } from "../compareStore";

export function SweepCompareAnalyticsStrip() {
  const slotA = useCompareStore((s) => s.slotA);
  const slotB = useCompareStore((s) => s.slotB);

  const sameSweep = useMemo(() => {
    const idA = slotA.bundle?.comparison_hints?.sweep_id;
    const idB = slotB.bundle?.comparison_hints?.sweep_id;
    return idA && idB && idA === idB;
  }, [slotA.bundle, slotB.bundle]);

  if (!sameSweep) return null;

  return (
    <section className="rounded border border-violet-900/40 bg-violet-950/20 p-2 text-xs text-slate-300">
      <strong className="text-violet-200">Sweep context:</strong> both slots share sweep{" "}
      <code className="text-violet-300">{slotA.bundle?.comparison_hints?.sweep_id}</code> — use
      sweep mode for full distribution panels.
    </section>
  );
}
