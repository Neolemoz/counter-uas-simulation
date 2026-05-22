import { useMemo } from "react";
import { useCompareStore } from "../compareStore";
import { extractReplayOutcome, formatDeltaT } from "../replayOutcomeSummary";

export function ReplayOutcomeComparePanel() {
  const slotA = useCompareStore((s) => s.slotA);
  const slotB = useCompareStore((s) => s.slotB);

  const rows = useMemo(() => {
    if (!slotA.bundle || !slotB.bundle) return [];
    const a = extractReplayOutcome(slotA.bundle);
    const b = extractReplayOutcome(slotB.bundle);
    return [
      { label: "Replay span (log lines)", a: String(a.durationSpan), b: String(b.durationSpan), delta: formatDeltaT(a.durationSpan, b.durationSpan) },
      { label: "First detection t", a: a.firstDetectionT != null ? String(a.firstDetectionT) : "—", b: b.firstDetectionT != null ? String(b.firstDetectionT) : "—", delta: formatDeltaT(a.firstDetectionT, b.firstDetectionT) },
      { label: "First selection t", a: a.firstSelectionT != null ? String(a.firstSelectionT) : "—", b: b.firstSelectionT != null ? String(b.firstSelectionT) : "—", delta: formatDeltaT(a.firstSelectionT, b.firstSelectionT) },
      { label: "Ambiguity windows", a: String(a.ambiguityWindowCount), b: String(b.ambiguityWindowCount), delta: formatDeltaT(a.ambiguityWindowCount, b.ambiguityWindowCount) },
      { label: "LOS degraded segments", a: String(a.losDegradedCount), b: String(b.losDegradedCount), delta: formatDeltaT(a.losDegradedCount, b.losDegradedCount) },
    ];
  }, [slotA.bundle, slotB.bundle]);

  if (rows.length === 0) return null;

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <h2 className="mb-2 font-semibold text-slate-200">Replay observations</h2>
      <table className="w-full text-xs">
        <thead>
          <tr className="text-left text-slate-500">
            <th className="pb-1">Metric</th>
            <th>A</th>
            <th>B</th>
            <th>Δ</th>
          </tr>
        </thead>
        <tbody>
          {rows.map((r) => (
            <tr key={r.label} className="border-t border-slate-800 text-slate-300">
              <td className="py-1 pr-2 text-slate-400">{r.label}</td>
              <td className="py-1">{r.a}</td>
              <td className="py-1">{r.b}</td>
              <td className="py-1 font-mono text-amber-200/90">{r.delta}</td>
            </tr>
          ))}
        </tbody>
      </table>
      <p className="mt-2 text-[10px] text-slate-500">
        Descriptive replay observations — not validated effectiveness or tactical superiority.
      </p>
    </section>
  );
}
