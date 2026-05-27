import { getAnnexForRun } from "./annexReviewStore";
import { timelineCounts } from "./tacticalAnnexSchema";

export function TacticalAnnexCompareStrip({
  runIdA,
  runIdB,
  labelA,
  labelB,
}: {
  runIdA?: string;
  runIdB?: string;
  labelA: string;
  labelB: string;
}) {
  if (!runIdA && !runIdB) return null;

  const annexA = runIdA ? getAnnexForRun(runIdA) : null;
  const annexB = runIdB ? getAnnexForRun(runIdB) : null;

  const badges: { id: string; label: string }[] = [];
  if (runIdA && !annexA) badges.push({ id: "annex_missing_a", label: "annex_missing_a" });
  if (runIdB && !annexB) badges.push({ id: "annex_missing_b", label: "annex_missing_b" });
  if (
    annexA?.final_tactical_mode &&
    annexB?.final_tactical_mode &&
    annexA.final_tactical_mode !== annexB.final_tactical_mode
  ) {
    badges.push({ id: "mode_final_diff", label: "mode_final_diff" });
  }

  const countKeys = [
    "mode_switches",
    "assignment_timeline",
    "selected_timeline",
    "tti_timeline",
    "recommendation_timeline",
    "pause_resume_transitions",
  ] as const;

  return (
    <div
      className="rounded border border-slate-800 bg-slate-950/50 p-2 text-[10px] text-slate-400"
      data-testid="tactical-annex-compare-strip"
    >
      <p className="mb-2 font-medium text-slate-500">Annex compare (finals + counts)</p>
      {badges.length > 0 && (
        <div className="mb-2 flex flex-wrap gap-1">
          {badges.map((b) => (
            <span
              key={b.id}
              className="rounded border border-slate-700 bg-slate-900 px-1.5 py-0.5 text-sky-200/90"
            >
              {b.label}
            </span>
          ))}
        </div>
      )}
      <table className="w-full">
        <thead>
          <tr className="text-left text-slate-500">
            <th className="pr-2">field</th>
            <th className="pr-2">{labelA}</th>
            <th>{labelB}</th>
          </tr>
        </thead>
        <tbody>
          <tr className="border-t border-slate-800">
            <td className="py-1">final mode</td>
            <td>{annexA?.final_tactical_mode ?? "—"}</td>
            <td>{annexB?.final_tactical_mode ?? "—"}</td>
          </tr>
          <tr className="border-t border-slate-800">
            <td className="py-1">selected</td>
            <td className="font-mono">{annexA?.selected_id ?? "—"}</td>
            <td className="font-mono">{annexB?.selected_id ?? "—"}</td>
          </tr>
          <tr className="border-t border-slate-800">
            <td className="py-1">assigned</td>
            <td className="font-mono">{annexA?.assigned_target ?? "—"}</td>
            <td className="font-mono">{annexB?.assigned_target ?? "—"}</td>
          </tr>
          {countKeys.map((key) => (
            <tr key={key} className="border-t border-slate-800">
              <td className="py-1">{key}</td>
              <td>{annexA ? (timelineCounts(annexA)[key] ?? 0) : "—"}</td>
              <td>{annexB ? (timelineCounts(annexB)[key] ?? 0) : "—"}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}
