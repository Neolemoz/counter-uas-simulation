import type { ExperimentRun } from "./experimentSchema";
import { entityCount, sideFromPinnedRun } from "./experimentCompare";

export function ExperimentRunSummaryCard({ run }: { run: ExperimentRun }) {
  const side = sideFromPinnedRun(run);
  return (
    <div className="rounded border border-slate-800 bg-slate-950/40 p-2 text-xs text-slate-400">
      <div className="font-medium text-slate-300">{run.label}</div>
      <div className="font-mono text-[10px] text-slate-500">{run.run_id}</div>
      <div>mode: {side.tactical?.tactical_mode ?? "—"}</div>
      <div>entities: {entityCount(side.worldSummary)}</div>
      {run.capture_candidate_id && (
        <div className="text-amber-200/80">capture: {run.capture_candidate_id.slice(0, 8)}</div>
      )}
    </div>
  );
}
