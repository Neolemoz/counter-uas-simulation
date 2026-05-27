import { BANNER_EXPERIMENT_F5 } from "@/governance/banners";
import type { HandoffEligibility } from "./experimentSchema";
import { handoffDisplayLevel } from "./experimentF5UiHelpers";

const HANDOFF_DOC_PATH = "docs/evaluation/rt_manual_sa_import_workflow_v1.md";

export function ExperimentHandoffEligibilityStrip({
  handoff,
  maintainerAckPoseReviewed,
  onMaintainerAckPoseReviewedChange,
}: {
  handoff: HandoffEligibility;
  maintainerAckPoseReviewed: boolean;
  onMaintainerAckPoseReviewedChange: (value: boolean) => void;
}) {
  const display = handoffDisplayLevel(handoff.experiment_level);
  const pillClass =
    display === "eligible"
      ? "border-emerald-800/60 text-emerald-200/90"
      : display === "review_needed"
        ? "border-amber-800/60 text-amber-200/90"
        : "border-slate-700 text-slate-300";

  return (
    <section className="space-y-3" data-testid="experiment-handoff-eligibility-strip">
      <p className="text-[10px] text-amber-100/80">{BANNER_EXPERIMENT_F5}</p>
      <p className="text-[10px] text-slate-500">
        Handoff eligibility — advisory only; no import actions
      </p>

      <span
        className={`inline-block rounded border px-2 py-0.5 text-xs font-mono ${pillClass}`}
        data-testid="handoff-display-level"
      >
        {display}
      </span>

      <table className="w-full text-[10px] text-slate-400">
        <thead>
          <tr className="text-left text-slate-500">
            <th className="p-1">gate</th>
            <th className="p-1">pass</th>
            <th className="p-1">detail</th>
          </tr>
        </thead>
        <tbody>
          {handoff.gates.map((g) => (
            <tr key={g.id} className="border-t border-slate-800">
              <td className="p-1 font-mono">{g.id}</td>
              <td className="p-1">{g.pass ? "yes" : "no"}</td>
              <td className="p-1">{g.detail}</td>
            </tr>
          ))}
        </tbody>
      </table>

      {handoff.per_run_gates.length > 0 && (
        <details className="text-[10px] text-slate-500">
          <summary className="cursor-pointer">Per-run gates</summary>
          <ul className="mt-1 space-y-1">
            {handoff.per_run_gates.map((pr) => (
              <li key={pr.run_id} className="font-mono">
                {pr.run_id}: {pr.eligible ? "eligible" : "ineligible"}
              </li>
            ))}
          </ul>
        </details>
      )}

      <label className="flex items-center gap-1 text-xs text-slate-400">
        <input
          type="checkbox"
          checked={maintainerAckPoseReviewed}
          onChange={(e) => onMaintainerAckPoseReviewedChange(e.target.checked)}
        />
        Pose cognition reviewed (local attestation)
      </label>

      <p className="text-[10px] text-slate-600">
        Maintainer workflow: {HANDOFF_DOC_PATH} — manual import only
      </p>
    </section>
  );
}
