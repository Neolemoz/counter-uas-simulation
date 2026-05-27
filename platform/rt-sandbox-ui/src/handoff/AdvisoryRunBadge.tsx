import { CAPTURE_PIPELINE_STEPS } from "@/workflow/captureHandoffCognition";
import { AdvisoryStateBadge } from "./AdvisoryStateBadge";
import type { AdvisoryStatus } from "./advisoryTypes";

export function AdvisoryRunBadge({ status }: { status: AdvisoryStatus | null }) {
  if (!status) return null;

  return (
    <div
      className="mt-1 space-y-1 rounded border border-slate-800 bg-slate-950/40 p-1.5"
      data-testid="advisory-run-badge"
    >
      <AdvisoryStateBadge
        state={status.advisory_state}
        blocked={status.blocked}
        label={status.advisory_state_label}
      />
      <details className="text-[9px] text-slate-500">
        <summary className="cursor-pointer text-slate-400">Pipeline map (read-only)</summary>
        <ul className="mt-1 space-y-0.5">
          {CAPTURE_PIPELINE_STEPS.slice(0, 4).map((step) => (
            <li key={step.id} className="font-mono">
              {step.cli}
            </li>
          ))}
        </ul>
      </details>
    </div>
  );
}
