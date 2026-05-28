import { StatusBadge } from "@/workstation/StatusBadge";
import { CAPTURE_PIPELINE_STEPS } from "@/workflow/captureHandoffCognition";
import { advisoryStateTone } from "./advisoryLabels";
import type { AdvisoryState, AdvisoryStatus } from "./advisoryTypes";

function nextStepCli(state: AdvisoryState | null): string | null {
  switch (state) {
    case "capture_ready":
      return "scripts/rt/rt_handoff_review.py ready <capture_id>";
    case "review_complete":
      return "scripts/rt/rt_handoff_review.py reviewed <capture_id>";
    case "approval_ready":
      return "scripts/rt/rt_capture_approve.py <capture_id>";
    case "handoff_ready":
      return "scripts/rt/rt_sa_import.py prepare <capture_id>";
    case "import_ready":
      return "scripts/rt/rt_sa_import.py commit --corpus-dest <capture_id>";
    default:
      return null;
  }
}

export function HandoffAdvisoryMirrorStrip({
  status,
  compact = false,
}: {
  status: AdvisoryStatus | null;
  compact?: boolean;
}) {
  if (!status) {
    return (
      <p className="text-xs text-slate-500">
        Advisory available after capture — see handoff pipeline panel for mirror rows.
      </p>
    );
  }

  const tone = advisoryStateTone(status.advisory_state, status.blocked);
  const hint = nextStepCli(status.advisory_state);

  return (
    <div
      className={`rounded border border-slate-700 bg-slate-950/50 ${compact ? "p-2" : "p-3"}`}
    >
      <div className="mb-2 flex flex-wrap items-center gap-2">
        <span className="text-xs font-semibold uppercase tracking-wide text-slate-400">
          SA workflow advisory
        </span>
        <StatusBadge
          label={status.advisory_state_label}
          tone={tone}
          title={`Advisory state: ${status.advisory_state_label} — not SA authority`}
        />
      </div>

      <p className="mb-2 text-[11px] text-amber-200/90">{status.governance_banner}</p>

      {status.blocked && status.block_reasons.length > 0 && (
        <ul className="mb-2 list-inside list-disc text-xs text-red-300/90">
          {status.block_reasons.map((reason) => (
            <li key={reason}>{reason}</li>
          ))}
        </ul>
      )}

      {status.lineage_warnings && status.lineage_warnings.length > 0 && (
        <ul className="mb-2 list-inside list-disc text-xs text-amber-300/90">
          {status.lineage_warnings.map((w) => (
            <li key={w}>{w}</li>
          ))}
        </ul>
      )}

      {hint && (
        <p className="mb-2 font-mono text-[10px] text-slate-400">
          Next maintainer step: {hint.replace("<capture_id>", status.capture_candidate_id)}
        </p>
      )}

      {!compact && (
        <details className="text-[10px] text-slate-500">
          <summary className="cursor-pointer text-slate-400">Pipeline map (read-only)</summary>
          <ul className="mt-2 space-y-1">
            {CAPTURE_PIPELINE_STEPS.map((step) => (
              <li key={step.id}>
                <span className="text-slate-400">{step.phase}.</span> {step.title} —{" "}
                <span className="font-mono">{step.cli}</span>
              </li>
            ))}
          </ul>
        </details>
      )}

      {status.terminal && (
        <p className="mt-2 text-xs text-emerald-300/80">
          Terminal: {status.terminal} — SA lineage active; not an advisory rung.
        </p>
      )}
    </div>
  );
}
