import { BANNER_SA_WORKFLOW_ADVISORY } from "@/governance/banners";
import { CHECKLIST_LABELS } from "./advisoryChecklist";
import { checklistChipLabel, checklistStatusIcon } from "./advisoryLabels";
import { AdvisoryStateBadge } from "./AdvisoryStateBadge";
import type { AdvisoryStatus } from "./advisoryTypes";

export function SaWorkflowAdvisoryPanel({
  status,
  selectedCaptureId,
}: {
  status: AdvisoryStatus | null;
  selectedCaptureId?: string | null;
}) {
  return (
    <section
      className="rounded border border-slate-700 bg-slate-950/50 p-3"
      data-testid="sa-workflow-advisory-panel"
    >
      <p className="mb-2 text-[10px] text-amber-100/80">{BANNER_SA_WORKFLOW_ADVISORY}</p>

      {!status ? (
        <p className="text-xs text-slate-500">
          Select a staged capture to view workflow checklist (read-only).
        </p>
      ) : (
        <>
          <div className="mb-2 flex flex-wrap items-center gap-2">
            <span className="text-xs font-semibold uppercase tracking-wide text-slate-400">
              Review checklist
            </span>
            {selectedCaptureId && (
              <span className="font-mono text-[10px] text-slate-500">{selectedCaptureId}</span>
            )}
            {status.terminal ? (
              <span className="text-xs text-slate-400">{status.advisory_state_label}</span>
            ) : (
              <AdvisoryStateBadge
                state={status.advisory_state}
                blocked={status.blocked}
                label={status.advisory_state_label}
              />
            )}
          </div>

          <ul className="mb-2 flex flex-wrap gap-1.5" aria-label="SA workflow checklist items">
            {(status.checklist ?? []).map((item) => {
              const label = CHECKLIST_LABELS[item.id as keyof typeof CHECKLIST_LABELS] ?? item.id;
              const icon = checklistStatusIcon(item.status);
              const aria = checklistChipLabel(item.id, item.status);
              return (
                <li
                  key={item.id}
                  className="flex items-center gap-1 rounded border border-slate-700 bg-slate-900 px-2 py-0.5 text-[10px] text-slate-300"
                  title={item.detail ?? aria}
                  aria-label={aria}
                >
                  <span aria-hidden="true">{icon}</span>
                  <span>{label}</span>
                  {item.detail && (
                    <span className="text-slate-500">({item.detail})</span>
                  )}
                </li>
              );
            })}
          </ul>

          {status.blocked && status.block_reasons.length > 0 && (
            <details className="text-xs text-red-300/90" open>
              <summary className="cursor-pointer text-red-200/90">Block reasons</summary>
              <ul className="mt-1 list-inside list-disc">
                {status.block_reasons.map((reason) => (
                  <li key={reason}>{reason}</li>
                ))}
              </ul>
            </details>
          )}
        </>
      )}
    </section>
  );
}
