import { BANNER_MANUAL_HANDOFF_ONLY, BANNER_SA_WORKFLOW_ADVISORY } from "@/governance/banners";
import { AdvisoryStateBadge } from "@/handoff/AdvisoryStateBadge";
import type { AdvisoryStatus } from "@/handoff/advisoryTypes";
import type { HandoffEligibility } from "./experimentSchema";
import { handoffDisplayLevel } from "./experimentF5UiHelpers";

export function ExperimentImportAdvisoryStrip({
  advisoryStatus,
  handoff,
}: {
  advisoryStatus: AdvisoryStatus | null;
  handoff?: HandoffEligibility | null;
}) {
  const f5Display = handoff ? handoffDisplayLevel(handoff.experiment_level) : null;
  const f5Warn = f5Display !== null && f5Display !== "eligible";

  return (
    <section className="space-y-2" data-testid="experiment-import-advisory-strip">
      <p className="text-[10px] text-amber-100/80">{BANNER_SA_WORKFLOW_ADVISORY}</p>
      <p className="text-[10px] text-slate-500">{BANNER_MANUAL_HANDOFF_ONLY}</p>

      <p className="text-[10px] text-slate-400">
        Per-capture import advisory — experiment F5 eligibility does not imply import ready
      </p>

      {f5Warn && handoff && (
        <p className="text-[10px] text-amber-200/80">
          F5 experiment handoff: {handoffDisplayLevel(handoff.experiment_level)} — warn only
        </p>
      )}

      {advisoryStatus ? (
        <>
          <AdvisoryStateBadge
            state={advisoryStatus.advisory_state}
            blocked={advisoryStatus.blocked}
            label={advisoryStatus.advisory_state_label}
          />

          {advisoryStatus.block_reasons.length > 0 && (
            <ul className="list-inside list-disc text-[10px] text-red-300/90">
              {advisoryStatus.block_reasons.map((r) => (
                <li key={r}>{r}</li>
              ))}
            </ul>
          )}
        </>
      ) : (
        <p className="text-[10px] text-slate-500">No capture-linked advisory for active session.</p>
      )}

      <p className="text-[10px] text-slate-600">
        corpus diff — maintainer CLI only (P2)
      </p>
      <p className="text-[10px] font-mono text-slate-500">
        SA lineage begins only at rt_sa_import commit --corpus-dest
      </p>
    </section>
  );
}
