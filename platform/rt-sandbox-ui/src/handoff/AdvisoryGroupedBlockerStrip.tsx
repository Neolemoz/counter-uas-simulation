import { useState } from "react";
import {
  ADVISORY_GOVERNANCE_BANNER,
  type AdvisoryExperimentRollup,
  type EnrichedAdvisoryRow,
} from "./advisoryTypes";
import { rollupBlockerGroupExemplars } from "./advisoryTriageGrouping";
import { shortCaptureId } from "./captureIdDisplay";

const GROUP_LABELS: Record<string, string> = {
  normalization: "normalization",
  review_attestation: "review / attestation",
  approval_gate: "approval gate",
  packaging: "packaging",
  lineage: "lineage",
  experiment_warn: "experiment (warn-only)",
  terminal_block: "reject / defer",
};

export function AdvisoryGroupedBlockerStrip({
  rows,
  experimentRollup,
}: {
  rows: EnrichedAdvisoryRow[];
  experimentRollup?: AdvisoryExperimentRollup | null;
}) {
  const [open, setOpen] = useState(true);
  const rollup = rollupBlockerGroupExemplars(rows);
  const entries = Object.entries(rollup).filter(([, v]) => (v?.count ?? 0) > 0);

  if (entries.length === 0 && !experimentRollup) {
    return null;
  }

  return (
    <div className="mb-4 rounded border border-slate-700 bg-slate-950/50 p-3">
      <button
        type="button"
        className="mb-2 flex w-full items-center justify-between text-left text-xs font-semibold uppercase tracking-wide text-slate-400"
        onClick={() => setOpen((v) => !v)}
        aria-expanded={open}
      >
        <span>Grouped blockers (F7 — read-only)</span>
        <span className="font-normal normal-case text-slate-500">{open ? "−" : "+"}</span>
      </button>
      {open && (
        <>
          <p className="mb-2 text-[11px] text-amber-200/90">{ADVISORY_GOVERNANCE_BANNER}</p>
          {experimentRollup && (
            <p className="mb-2 text-xs text-slate-500">
              Experiment rollup: {experimentRollup.handoff_eligibility} —{" "}
              {experimentRollup.note ?? "warn-only"}
            </p>
          )}
          <ul className="space-y-2">
            {entries.map(([id, data]) => (
              <li
                key={id}
                className="rounded border border-slate-800/80 bg-slate-900/40 px-2 py-1.5 text-xs"
              >
                <span className="font-medium text-slate-300">
                  {GROUP_LABELS[id] ?? id}
                </span>
                <span className="ml-2 text-slate-500">({data?.count ?? 0})</span>
                <div className="mt-1 font-mono text-[10px] text-slate-500">
                  {(data?.exemplar_capture_ids ?? []).map(shortCaptureId).join(", ")}
                </div>
              </li>
            ))}
          </ul>
        </>
      )}
    </div>
  );
}
