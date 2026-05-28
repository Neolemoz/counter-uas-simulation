import { useCallback, useEffect, useState } from "react";
import { COHORT_GOVERNANCE_BANNER } from "./cohortSchema";
import {
  exportCohortJson,
  getCohort,
  importCohortJson,
  listCohortIds,
} from "./cohortIndexStore";
import { safeParseCohortIndex } from "./cohortImportGuards";
import { formatImportError } from "./experimentImportGuards";
import type { ExperimentCohortIndex } from "./cohortSchema";
import {
  loadWorkbenchV2State,
  saveWorkbenchV2State,
  selectPrimaryManifestRef,
  selectSecondaryManifestRef,
  setActiveCohort as setActiveCohortId,
  type WorkbenchV2State,
} from "./workbenchV2State";

export function ExperimentCohortNavigator({
  v2State,
  onV2StateChange,
}: {
  v2State: WorkbenchV2State;
  onV2StateChange: (state: WorkbenchV2State) => void;
}) {
  const [cohortIds, setCohortIds] = useState<string[]>(() => listCohortIds());
  const [activeCohort, setActiveCohort] = useState<ExperimentCohortIndex | null>(() =>
    v2State.active_cohort_id ? getCohort(v2State.active_cohort_id) : null,
  );
  const [importError, setImportError] = useState<string | null>(null);

  const refreshCohorts = useCallback(() => {
    setCohortIds(listCohortIds());
  }, []);

  useEffect(() => {
    if (!v2State.active_cohort_id) {
      setActiveCohort(null);
      return;
    }
    setActiveCohort(getCohort(v2State.active_cohort_id));
  }, [v2State.active_cohort_id]);

  const selectCohort = (cohortId: string) => {
    const cohort = getCohort(cohortId);
    if (!cohort) return;
    let next = setActiveCohortId(loadWorkbenchV2State(), cohortId);
    const firstRef = cohort.manifest_refs[0]?.manifest_ref ?? null;
    next = selectPrimaryManifestRef(next, firstRef);
    if (cohort.manifest_refs.length > 1) {
      next = selectSecondaryManifestRef(next, cohort.manifest_refs[1]?.manifest_ref ?? null);
    }
    saveWorkbenchV2State(next);
    onV2StateChange(next);
    setActiveCohort(cohort);
  };

  const importCohort = () => {
    const text = window.prompt("Paste rt_experiment_cohort_index_v1 JSON");
    if (!text) return;
    const preview = safeParseCohortIndex(text);
    if (!preview.ok) {
      setImportError(formatImportError(preview.error));
      return;
    }
    try {
      const cohort = importCohortJson(text);
      setImportError(null);
      refreshCohorts();
      selectCohort(cohort.cohort_id);
    } catch (err) {
      setImportError(formatImportError(String(err)));
    }
  };

  const exportActive = () => {
    if (!v2State.active_cohort_id) return;
    try {
      const text = exportCohortJson(v2State.active_cohort_id);
      void navigator.clipboard.writeText(text);
    } catch {
      setImportError("export failed");
    }
  };

  const setPrimary = (manifestRef: string) => {
    const next = selectPrimaryManifestRef(v2State, manifestRef);
    saveWorkbenchV2State(next);
    onV2StateChange(next);
  };

  const setSecondary = (manifestRef: string | null) => {
    const next = selectSecondaryManifestRef(v2State, manifestRef);
    saveWorkbenchV2State(next);
    onV2StateChange(next);
  };

  return (
    <div className="space-y-2 rounded border border-slate-800 bg-slate-950/40 p-2" data-testid="cohort-navigator">
      <p className="text-[10px] text-slate-500">{COHORT_GOVERNANCE_BANNER}</p>
      <p className="text-[10px] text-amber-200/80">
        Experiment cohort — not F7 readiness cohort; per-manifest authority unchanged
      </p>
      <div className="flex flex-wrap gap-2">
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
          onClick={importCohort}
        >
          Import cohort JSON
        </button>
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200 disabled:opacity-40"
          disabled={!v2State.active_cohort_id}
          onClick={exportActive}
        >
          Copy cohort export
        </button>
      </div>
      {importError && <p className="text-xs text-red-400">{importError}</p>}
      {cohortIds.length > 0 && (
        <label className="flex flex-col gap-1 text-xs text-slate-400">
          Stored cohorts
          <select
            className="rounded border border-slate-700 bg-slate-950 px-2 py-1 font-mono text-xs"
            value={v2State.active_cohort_id ?? ""}
            onChange={(e) => {
              if (e.target.value) selectCohort(e.target.value);
            }}
          >
            <option value="">— select —</option>
            {cohortIds.map((id) => (
              <option key={id} value={id}>
                {id}
              </option>
            ))}
          </select>
        </label>
      )}
      {activeCohort && (
        <div className="space-y-2">
          <p className="text-xs font-medium text-slate-300">{activeCohort.label}</p>
          <ul className="space-y-1">
            {activeCohort.manifest_refs.map((ref) => (
              <li
                key={ref.manifest_ref}
                className="rounded border border-slate-800 px-2 py-1 text-[11px] text-slate-400"
              >
                <span className="font-mono text-slate-300">{ref.label}</span>
                {ref.run_count_hint != null && (
                  <span className="ml-2 text-slate-500">~{ref.run_count_hint} runs</span>
                )}
                <div className="truncate font-mono text-[10px]">{ref.manifest_ref}</div>
                <div className="mt-1 flex gap-2">
                  <button
                    type="button"
                    className="text-[10px] text-cyan-400 underline"
                    onClick={() => setPrimary(ref.manifest_ref)}
                  >
                    Set primary
                  </button>
                  <button
                    type="button"
                    className="text-[10px] text-slate-500 underline"
                    onClick={() =>
                      setSecondary(
                        v2State.secondary_manifest_ref === ref.manifest_ref
                          ? null
                          : ref.manifest_ref,
                      )
                    }
                  >
                    {v2State.secondary_manifest_ref === ref.manifest_ref
                      ? "Clear secondary"
                      : "Set secondary"}
                  </button>
                </div>
              </li>
            ))}
          </ul>
        </div>
      )}
    </div>
  );
}
