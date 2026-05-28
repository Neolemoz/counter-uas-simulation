import type { ExperimentCohortIndex } from "./cohortSchema";
import type { ExperimentManifest } from "./experimentSchema";
import { deriveManifestRefStatus, runCountLabel } from "./manifestRefStatus";
import {
  selectPrimaryManifestRef,
  selectSecondaryManifestRef,
  saveWorkbenchV2State,
  type WorkbenchV2State,
} from "./workbenchV2State";

const COHORT_MANIFEST_CAP = 8;

export function ExperimentManifestRoster({
  v2State,
  cohort,
  loadedManifest,
  onV2StateChange,
}: {
  v2State: WorkbenchV2State;
  cohort: ExperimentCohortIndex | null;
  loadedManifest: ExperimentManifest;
  onV2StateChange: (state: WorkbenchV2State) => void;
}) {
  if (!cohort) {
    return (
      <p className="text-[10px] text-slate-500" data-testid="manifest-roster">
        Import a cohort to view manifest roster.
      </p>
    );
  }

  const tagFilter = v2State.cohort_tag_filter;
  const refs = cohort.manifest_refs.filter((ref) => {
    if (!tagFilter) return true;
    return ref.experiment_class === tagFilter;
  });

  const overCap = cohort.manifest_refs.length > COHORT_MANIFEST_CAP;

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
    <div className="space-y-2" data-testid="manifest-roster">
      {overCap && (
        <p className="text-[10px] text-amber-200/80">
          Cohort has {cohort.manifest_refs.length} manifests (warn: &gt;{COHORT_MANIFEST_CAP}).
        </p>
      )}
      <table className="w-full text-left text-[11px] text-slate-400">
        <thead>
          <tr className="border-b border-slate-800 text-[10px] uppercase text-slate-500">
            <th className="py-1 pr-2">Label</th>
            <th className="py-1 pr-2">experiment_id</th>
            <th className="py-1 pr-2">Runs</th>
            <th className="py-1 pr-2">Role</th>
            <th className="py-1">Status</th>
          </tr>
        </thead>
        <tbody>
          {refs.map((ref) => {
            const isPrimary = v2State.primary_manifest_ref === ref.manifest_ref;
            const isSecondary = v2State.secondary_manifest_ref === ref.manifest_ref;
            const status = deriveManifestRefStatus(
              ref,
              loadedManifest,
              isPrimary || isSecondary,
            );
            return (
              <tr key={ref.manifest_ref} className="border-b border-slate-900/80">
                <td className="py-1 pr-2 text-slate-300">{ref.label}</td>
                <td className="max-w-[6rem] truncate py-1 pr-2 font-mono" title={ref.experiment_id}>
                  {ref.experiment_id}
                </td>
                <td className="py-1 pr-2">{runCountLabel(ref, loadedManifest)}</td>
                <td className="py-1 pr-2">
                  {isPrimary && (
                    <span className="mr-1 rounded bg-cyan-900/30 px-1 text-[10px] text-cyan-300">
                      primary
                    </span>
                  )}
                  {isSecondary && (
                    <span className="rounded bg-slate-800 px-1 text-[10px] text-slate-400">
                      secondary
                    </span>
                  )}
                </td>
                <td className="py-1">
                  <span className="text-[10px] text-slate-500">{status}</span>
                  <div className="mt-0.5 flex gap-2">
                    <button
                      type="button"
                      className="text-[10px] text-cyan-400 underline"
                      onClick={() => setPrimary(ref.manifest_ref)}
                    >
                      primary
                    </button>
                    <button
                      type="button"
                      className="text-[10px] text-slate-500 underline"
                      onClick={() =>
                        setSecondary(isSecondary ? null : ref.manifest_ref)
                      }
                    >
                      {isSecondary ? "clear 2°" : "secondary"}
                    </button>
                  </div>
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}
