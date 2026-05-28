import type { ExperimentCohortIndex } from "./cohortSchema";
import {
  saveWorkbenchV2State,
  selectSecondaryManifestRef,
  type WorkbenchV2State,
} from "./workbenchV2State";

export function ExperimentSecondaryManifestPicker({
  v2State,
  cohort,
  onV2StateChange,
}: {
  v2State: WorkbenchV2State;
  cohort: ExperimentCohortIndex | null;
  onV2StateChange: (state: WorkbenchV2State) => void;
}) {
  if (!cohort) return null;

  return (
    <label
      className="flex flex-col gap-1 text-xs text-slate-400"
      data-testid="secondary-manifest-picker"
    >
      Secondary manifest (explicit)
      <select
        className="rounded border border-slate-700 bg-slate-950 px-2 py-1 font-mono text-xs"
        value={v2State.secondary_manifest_ref ?? ""}
        onChange={(e) => {
          const value = e.target.value || null;
          const next = selectSecondaryManifestRef(v2State, value);
          saveWorkbenchV2State(next);
          onV2StateChange(next);
        }}
      >
        <option value="">— none —</option>
        {cohort.manifest_refs.map((ref) => (
          <option key={ref.manifest_ref} value={ref.manifest_ref}>
            {ref.label} ({ref.experiment_id})
          </option>
        ))}
      </select>
    </label>
  );
}
