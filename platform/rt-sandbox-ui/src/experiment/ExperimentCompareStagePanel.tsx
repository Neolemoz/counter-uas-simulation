import { COMPARE_MODE_UI, compareModeLabel, type CompareModeId } from "./experimentUnifiedReview";
import {
  buildManifestSummaryChips,
  MULTI_MANIFEST_DIFF_BANNER,
} from "./multiManifestDiff";
import type { ExperimentCohortIndex } from "./cohortSchema";
import { ManifestSummaryChips } from "./ManifestSummaryChips";
import { setCompareMode, type WorkbenchV2State } from "./workbenchV2State";

export function ExperimentCompareStagePanel({
  v2State,
  onV2StateChange,
  onApplyCompareMode,
  cohort,
}: {
  v2State: WorkbenchV2State;
  onV2StateChange: (state: WorkbenchV2State) => void;
  onApplyCompareMode: (mode: CompareModeId) => void;
  cohort?: ExperimentCohortIndex | null;
}) {
  const isMultiManifest = v2State.compare_mode === "multi_manifest_diff";
  const chips =
    isMultiManifest && cohort
      ? buildManifestSummaryChips({
          cohort,
          primaryRef: v2State.primary_manifest_ref,
          secondaryRef: v2State.secondary_manifest_ref,
        })
      : [];

  return (
    <div
      className="space-y-2 rounded border border-slate-800 bg-slate-950/40 p-2"
      data-testid="compare-stage-panel"
    >
      <label className="flex flex-col gap-1 text-xs text-slate-400">
        Compare mode
        <select
          className="rounded border border-slate-700 bg-slate-950 px-2 py-1 text-xs"
          value={v2State.compare_mode}
          onChange={(e) => {
            const mode = e.target.value as CompareModeId;
            const next = setCompareMode(v2State, mode);
            onV2StateChange(next);
            onApplyCompareMode(mode);
          }}
        >
          {COMPARE_MODE_UI.map((m) => (
            <option key={m.id} value={m.id}>
              {m.label}
            </option>
          ))}
        </select>
      </label>
      <p className="text-xs text-slate-400">
        Active: <span className="font-mono text-slate-300">{compareModeLabel(v2State.compare_mode)}</span>
      </p>
      {v2State.primary_manifest_ref && (
        <p className="truncate font-mono text-[10px] text-slate-500">
          Primary manifest: {v2State.primary_manifest_ref}
        </p>
      )}
      {v2State.secondary_manifest_ref && (
        <p className="truncate font-mono text-[10px] text-slate-500">
          Secondary manifest: {v2State.secondary_manifest_ref}
        </p>
      )}
      {isMultiManifest ? (
        <div className="space-y-2">
          <p className="text-[10px] text-amber-200/80">{MULTI_MANIFEST_DIFF_BANNER}</p>
          <p className="text-[10px] text-slate-500">
            Set primary and secondary in the cohort navigator — metadata table below.
          </p>
          {chips.length > 0 && <ManifestSummaryChips chips={chips} />}
        </div>
      ) : (
        <p className="text-[10px] text-slate-500">
          Modes delegate to frozen panels below — pairwise, extended, matrix, fidelity.
        </p>
      )}
    </div>
  );
}
