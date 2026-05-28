import type { ExperimentCohortIndex } from "./cohortSchema";
import {
  setActiveRunId,
  setBreadcrumbFocus,
  setCohortTagFilter,
  type BreadcrumbFocus,
  type WorkbenchV2State,
} from "./workbenchV2State";

export function ExperimentProgramContextStrip({
  v2State,
  cohort,
  onV2StateChange,
}: {
  v2State: WorkbenchV2State;
  cohort: ExperimentCohortIndex | null;
  onV2StateChange: (state: WorkbenchV2State) => void;
}) {
  const cohortLabel = cohort?.label ?? "single manifest";
  const tags = cohort?.tags ?? [];
  const primaryRef = v2State.primary_manifest_ref;
  const secondaryRef = v2State.secondary_manifest_ref;

  const navigateBreadcrumb = (focus: BreadcrumbFocus) => {
    let next = setBreadcrumbFocus(v2State, focus);
    if (focus === "cohort" || focus === "manifest") {
      next = setActiveRunId(next, null);
    }
    onV2StateChange(next);
  };

  const focusClass = (focus: BreadcrumbFocus) =>
    v2State.breadcrumb_focus === focus
      ? "text-cyan-300 underline"
      : "text-slate-400 underline hover:text-slate-200";

  return (
    <div
      className="space-y-2 rounded border border-slate-800 bg-slate-950/50 p-2"
      data-testid="program-context-strip"
    >
      <p className="text-xs font-medium text-slate-300">{cohortLabel}</p>
      {tags.length > 0 && (
        <div className="flex flex-wrap gap-1">
          {tags.map((tag) => (
            <button
              key={tag}
              type="button"
              className={
                v2State.cohort_tag_filter === tag
                  ? "rounded bg-cyan-900/40 px-2 py-0.5 text-[10px] text-cyan-200"
                  : "rounded border border-slate-700 px-2 py-0.5 text-[10px] text-slate-400"
              }
              onClick={() =>
                onV2StateChange(
                  setCohortTagFilter(
                    v2State,
                    v2State.cohort_tag_filter === tag ? null : tag,
                  ),
                )
              }
            >
              {tag}
            </button>
          ))}
          {v2State.cohort_tag_filter && (
            <button
              type="button"
              className="text-[10px] text-slate-500 underline"
              onClick={() => onV2StateChange(setCohortTagFilter(v2State, null))}
            >
              clear filter
            </button>
          )}
        </div>
      )}
      <nav className="flex flex-wrap items-center gap-1 text-[10px]">
        <button type="button" className={focusClass("cohort")} onClick={() => navigateBreadcrumb("cohort")}>
          cohort
        </button>
        <span className="text-slate-600">→</span>
        <button
          type="button"
          className={focusClass("manifest")}
          onClick={() => navigateBreadcrumb("manifest")}
        >
          manifest
        </button>
        <span className="text-slate-600">→</span>
        <button
          type="button"
          className={focusClass("run")}
          onClick={() => navigateBreadcrumb("run")}
          disabled={!v2State.active_run_id}
        >
          run
        </button>
      </nav>
      <p className="text-[10px] text-slate-500">
        {primaryRef ? (
          <>
            Primary: <span className="font-mono text-slate-300">{primaryRef}</span>
          </>
        ) : (
          "Primary: —"
        )}
        {secondaryRef ? (
          <>
            {" "}
            · Secondary: <span className="font-mono text-slate-300">{secondaryRef}</span>
          </>
        ) : null}
        {v2State.active_run_id ? (
          <>
            {" "}
            → run <span className="font-mono text-slate-300">{v2State.active_run_id}</span>
          </>
        ) : null}
      </p>
    </div>
  );
}
