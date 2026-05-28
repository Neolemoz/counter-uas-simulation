import {
  useCallback,
  useMemo,
  useState,
  type ReactNode,
  type SyntheticEvent,
} from "react";
import { BANNER_SA_WORKFLOW_ADVISORY } from "@/governance/banners";
import { AdvisoryBlockerGroupChips } from "./AdvisoryBlockerGroupChips";
import { AdvisoryFocusChips } from "./AdvisoryFocusChips";
import { AdvisoryPresetSelector } from "./AdvisoryPresetSelector";
import { AdvisoryQueueBandChip } from "./AdvisoryQueueBandChip";
import { AdvisoryRollupSummaryBar } from "./AdvisoryRollupSummaryBar";
import {
  AdvisoryStandupPassSelector,
  filterPresetForStandupPass,
} from "./AdvisoryStandupPassSelector";
import { AdvisoryStateBadge } from "./AdvisoryStateBadge";
import { ReadinessCohortChip } from "./ReadinessCohortChip";
import { ReadinessCohortV2Chip } from "./ReadinessCohortV2Chip";
import {
  buildClientExportPreviewV2,
  formatExportJsonPreview,
} from "./advisoryBatchExportPreview";
import {
  renderTemplatePackPreview,
  STALE_AGE_HOURS,
  type StandupPassDef,
} from "./advisoryAggregationV2";
import {
  getGroupOpenState,
  setGroupOpenState,
} from "./advisoryTriageGroupMemory";
import {
  defaultBandOpen,
  formatTriageStandUpSummary,
  groupEnrichedRows,
} from "./advisoryTriageGrouping";
import {
  ADVISORY_GOVERNANCE_BANNER,
  type AdvisoryExperimentRollup,
  type EnrichedAdvisoryRow,
  type FilterPresetId,
  type SessionAdvisorySummary,
  type TemplatePackId,
  type TriageGroupMode,
  type TriageSortMode,
} from "./advisoryTypes";
import { shortCaptureId } from "./captureIdDisplay";
import { StatusBadge } from "@/workstation/StatusBadge";

const TEMPLATE_PACK_OPTIONS: { id: TemplatePackId; label: string }[] = [
  { id: "standup_json_v2", label: "JSON v2" },
  { id: "standup_md_daily", label: "Markdown daily" },
  { id: "standup_md_minimal", label: "Markdown minimal" },
];

function TriageRow({
  row,
  selected,
  onSelect,
}: {
  row: EnrichedAdvisoryRow;
  selected: boolean;
  onSelect: (id: string) => void;
}) {
  const { status } = row;
  const stateLabel = status.terminal
    ? "committed"
    : status.advisory_state_label;
  const ariaState = status.terminal
    ? "terminal committed"
    : status.advisory_state ?? "pending";
  const staleWarn =
    row.stale_age_hours != null && row.stale_age_hours >= STALE_AGE_HOURS;

  return (
    <button
      type="button"
      className={`w-full rounded border px-2 py-1.5 text-left text-xs ${
        selected
          ? "border-amber-700/60 bg-amber-950/30"
          : row.in_focus_set
            ? "border-violet-700/50 bg-violet-950/20 ring-1 ring-violet-800/40"
            : "border-slate-800/80 bg-slate-900/40 hover:bg-slate-900/70"
      }`}
      onClick={() => onSelect(row.capture_candidate_id)}
      aria-label={`Capture ${row.capture_candidate_id}, advisory ${ariaState}, not SA authority`}
      aria-pressed={selected}
    >
      <div className="flex flex-wrap items-center gap-2">
        <span className="font-mono text-slate-200">
          {shortCaptureId(row.capture_candidate_id)}
        </span>
        <AdvisoryQueueBandChip priority={row.queue_priority} />
        <ReadinessCohortChip cohort={row.readiness_cohort} />
        {row.readiness_cohort_v2 && (
          <ReadinessCohortV2Chip cohort={row.readiness_cohort_v2} />
        )}
        {staleWarn && (
          <span
            className="text-[10px] text-amber-300/80"
            title="Stale age advisory — not operational readiness"
          >
            stale {row.stale_age_hours}h
          </span>
        )}
        {status.terminal ? (
          <StatusBadge label="committed" tone="ok" title={stateLabel} />
        ) : status.advisory_state ? (
          <AdvisoryStateBadge
            state={status.advisory_state}
            blocked={status.blocked}
            label={stateLabel}
          />
        ) : null}
        <AdvisoryBlockerGroupChips groups={row.blocker_groups} max={5} />
      </div>
      {status.lineage_warnings && status.lineage_warnings.length > 0 && (
        <ul className="mt-1 list-inside list-disc text-[10px] text-amber-200/80">
          {status.lineage_warnings.map((w) => (
            <li key={w}>{w}</li>
          ))}
        </ul>
      )}
    </button>
  );
}

function PersistedDetails({
  groupMode,
  groupKey,
  defaultOpen,
  label,
  rowCount,
  children,
}: {
  groupMode: TriageGroupMode;
  groupKey: string;
  defaultOpen: boolean;
  label: string;
  rowCount: number;
  children: ReactNode;
}) {
  const [open, setOpen] = useState(() =>
    getGroupOpenState(groupMode, groupKey, defaultOpen),
  );

  const onToggle = (e: SyntheticEvent<HTMLDetailsElement>) => {
    const next = (e.target as HTMLDetailsElement).open;
    setOpen(next);
    setGroupOpenState(groupMode, groupKey, next);
  };

  return (
    <details
      className="rounded border border-slate-800/80 bg-slate-900/30"
      open={open}
      onToggle={onToggle}
    >
      <summary className="cursor-pointer px-2 py-1.5 text-xs font-medium text-slate-300">
        {label}
        <span className="ml-2 font-normal text-slate-500">({rowCount})</span>
      </summary>
      {children}
    </details>
  );
}

export function AdvisoryTriageQueuePanel({
  rows,
  selectedCaptureId,
  onSelectCapture,
  experimentRollup,
  filterPreset,
  onFilterPresetChange,
  focusSet,
  captureIdsForFocus,
  onFocusToggle,
  onFocusClear,
  sessionSummary,
  cohortHintFilter,
  onCohortHintFilterChange,
  rowCountsByPass,
}: {
  rows: EnrichedAdvisoryRow[];
  selectedCaptureId: string | null;
  onSelectCapture: (id: string | null) => void;
  experimentRollup?: AdvisoryExperimentRollup | null;
  filterPreset?: FilterPresetId;
  onFilterPresetChange?: (id: FilterPresetId) => void;
  focusSet?: ReadonlySet<string>;
  captureIdsForFocus?: string[];
  onFocusToggle?: (id: string) => void;
  onFocusClear?: () => void;
  sessionSummary?: SessionAdvisorySummary | null;
  cohortHintFilter?: string | null;
  onCohortHintFilterChange?: (hint: string | null) => void;
  rowCountsByPass?: Partial<Record<string, number>>;
}) {
  const showF8Toolbar =
    onFilterPresetChange != null ||
    onFocusToggle != null ||
    sessionSummary != null;

  const [groupMode, setGroupMode] = useState<TriageGroupMode>("queue_band");
  const [sortMode, setSortMode] = useState<TriageSortMode>("queue");
  const [copyStatus, setCopyStatus] = useState<string | null>(null);
  const [activePassId, setActivePassId] = useState<string | null>(null);
  const [templatePackId, setTemplatePackId] = useState<TemplatePackId>(
    "standup_md_minimal",
  );

  const groups = useMemo(
    () => groupEnrichedRows(rows, groupMode, { sort: sortMode, experimentRollup }),
    [rows, groupMode, sortMode, experimentRollup],
  );

  const focusedRowIds = useMemo(() => {
    if (!focusSet?.size) return [];
    return rows
      .filter((r) => focusSet.has(r.capture_candidate_id))
      .map((r) => r.capture_candidate_id);
  }, [rows, focusSet]);

  const cycleFocusSelection = useCallback(
    (delta: number) => {
      if (focusedRowIds.length === 0) return;
      const idx = selectedCaptureId
        ? focusedRowIds.indexOf(selectedCaptureId)
        : -1;
      const next =
        idx < 0
          ? focusedRowIds[0]
          : focusedRowIds[(idx + delta + focusedRowIds.length) % focusedRowIds.length];
      onSelectCapture(next ?? null);
    },
    [focusedRowIds, selectedCaptureId, onSelectCapture],
  );

  const handleCopyText = async () => {
    const allGroups = groupEnrichedRows(rows, "queue_band", { sort: sortMode });
    const text = formatTriageStandUpSummary(allGroups);
    try {
      await navigator.clipboard.writeText(text);
      setCopyStatus("Copied stand-up summary");
      window.setTimeout(() => setCopyStatus(null), 2000);
    } catch {
      setCopyStatus("Copy failed");
    }
  };

  const handleCopyJson = async () => {
    const preview = buildClientExportPreviewV2(rows, experimentRollup);
    try {
      await navigator.clipboard.writeText(formatExportJsonPreview(preview));
      setCopyStatus("Copied JSON preview (read-only)");
      window.setTimeout(() => setCopyStatus(null), 2000);
    } catch {
      setCopyStatus("Copy failed");
    }
  };

  const handleCopyTemplate = async () => {
    if (!sessionSummary) return;
    const text = renderTemplatePackPreview(sessionSummary, templatePackId);
    try {
      await navigator.clipboard.writeText(text);
      setCopyStatus("Copied F8 template preview (read-only)");
      window.setTimeout(() => setCopyStatus(null), 2000);
    } catch {
      setCopyStatus("Copy failed");
    }
  };

  const handleSelect = (id: string) => {
    onSelectCapture(selectedCaptureId === id ? null : id);
  };

  const handleStandupPass = (pass: StandupPassDef) => {
    setActivePassId(pass.pass_id);
    const { preset, cohortHint } = filterPresetForStandupPass(pass);
    onFilterPresetChange?.(preset);
    onCohortHintFilterChange?.(cohortHint);
  };

  return (
    <section
      className="mb-4 rounded border border-slate-700 bg-slate-950/50 p-3"
      data-testid="advisory-triage-queue-panel"
    >
      <h3 className="mb-2 text-xs font-semibold uppercase tracking-wide text-slate-400">
        Advisory triage queue (F8 hub — read-only)
      </h3>
      <p className="mb-1 text-[10px] text-amber-100/80">{BANNER_SA_WORKFLOW_ADVISORY}</p>
      <p className="mb-3 text-[10px] text-amber-200/70">{ADVISORY_GOVERNANCE_BANNER}</p>

      {showF8Toolbar && sessionSummary && (
        <AdvisoryRollupSummaryBar
          summary={sessionSummary}
          scopeNote="table uses preset/focus filter"
        />
      )}

      {showF8Toolbar && filterPreset != null && onFilterPresetChange && (
        <AdvisoryPresetSelector value={filterPreset} onChange={onFilterPresetChange} />
      )}

      {showF8Toolbar &&
        captureIdsForFocus &&
        focusSet != null &&
        onFocusToggle &&
        onFocusClear && (
          <>
            <AdvisoryFocusChips
              captureIds={captureIdsForFocus}
              focusSet={focusSet}
              onToggle={onFocusToggle}
              onClear={onFocusClear}
            />
            {focusSet.size > 0 && (
              <div className="mb-2 flex flex-wrap gap-2 text-xs">
                <button
                  type="button"
                  className="rounded border border-slate-600 bg-slate-900 px-2 py-0.5 text-slate-400"
                  onClick={() => cycleFocusSelection(-1)}
                >
                  Focus prev
                </button>
                <button
                  type="button"
                  className="rounded border border-slate-600 bg-slate-900 px-2 py-0.5 text-slate-400"
                  onClick={() => cycleFocusSelection(1)}
                >
                  Focus next
                </button>
              </div>
            )}
          </>
        )}

      {showF8Toolbar && onFilterPresetChange && (
        <AdvisoryStandupPassSelector
          activePassId={activePassId}
          onSelectPass={handleStandupPass}
          rowCountsByPass={rowCountsByPass}
        />
      )}

      {cohortHintFilter && (
        <p className="mb-2 text-[10px] text-violet-200/80">
          Cohort hint filter: {cohortHintFilter.replace(/_/g, " ")} (Pass D — advisory only)
        </p>
      )}

      {showF8Toolbar && sessionSummary && (
        <details className="mb-3 rounded border border-slate-800/80 bg-slate-900/30">
          <summary className="cursor-pointer px-2 py-1.5 text-xs text-slate-400">
            Template pack preview (render-only)
          </summary>
          <div className="border-t border-slate-800/80 p-2">
            <p className="mb-2 text-[10px] text-slate-500">
              Preview only — does not run maintainer CLIs or commit.
            </p>
            <label className="mb-2 flex flex-wrap items-center gap-2 text-xs text-slate-400">
              <span>Pack</span>
              <select
                className="rounded border border-slate-600 bg-slate-900 px-2 py-1 text-slate-200"
                value={templatePackId}
                onChange={(e) => setTemplatePackId(e.target.value as TemplatePackId)}
                aria-label="F8 template pack"
              >
                {TEMPLATE_PACK_OPTIONS.map((o) => (
                  <option key={o.id} value={o.id}>
                    {o.label}
                  </option>
                ))}
              </select>
              <button
                type="button"
                className="rounded border border-slate-600 bg-slate-900 px-2 py-0.5 text-slate-400"
                onClick={() => void handleCopyTemplate()}
              >
                Copy F8 template
              </button>
            </label>
            <pre
              className="max-h-40 overflow-auto rounded border border-slate-800 bg-slate-900/80 p-2 text-[10px] text-slate-400 whitespace-pre-wrap"
              data-testid="f8-template-preview"
            >
              {renderTemplatePackPreview(sessionSummary, templatePackId)}
            </pre>
          </div>
        </details>
      )}

      {rows.length === 0 ? (
        <p className="text-xs text-slate-500">No staged captures to triage.</p>
      ) : (
        <>
          <div className="mb-3 flex flex-wrap items-center gap-2 text-xs">
            <label className="flex items-center gap-1 text-slate-500">
              View
              <select
                className="rounded border border-slate-700 bg-slate-900 px-1.5 py-0.5 text-slate-300"
                value={groupMode}
                onChange={(e) => setGroupMode(e.target.value as TriageGroupMode)}
                aria-label="Triage grouping mode"
              >
                <option value="queue_band">queue band</option>
                <option value="blocker">blocker group</option>
                <option value="cohort">readiness cohort</option>
                <option value="cohort_v2">cohort v2</option>
                <option value="handoff_stage">handoff stage</option>
                <option value="experiment">experiment rollup</option>
              </select>
            </label>
            <label className="flex items-center gap-1 text-slate-500">
              Sort
              <select
                className="rounded border border-slate-700 bg-slate-900 px-1.5 py-0.5 text-slate-300"
                value={sortMode}
                onChange={(e) => setSortMode(e.target.value as TriageSortMode)}
                aria-label="Triage sort mode"
              >
                <option value="queue">queue priority</option>
                <option value="capture_id">capture id</option>
              </select>
            </label>
            <button
              type="button"
              className="rounded border border-slate-600 bg-slate-900 px-2 py-0.5 text-slate-400 hover:text-slate-200"
              onClick={() => void handleCopyText()}
            >
              Copy stand-up summary
            </button>
            <button
              type="button"
              className="rounded border border-slate-600 bg-slate-900 px-2 py-0.5 text-slate-400 hover:text-slate-200"
              onClick={() => void handleCopyJson()}
              title="Client-side v2 preview — not SA authority"
            >
              Copy JSON preview
            </button>
            {copyStatus && <span className="text-slate-500">{copyStatus}</span>}
          </div>

          {groupMode === "experiment" && groups.length === 0 && (
            <p className="mb-2 text-xs text-slate-500">
              No experiment warn captures. Load experiment metrics in the workbench for
              warn-only rollup, or select captures with experiment_warn blockers.
            </p>
          )}

          {experimentRollup && groupMode === "experiment" && (
            <p className="mb-2 text-xs text-slate-500">
              Rollup: {experimentRollup.handoff_eligibility} —{" "}
              {experimentRollup.note ?? "warn-only"}
            </p>
          )}

          <div className="space-y-2">
            {groups.map((group) => {
              const isBand = groupMode === "queue_band";
              const defaultOpen = isBand ? defaultBandOpen(group.key) : true;
              return (
                <PersistedDetails
                  key={group.key}
                  groupMode={groupMode}
                  groupKey={group.key}
                  defaultOpen={defaultOpen}
                  label={group.label}
                  rowCount={group.rows.length}
                >
                  <ul className="space-y-1 border-t border-slate-800/80 p-2">
                    {group.rows.map((row) => (
                      <li key={`${group.key}-${row.capture_candidate_id}`}>
                        <TriageRow
                          row={row}
                          selected={row.capture_candidate_id === selectedCaptureId}
                          onSelect={handleSelect}
                        />
                      </li>
                    ))}
                  </ul>
                </PersistedDetails>
              );
            })}
          </div>
        </>
      )}
    </section>
  );
}
