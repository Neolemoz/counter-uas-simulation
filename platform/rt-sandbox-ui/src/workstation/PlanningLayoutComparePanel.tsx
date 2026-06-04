import {
  derivePlanningLayoutCompareAnalytics,
  MAX_PLANNING_LAYOUT_COMPARE_SLOTS,
  PLANNING_LAYOUT_COMPARE_GOVERNANCE_COPY,
  PLANNING_LAYOUT_COMPARE_SLOT_LABELS,
  type PlanningLayoutCompareAnalyticsV1,
  type PlanningLayoutCompareDeltaV1,
  type PlanningLayoutCompareSlotLabel,
  type PlanningLayoutCompareSlotV1,
} from "@/layout/planningLayoutComparison";

function formatDeltaValue(delta: PlanningLayoutCompareDeltaV1): string {
  if (delta.metric === "radar_count") {
    return `${delta.delta >= 0 ? "+" : ""}${delta.delta}`;
  }
  return `${delta.delta >= 0 ? "+" : ""}${delta.delta.toFixed(1)}%`;
}

function formatDeltaLabel(label: PlanningLayoutCompareDeltaV1["label"]): string {
  return label.charAt(0).toUpperCase() + label.slice(1);
}

function formatRecommendationSummary(
  summary: PlanningLayoutCompareAnalyticsV1["side_by_side"][number]["recommendation_summary"],
): string {
  const radar = summary.suggested_radar ?? "None";
  const position = summary.suggested_position
    ? `${Math.round(summary.suggested_position.x)}, ${Math.round(summary.suggested_position.y)}`
    : "None";
  const reason = summary.reason ?? "No advisory reason recorded.";
  return `Suggested radar ${radar} · Position ${position} · ${reason}`;
}

export function PlanningLayoutComparePanel({
  slots,
  analytics,
  captureDisabled,
  importSlotLabel,
  importText,
  importError,
  onImportSlotLabelChange,
  onImportTextChange,
  onImportSnapshot,
  onCaptureCurrentLayout,
  onRemoveSlot,
  onClearAllSlots,
}: {
  slots: readonly PlanningLayoutCompareSlotV1[];
  analytics: PlanningLayoutCompareAnalyticsV1;
  captureDisabled: boolean;
  importSlotLabel: PlanningLayoutCompareSlotLabel;
  importText: string;
  importError: string | null;
  onImportSlotLabelChange: (slotLabel: PlanningLayoutCompareSlotLabel) => void;
  onImportTextChange: (value: string) => void;
  onImportSnapshot: () => void;
  onCaptureCurrentLayout: () => void;
  onRemoveSlot: (slotLabel: PlanningLayoutCompareSlotLabel) => void;
  onClearAllSlots: () => void;
}) {
  const { rows, warnings, deltas, side_by_side: sideBySide } = analytics;
  const slotsFull = slots.length >= MAX_PLANNING_LAYOUT_COMPARE_SLOTS;
  const deltaTargets = [...new Set(deltas.map((delta) => delta.target_slot))];

  return (
    <div
      className="mt-3 rounded border border-sky-900/60 bg-sky-950/20 p-2"
      data-testid="planning-layout-compare-panel"
    >
      <p className="text-[11px] font-semibold uppercase tracking-wide text-sky-200">
        Planning layout comparison
      </p>
      <p
        className="mt-1 text-[11px] text-slate-500"
        data-testid="planning-layout-compare-governance"
      >
        Planning-only · read-only · non-authoritative. {PLANNING_LAYOUT_COMPARE_GOVERNANCE_COPY}
      </p>
      {warnings.length > 0 ? (
        <div
          className="mt-2 grid gap-1"
          data-testid="planning-layout-compare-warnings"
        >
          {warnings.map((warning) => (
            <p
              key={`${warning.warning_id}:${warning.slot_labels.join("-")}`}
              className="rounded border border-amber-700/60 bg-amber-950/35 px-2 py-1 text-[11px] text-amber-100"
              data-testid={`planning-layout-compare-warning-${warning.warning_id}`}
              role="alert"
            >
              {warning.message}
            </p>
          ))}
        </div>
      ) : null}
      {rows.length > 0 ? (
        <div className="mt-2 overflow-x-auto" data-testid="planning-layout-compare-table">
          <table className="w-full min-w-[32rem] border-collapse text-[11px] text-slate-300">
            <thead>
              <tr className="border-b border-slate-800 text-left text-slate-400">
                <th className="px-2 py-1 font-semibold">Slot</th>
                <th className="px-2 py-1 font-semibold">Snapshot</th>
                <th className="px-2 py-1 font-semibold">Extent</th>
                <th className="px-2 py-1 font-semibold">Coverage %</th>
                <th className="px-2 py-1 font-semibold">Overlap %</th>
                <th className="px-2 py-1 font-semibold">Redundancy %</th>
                <th className="px-2 py-1 font-semibold">Radars</th>
                <th className="px-2 py-1 font-semibold">Actions</th>
              </tr>
            </thead>
            <tbody>
              {rows.map((row) => (
                <tr
                  key={row.slot_label}
                  className="border-b border-slate-900/80"
                  data-testid={`planning-layout-compare-row-${row.slot_label}`}
                >
                  <td className="px-2 py-1 font-semibold text-sky-100">{row.slot_label}</td>
                  <td className="px-2 py-1 font-mono text-[10px]">{row.planning_snapshot_id}</td>
                  <td className="px-2 py-1">{row.planning_extent_id}</td>
                  <td className="px-2 py-1">{row.coverage_percent.toFixed(1)}%</td>
                  <td className="px-2 py-1">{row.overlap_percent.toFixed(1)}%</td>
                  <td className="px-2 py-1">{row.redundancy_percent.toFixed(1)}%</td>
                  <td className="px-2 py-1">{row.radar_count}</td>
                  <td className="px-2 py-1">
                    <button
                      type="button"
                      onClick={() => onRemoveSlot(row.slot_label)}
                      className="rounded border border-slate-700 bg-slate-950 px-2 py-0.5 text-[10px] font-semibold uppercase text-slate-200"
                      data-testid={`planning-layout-compare-remove-${row.slot_label}`}
                    >
                      Remove
                    </button>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      ) : (
        <p className="mt-2 text-[11px] text-slate-500" data-testid="planning-layout-compare-empty">
          No captured layout compare slots yet.
        </p>
      )}
      {deltas.length > 0 ? (
        <div className="mt-3 overflow-x-auto" data-testid="planning-layout-compare-deltas">
          <p className="mb-1 text-[11px] font-semibold uppercase tracking-wide text-slate-300">
            Delta vs slot A
          </p>
          <table className="w-full min-w-[28rem] border-collapse text-[11px] text-slate-300">
            <thead>
              <tr className="border-b border-slate-800 text-left text-slate-400">
                <th className="px-2 py-1 font-semibold">Metric</th>
                {deltaTargets.map((targetSlot) => (
                  <th key={targetSlot} className="px-2 py-1 font-semibold">
                    {targetSlot} − A
                  </th>
                ))}
              </tr>
            </thead>
            <tbody>
              {(["coverage_percent", "overlap_percent", "redundancy_percent", "radar_count"] as const).map(
                (metric) => (
                  <tr
                    key={metric}
                    className="border-b border-slate-900/80"
                    data-testid={`planning-layout-compare-delta-row-${metric}`}
                  >
                    <td className="px-2 py-1">{metric.replace(/_/g, " ")}</td>
                    {deltaTargets.map((targetSlot) => {
                      const delta = deltas.find(
                        (row) => row.metric === metric && row.target_slot === targetSlot,
                      );
                      if (!delta) {
                        return (
                          <td key={targetSlot} className="px-2 py-1 text-slate-500">
                            —
                          </td>
                        );
                      }
                      return (
                        <td
                          key={targetSlot}
                          className="px-2 py-1"
                          data-testid={`planning-layout-compare-delta-${metric}-${targetSlot}`}
                        >
                          {formatDeltaValue(delta)} · {formatDeltaLabel(delta.label)}
                        </td>
                      );
                    })}
                  </tr>
                ),
              )}
            </tbody>
          </table>
        </div>
      ) : null}
      {sideBySide.length > 0 ? (
        <div className="mt-3 grid gap-2" data-testid="planning-layout-compare-blind-spots">
          <p className="text-[11px] font-semibold uppercase tracking-wide text-slate-300">
            Blind spot summaries
          </p>
          {sideBySide.map((entry) => (
            <p
              key={`blind-${entry.slot_label}`}
              className="rounded border border-slate-800 bg-slate-950/70 px-2 py-1 text-[11px] text-slate-300"
              data-testid={`planning-layout-compare-blind-spot-${entry.slot_label}`}
            >
              Slot {entry.slot_label}: {entry.blind_spot_summary}
            </p>
          ))}
        </div>
      ) : null}
      {sideBySide.length > 0 ? (
        <div className="mt-3 grid gap-2" data-testid="planning-layout-compare-recommendations">
          <p className="text-[11px] font-semibold uppercase tracking-wide text-slate-300">
            Recommendation summaries
          </p>
          {sideBySide.map((entry) => (
            <p
              key={`rec-${entry.slot_label}`}
              className="rounded border border-slate-800 bg-slate-950/70 px-2 py-1 text-[11px] text-slate-300"
              data-testid={`planning-layout-compare-recommendation-${entry.slot_label}`}
            >
              Slot {entry.slot_label}: {formatRecommendationSummary(entry.recommendation_summary)}
            </p>
          ))}
        </div>
      ) : null}
      <div className="mt-2 flex flex-wrap gap-2">
        <button
          type="button"
          disabled={captureDisabled}
          onClick={onCaptureCurrentLayout}
          className="rounded border border-sky-700/60 bg-sky-950/45 px-2 py-1 text-[10px] font-semibold uppercase text-sky-100 disabled:opacity-40"
          data-testid="planning-layout-compare-capture"
        >
          Capture Current Layout
        </button>
        <button
          type="button"
          disabled={slots.length === 0}
          onClick={onClearAllSlots}
          className="rounded border border-slate-700 bg-slate-950 px-2 py-1 text-[10px] font-semibold uppercase text-slate-200 disabled:opacity-40"
          data-testid="planning-layout-compare-clear-all"
        >
          Clear All Slots
        </button>
        {slotsFull ? (
          <span className="self-center text-[10px] text-amber-200/90">
            All {MAX_PLANNING_LAYOUT_COMPARE_SLOTS} compare slots are full.
          </span>
        ) : null}
      </div>
      <div
        className="mt-3 rounded border border-slate-800 bg-slate-950/70 p-2"
        data-testid="planning-layout-compare-import"
      >
        <p className="text-[11px] font-semibold uppercase tracking-wide text-slate-300">
          Import snapshot JSON
        </p>
        <p className="mt-1 text-[11px] text-slate-500">
          Paste rt_planning_mc_snapshot_v1 JSON into the selected slot. Validation only; no persistence.
        </p>
        <div className="mt-2 flex flex-wrap items-end gap-2">
          <label className="grid gap-1 text-[11px] text-slate-400">
            Target slot
            <select
              value={importSlotLabel}
              onChange={(event) =>
                onImportSlotLabelChange(event.currentTarget.value as PlanningLayoutCompareSlotLabel)
              }
              className="rounded border border-slate-700 bg-slate-950 px-2 py-1 text-slate-100"
              data-testid="planning-layout-compare-import-slot"
            >
              {PLANNING_LAYOUT_COMPARE_SLOT_LABELS.map((slotLabel) => (
                <option key={slotLabel} value={slotLabel}>
                  Slot {slotLabel}
                </option>
              ))}
            </select>
          </label>
        </div>
        <label className="mt-2 grid gap-1 text-[11px] text-slate-400">
          Snapshot JSON
          <textarea
            value={importText}
            onChange={(event) => onImportTextChange(event.currentTarget.value)}
            rows={3}
            className="rounded border border-slate-800 bg-slate-950 px-2 py-1 font-mono text-[10px] text-slate-200"
            placeholder='{"schema_version":"rt_planning_mc_snapshot_v1",...}'
            data-testid="planning-layout-compare-import-text"
          />
        </label>
        {importError ? (
          <p
            className="mt-2 rounded border border-rose-700/60 bg-rose-950/35 px-2 py-1 text-[11px] text-rose-100"
            role="alert"
            data-testid="planning-layout-compare-import-error"
          >
            {importError}
          </p>
        ) : null}
        <div className="mt-2">
          <button
            type="button"
            disabled={importText.trim().length === 0}
            onClick={onImportSnapshot}
            className="rounded border border-sky-700/60 bg-sky-950/45 px-2 py-1 text-[10px] font-semibold uppercase text-sky-100 disabled:opacity-40"
            data-testid="planning-layout-compare-import-submit"
          >
            Import snapshot
          </button>
        </div>
      </div>
    </div>
  );
}

export function buildPlanningLayoutComparePanelDerivation(
  slots: readonly PlanningLayoutCompareSlotV1[],
): PlanningLayoutCompareAnalyticsV1 {
  return derivePlanningLayoutCompareAnalytics(slots);
}
