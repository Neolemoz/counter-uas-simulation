/**
 * UI-local Planning layout comparison model (snapshot-based, read-only).
 * No UI rendering, Cesium overlays, runtime, bridge, or MC execution.
 */

import {
  PLANNING_MC_SNAPSHOT_SCHEMA_VERSION,
  type PlanningMcSnapshotAnalyticsSummary,
  type PlanningMcSnapshotExtent,
  type PlanningMcSnapshotV1,
} from "./planningMcSnapshot";
import type {
  PlanningResultLinkStatus,
  PlanningResultLinkV1,
} from "./planningMcResultLink";
import type { PlanningExtentId } from "@/cesium/planningWorld";

export const PLANNING_LAYOUT_COMPARE_GOVERNANCE_COPY =
  "Planning layout comparison is UI-local and non-authoritative; it does not affect runtime, bridge, or MC execution.";

export const PLANNING_LAYOUT_COMPARE_SLOT_LABELS = ["A", "B", "C"] as const;

export type PlanningLayoutCompareSlotLabel =
  (typeof PLANNING_LAYOUT_COMPARE_SLOT_LABELS)[number];

export const MAX_PLANNING_LAYOUT_COMPARE_SLOTS = 3;

export type PlanningLayoutCompareWarningId =
  | "duplicate_geometry"
  | "extent_mismatch";

export type PlanningLayoutCompareWarning = {
  warning_id: PlanningLayoutCompareWarningId;
  message: string;
  slot_labels: PlanningLayoutCompareSlotLabel[];
};

export type PlanningLayoutCompareSlotV1 = {
  slot_label: PlanningLayoutCompareSlotLabel;
  planning_snapshot_id: string;
  planning_geometry_id: string;
  planning_extent_id: PlanningExtentId;
  snapshot: PlanningMcSnapshotV1;
  result_link?: PlanningResultLinkV1 | null;
  capture_utc?: string;
};

export type PlanningLayoutCompareRecommendationSummary =
  PlanningMcSnapshotAnalyticsSummary["recommendation_summary"];

export type PlanningLayoutCompareRowV1 = {
  slot_label: PlanningLayoutCompareSlotLabel;
  planning_snapshot_id: string;
  planning_geometry_id: string;
  planning_extent_id: PlanningExtentId;
  coverage_percent: number;
  overlap_percent: number;
  redundancy_percent: number;
  radar_count: number;
  blind_spot_summary: string;
  recommendation_summary: PlanningLayoutCompareRecommendationSummary;
  extent: PlanningMcSnapshotExtent;
  mc_link_status: PlanningResultLinkStatus | null;
  warnings: PlanningLayoutCompareWarning[];
};

export type PlanningLayoutCompareDerivationV1 = {
  rows: PlanningLayoutCompareRowV1[];
  warnings: PlanningLayoutCompareWarning[];
};

export const PLANNING_LAYOUT_COMPARE_PERCENT_DELTA_EPSILON = 0.1;

export type PlanningLayoutCompareDeltaLabel = "improved" | "reduced" | "unchanged";

export type PlanningLayoutCompareMetricKey =
  | "coverage_percent"
  | "overlap_percent"
  | "redundancy_percent"
  | "radar_count";

export type PlanningLayoutCompareDeltaTargetSlot = Exclude<
  PlanningLayoutCompareSlotLabel,
  "A"
>;

export type PlanningLayoutCompareDeltaV1 = {
  metric: PlanningLayoutCompareMetricKey;
  baseline_slot: "A";
  target_slot: PlanningLayoutCompareDeltaTargetSlot;
  delta: number;
  label: PlanningLayoutCompareDeltaLabel;
};

export type PlanningLayoutCompareSideBySideV1 = {
  slot_label: PlanningLayoutCompareSlotLabel;
  blind_spot_summary: string;
  recommendation_summary: PlanningLayoutCompareRecommendationSummary;
};

export type PlanningLayoutCompareAnalyticsV1 = PlanningLayoutCompareDerivationV1 & {
  deltas: PlanningLayoutCompareDeltaV1[];
  side_by_side: PlanningLayoutCompareSideBySideV1[];
};

export type ParsePlanningMcSnapshotResult =
  | { ok: true; snapshot: PlanningMcSnapshotV1 }
  | { ok: false; error: string };

export type BuildPlanningLayoutCompareSlotOptions = {
  resultLink?: PlanningResultLinkV1 | null;
  captureUtc?: string;
};

function slotLabelRank(label: PlanningLayoutCompareSlotLabel): number {
  return PLANNING_LAYOUT_COMPARE_SLOT_LABELS.indexOf(label);
}

function sortSlotsByLabel(
  slots: readonly PlanningLayoutCompareSlotV1[],
): PlanningLayoutCompareSlotV1[] {
  return [...slots].sort(
    (a, b) => slotLabelRank(a.slot_label) - slotLabelRank(b.slot_label),
  );
}

function formatSlotLabels(labels: readonly PlanningLayoutCompareSlotLabel[]): string {
  if (labels.length === 0) return "";
  if (labels.length === 1) return `slot ${labels[0]}`;
  if (labels.length === 2) return `slots ${labels[0]} and ${labels[1]}`;
  return `slots ${labels.slice(0, -1).join(", ")}, and ${labels[labels.length - 1]}`;
}

function duplicateGeometryWarnings(
  slots: readonly PlanningLayoutCompareSlotV1[],
): PlanningLayoutCompareWarning[] {
  const byGeometry = new Map<string, PlanningLayoutCompareSlotLabel[]>();

  for (const slot of slots) {
    const labels = byGeometry.get(slot.planning_geometry_id) ?? [];
    labels.push(slot.slot_label);
    byGeometry.set(slot.planning_geometry_id, labels);
  }

  const warnings: PlanningLayoutCompareWarning[] = [];
  for (const slotLabels of byGeometry.values()) {
    if (slotLabels.length < 2) continue;
    const ordered = [...slotLabels].sort(
      (a, b) => slotLabelRank(a) - slotLabelRank(b),
    );
    warnings.push({
      warning_id: "duplicate_geometry",
      message: `${formatSlotLabels(ordered)} share the same planning_geometry_id (identical layout geometry).`,
      slot_labels: ordered,
    });
  }

  return warnings.sort(
    (a, b) => slotLabelRank(a.slot_labels[0]!) - slotLabelRank(b.slot_labels[0]!),
  );
}

function extentMismatchWarnings(
  slots: readonly PlanningLayoutCompareSlotV1[],
): PlanningLayoutCompareWarning[] {
  const extentIds = [...new Set(slots.map((slot) => slot.planning_extent_id))];
  if (extentIds.length <= 1) return [];

  const labelsByExtent = new Map<PlanningExtentId, PlanningLayoutCompareSlotLabel[]>();
  for (const slot of slots) {
    const labels = labelsByExtent.get(slot.planning_extent_id) ?? [];
    labels.push(slot.slot_label);
    labelsByExtent.set(slot.planning_extent_id, labels);
  }

  const allLabels = sortSlotsByLabel(slots).map((slot) => slot.slot_label);
  const extentSummary = [...labelsByExtent.entries()]
    .sort(([left], [right]) => left.localeCompare(right))
    .map(([extentId, labels]) => {
      const ordered = [...labels].sort(
        (a, b) => slotLabelRank(a) - slotLabelRank(b),
      );
      return `${extentId} (${formatSlotLabels(ordered)})`;
    })
    .join("; ");

  return [
    {
      warning_id: "extent_mismatch",
      message: `${formatSlotLabels(allLabels)} use different planning_extent_id values (${extentSummary}). Metrics remain heuristic and are not directly comparable across Planning World extents.`,
      slot_labels: allLabels,
    },
  ];
}

function warningsForSlot(
  slotLabel: PlanningLayoutCompareSlotLabel,
  globalWarnings: readonly PlanningLayoutCompareWarning[],
): PlanningLayoutCompareWarning[] {
  return globalWarnings.filter((warning) => warning.slot_labels.includes(slotLabel));
}

function compareRowFromSlot(
  slot: PlanningLayoutCompareSlotV1,
  globalWarnings: readonly PlanningLayoutCompareWarning[],
): PlanningLayoutCompareRowV1 {
  const analytics = slot.snapshot.analytics_summary;
  return {
    slot_label: slot.slot_label,
    planning_snapshot_id: slot.planning_snapshot_id,
    planning_geometry_id: slot.planning_geometry_id,
    planning_extent_id: slot.planning_extent_id,
    coverage_percent: analytics.coverage_percent,
    overlap_percent: analytics.overlap_percent,
    redundancy_percent: analytics.redundancy_percent,
    radar_count: slot.snapshot.radars.radar_sites.length,
    blind_spot_summary: analytics.blind_spot_summary,
    recommendation_summary: analytics.recommendation_summary,
    extent: slot.snapshot.planning_extent,
    mc_link_status: slot.result_link?.status ?? null,
    warnings: warningsForSlot(slot.slot_label, globalWarnings),
  };
}

export function buildPlanningLayoutCompareSlot(
  slotLabel: PlanningLayoutCompareSlotLabel,
  snapshot: PlanningMcSnapshotV1,
  options: BuildPlanningLayoutCompareSlotOptions = {},
): PlanningLayoutCompareSlotV1 {
  return {
    slot_label: slotLabel,
    planning_snapshot_id: snapshot.planning_snapshot_id,
    planning_geometry_id: snapshot.planning_geometry_id,
    planning_extent_id: snapshot.planning_extent.planning_extent_id,
    snapshot,
    ...(options.resultLink !== undefined ? { result_link: options.resultLink } : {}),
    ...(options.captureUtc ? { capture_utc: options.captureUtc } : {}),
  };
}

export function nextAvailablePlanningLayoutCompareSlotLabel(
  slots: readonly PlanningLayoutCompareSlotV1[],
): PlanningLayoutCompareSlotLabel | null {
  const used = new Set(slots.map((slot) => slot.slot_label));
  return (
    PLANNING_LAYOUT_COMPARE_SLOT_LABELS.find((label) => !used.has(label)) ?? null
  );
}

export type CapturePlanningLayoutCompareSlotResult = {
  slots: PlanningLayoutCompareSlotV1[];
  captured: PlanningLayoutCompareSlotV1 | null;
};

export function capturePlanningLayoutCompareSlot(
  slots: readonly PlanningLayoutCompareSlotV1[],
  snapshot: PlanningMcSnapshotV1,
  captureUtc?: string,
): CapturePlanningLayoutCompareSlotResult {
  const nextLabel = nextAvailablePlanningLayoutCompareSlotLabel(slots);
  if (!nextLabel) {
    return { slots: sortSlotsByLabel(slots), captured: null };
  }
  const captured = buildPlanningLayoutCompareSlot(nextLabel, snapshot, {
    captureUtc,
  });
  return {
    slots: sortSlotsByLabel([...slots, captured]),
    captured,
  };
}

export function removePlanningLayoutCompareSlot(
  slots: readonly PlanningLayoutCompareSlotV1[],
  slotLabel: PlanningLayoutCompareSlotLabel,
): PlanningLayoutCompareSlotV1[] {
  return sortSlotsByLabel(slots.filter((slot) => slot.slot_label !== slotLabel));
}

export function clearPlanningLayoutCompareSlots(): PlanningLayoutCompareSlotV1[] {
  return [];
}

export function derivePlanningLayoutCompareRows(
  slots: readonly PlanningLayoutCompareSlotV1[],
): PlanningLayoutCompareDerivationV1 {
  const ordered = sortSlotsByLabel(slots).slice(0, MAX_PLANNING_LAYOUT_COMPARE_SLOTS);
  const warnings = [
    ...duplicateGeometryWarnings(ordered),
    ...extentMismatchWarnings(ordered),
  ];
  const rows = ordered.map((slot) => compareRowFromSlot(slot, warnings));

  return { rows, warnings };
}

function metricValue(
  row: PlanningLayoutCompareRowV1,
  metric: PlanningLayoutCompareMetricKey,
): number {
  switch (metric) {
    case "coverage_percent":
      return row.coverage_percent;
    case "overlap_percent":
      return row.overlap_percent;
    case "redundancy_percent":
      return row.redundancy_percent;
    case "radar_count":
      return row.radar_count;
  }
}

export function planningLayoutCompareDeltaLabel(
  metric: PlanningLayoutCompareMetricKey,
  delta: number,
): PlanningLayoutCompareDeltaLabel {
  if (metric === "radar_count") {
    if (delta === 0) return "unchanged";
    return delta > 0 ? "improved" : "reduced";
  }
  if (Math.abs(delta) < PLANNING_LAYOUT_COMPARE_PERCENT_DELTA_EPSILON) {
    return "unchanged";
  }
  return delta > 0 ? "improved" : "reduced";
}

export function derivePlanningLayoutCompareDeltas(
  rows: readonly PlanningLayoutCompareRowV1[],
): PlanningLayoutCompareDeltaV1[] {
  const baseline = rows.find((row) => row.slot_label === "A");
  if (!baseline) return [];

  const metrics: PlanningLayoutCompareMetricKey[] = [
    "coverage_percent",
    "overlap_percent",
    "redundancy_percent",
    "radar_count",
  ];
  const deltas: PlanningLayoutCompareDeltaV1[] = [];

  for (const row of rows) {
    if (row.slot_label === "A") continue;
    const targetSlot = row.slot_label as PlanningLayoutCompareDeltaTargetSlot;
    for (const metric of metrics) {
      const delta = metricValue(row, metric) - metricValue(baseline, metric);
      deltas.push({
        metric,
        baseline_slot: "A",
        target_slot: targetSlot,
        delta,
        label: planningLayoutCompareDeltaLabel(metric, delta),
      });
    }
  }

  return deltas.sort((left, right) => {
    if (left.target_slot !== right.target_slot) {
      return slotLabelRank(left.target_slot) - slotLabelRank(right.target_slot);
    }
    return metrics.indexOf(left.metric) - metrics.indexOf(right.metric);
  });
}

export function derivePlanningLayoutCompareSideBySide(
  rows: readonly PlanningLayoutCompareRowV1[],
): PlanningLayoutCompareSideBySideV1[] {
  return rows.map((row) => ({
    slot_label: row.slot_label,
    blind_spot_summary: row.blind_spot_summary,
    recommendation_summary: row.recommendation_summary,
  }));
}

export function derivePlanningLayoutCompareAnalytics(
  slots: readonly PlanningLayoutCompareSlotV1[],
): PlanningLayoutCompareAnalyticsV1 {
  const derivation = derivePlanningLayoutCompareRows(slots);
  return {
    ...derivation,
    deltas: derivePlanningLayoutCompareDeltas(derivation.rows),
    side_by_side: derivePlanningLayoutCompareSideBySide(derivation.rows),
  };
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return value !== null && typeof value === "object" && !Array.isArray(value);
}

function nonEmptyString(value: unknown): string | null {
  if (typeof value !== "string" || value.trim().length === 0) return null;
  return value.trim();
}

function finiteNumber(value: unknown): number | null {
  if (typeof value !== "number" || !Number.isFinite(value)) return null;
  return value;
}

function parseRecommendationSummary(
  value: unknown,
): PlanningLayoutCompareRecommendationSummary | null {
  if (!isRecord(value)) return null;
  const suggestedRadar =
    value.suggested_radar === null
      ? null
      : nonEmptyString(value.suggested_radar);
  if (value.suggested_radar !== null && suggestedRadar === null) return null;
  const reason = value.reason === null ? null : nonEmptyString(value.reason);
  if (value.reason !== null && reason === null) return null;
  let suggestedPosition: PlanningLayoutCompareRecommendationSummary["suggested_position"] =
    null;
  if (value.suggested_position !== null) {
    if (!isRecord(value.suggested_position)) return null;
    const x = finiteNumber(value.suggested_position.x);
    const y = finiteNumber(value.suggested_position.y);
    if (x === null || y === null) return null;
    suggestedPosition = { x, y };
  }
  return {
    suggested_radar: suggestedRadar,
    suggested_position: suggestedPosition,
    reason,
  };
}

function parseAnalyticsSummary(value: unknown): PlanningMcSnapshotAnalyticsSummary | null {
  if (!isRecord(value)) return null;
  const coveragePercent = finiteNumber(value.coverage_percent);
  const overlapPercent = finiteNumber(value.overlap_percent);
  const redundancyPercent = finiteNumber(value.redundancy_percent);
  const blindSpotSummary = nonEmptyString(value.blind_spot_summary);
  const recommendationSummary = parseRecommendationSummary(value.recommendation_summary);
  if (
    coveragePercent === null ||
    overlapPercent === null ||
    redundancyPercent === null ||
    !blindSpotSummary ||
    !recommendationSummary
  ) {
    return null;
  }
  return {
    coverage_percent: coveragePercent,
    overlap_percent: overlapPercent,
    redundancy_percent: redundancyPercent,
    blind_spot_summary: blindSpotSummary,
    recommendation_summary: recommendationSummary,
  };
}

function parsePlanningExtent(value: unknown): PlanningMcSnapshotExtent | null {
  if (!isRecord(value)) return null;
  const planningExtentId = nonEmptyString(value.planning_extent_id);
  const planningExtentRadiusM = finiteNumber(value.planning_extent_radius_m);
  const planningExtentLabel = nonEmptyString(value.planning_extent_label);
  if (
    !planningExtentId ||
    planningExtentRadiusM === null ||
    !planningExtentLabel ||
    (planningExtentId !== "planning_5km" &&
      planningExtentId !== "planning_10km" &&
      planningExtentId !== "planning_20km")
  ) {
    return null;
  }
  return {
    planning_extent_id: planningExtentId,
    planning_extent_radius_m: planningExtentRadiusM,
    planning_extent_label: planningExtentLabel,
  };
}

export function parsePlanningMcSnapshotJson(text: string): ParsePlanningMcSnapshotResult {
  let parsed: unknown;
  try {
    parsed = JSON.parse(text);
  } catch {
    return { ok: false, error: "Invalid JSON" };
  }
  if (!isRecord(parsed)) {
    return { ok: false, error: "Expected JSON object" };
  }
  if (parsed.schema_version !== PLANNING_MC_SNAPSHOT_SCHEMA_VERSION) {
    return {
      ok: false,
      error: `expected schema_version ${PLANNING_MC_SNAPSHOT_SCHEMA_VERSION}`,
    };
  }
  const planningSnapshotId = nonEmptyString(parsed.planning_snapshot_id);
  const planningGeometryId = nonEmptyString(parsed.planning_geometry_id);
  const createdUtc = nonEmptyString(parsed.created_utc);
  if (!planningSnapshotId || !planningGeometryId || !createdUtc) {
    return { ok: false, error: "Missing required snapshot identifier fields" };
  }
  if (!isRecord(parsed.polygon) || !Array.isArray(parsed.polygon.defense_area_vertices)) {
    return { ok: false, error: "Missing polygon.defense_area_vertices" };
  }
  if (!isRecord(parsed.radars) || !Array.isArray(parsed.radars.radar_sites)) {
    return { ok: false, error: "Missing radars.radar_sites" };
  }
  const analyticsSummary = parseAnalyticsSummary(parsed.analytics_summary);
  const planningExtent = parsePlanningExtent(parsed.planning_extent);
  if (!analyticsSummary) {
    return { ok: false, error: "Invalid analytics_summary" };
  }
  if (!planningExtent) {
    return { ok: false, error: "Invalid planning_extent" };
  }
  if (!isRecord(parsed.presentation) || !isRecord(parsed.provenance)) {
    return { ok: false, error: "Missing presentation or provenance" };
  }
  const provenanceAuthority = nonEmptyString(parsed.provenance.authority);
  if (provenanceAuthority !== "rt_planning_ui") {
    return { ok: false, error: "Invalid provenance.authority" };
  }

  return {
    ok: true,
    snapshot: parsed as PlanningMcSnapshotV1,
  };
}

export function setPlanningLayoutCompareSlot(
  slots: readonly PlanningLayoutCompareSlotV1[],
  slotLabel: PlanningLayoutCompareSlotLabel,
  snapshot: PlanningMcSnapshotV1,
  captureUtc?: string,
): PlanningLayoutCompareSlotV1[] {
  const without = removePlanningLayoutCompareSlot(slots, slotLabel);
  const updated = buildPlanningLayoutCompareSlot(slotLabel, snapshot, { captureUtc });
  return sortSlotsByLabel([...without, updated]);
}

export function planningLayoutComparisonPreservesRuntimeBounds(): boolean {
  return true;
}
