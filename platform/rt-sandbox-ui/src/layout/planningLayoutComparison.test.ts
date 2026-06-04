import { describe, expect, it, vi } from "vitest";
import {
  EMPTY_PLANNING_RADARS,
  PLANNING_RADAR_PRESETS,
  type PlanningPolygonState,
  type PlanningRadarState,
} from "@/cesium/planningDrawing";
import { analyzePlanningCoverage } from "@/cesium/planningCoverageAnalysis";
import { planningExtentById } from "@/cesium/planningWorld";
import { buildPlanningMcSnapshot } from "./planningMcSnapshot";
import { buildPlanningMcPackage } from "./planningMcPackage";
import {
  buildMockPlanningMcResultRef,
  buildPlanningResultLink,
} from "./planningMcResultLink";
import {
  buildPlanningLayoutCompareSlot,
  capturePlanningLayoutCompareSlot,
  clearPlanningLayoutCompareSlots,
  derivePlanningLayoutCompareAnalytics,
  derivePlanningLayoutCompareRows,
  parsePlanningMcSnapshotJson,
  planningLayoutCompareDeltaLabel,
  planningLayoutComparisonPreservesRuntimeBounds,
  removePlanningLayoutCompareSlot,
  setPlanningLayoutCompareSlot,
} from "./planningLayoutComparison";
import { exportPlanningMcSnapshotJson } from "./planningMcSnapshot";

const POLYGON: PlanningPolygonState = {
  draftVertices: [],
  completedVertices: [
    { x: 0, y: 0 },
    { x: 1000, y: 0 },
    { x: 1000, y: 1000 },
    { x: 0, y: 1000 },
  ],
};

const RADARS: PlanningRadarState = {
  ...EMPTY_PLANNING_RADARS,
  sites: [
    {
      id: "planning-radar-1",
      position: { x: 150, y: 150 },
      radar_type: "Short Range",
      detection_range_m: 500,
    },
  ],
};

const ALT_RADARS: PlanningRadarState = {
  ...EMPTY_PLANNING_RADARS,
  sites: [
    {
      id: "planning-radar-2",
      position: { x: 800, y: 800 },
      radar_type: "Medium Range",
      detection_range_m: 1000,
    },
  ],
};

function snapshotFor(
  radars: PlanningRadarState,
  extentId: "planning_5km" | "planning_10km" | "planning_20km" = "planning_10km",
  createdUtc = "2026-06-04T00:00:00Z",
) {
  const polygon = POLYGON;
  return buildPlanningMcSnapshot(
    polygon,
    radars,
    analyzePlanningCoverage(polygon, radars, 4, { radarPresets: PLANNING_RADAR_PRESETS }),
    {
      createdUtc,
      terrainMode: "ellipsoid",
      selectedLocationPreset: "bangkok",
      planningExtent: planningExtentById(extentId),
    },
  );
}

describe("planningLayoutComparison", () => {
  it("derives deterministic compare rows from snapshot slots", () => {
    const snapA = snapshotFor(RADARS);
    const snapB = snapshotFor(ALT_RADARS);
    const slots = [
      buildPlanningLayoutCompareSlot("A", snapA, { captureUtc: "2026-06-04T01:00:00Z" }),
      buildPlanningLayoutCompareSlot("B", snapB, { captureUtc: "2026-06-04T02:00:00Z" }),
    ];

    const first = derivePlanningLayoutCompareRows(slots);
    const second = derivePlanningLayoutCompareRows(slots);

    expect(second).toEqual(first);
    expect(first.rows).toHaveLength(2);
    expect(first.rows[0]).toMatchObject({
      slot_label: "A",
      planning_snapshot_id: snapA.planning_snapshot_id,
      planning_geometry_id: snapA.planning_geometry_id,
      planning_extent_id: "planning_10km",
      radar_count: 1,
      coverage_percent: snapA.analytics_summary.coverage_percent,
      overlap_percent: snapA.analytics_summary.overlap_percent,
      redundancy_percent: snapA.analytics_summary.redundancy_percent,
      blind_spot_summary: snapA.analytics_summary.blind_spot_summary,
      recommendation_summary: snapA.analytics_summary.recommendation_summary,
      extent: snapA.planning_extent,
      mc_link_status: null,
    });
    expect(first.rows[1].planning_geometry_id).not.toBe(first.rows[0].planning_geometry_id);
    expect(first.warnings).toEqual([]);
  });

  it("orders rows by slot label regardless of input order", () => {
    const snapA = snapshotFor(RADARS);
    const snapB = snapshotFor(ALT_RADARS);
    const snapC = snapshotFor(ALT_RADARS, "planning_20km", "2026-06-04T03:00:00Z");

    const result = derivePlanningLayoutCompareRows([
      buildPlanningLayoutCompareSlot("C", snapC),
      buildPlanningLayoutCompareSlot("A", snapA),
      buildPlanningLayoutCompareSlot("B", snapB),
    ]);

    expect(result.rows.map((row) => row.slot_label)).toEqual(["A", "B", "C"]);
  });

  it("detects duplicate geometry across slots with advisory warnings only", () => {
    const snapA = snapshotFor(RADARS, "planning_10km", "2026-06-04T00:00:00Z");
    const snapB = snapshotFor(RADARS, "planning_10km", "2026-06-04T01:00:00Z");

    expect(snapA.planning_geometry_id).toBe(snapB.planning_geometry_id);
    expect(snapA.planning_snapshot_id).not.toBe(snapB.planning_snapshot_id);

    const result = derivePlanningLayoutCompareRows([
      buildPlanningLayoutCompareSlot("A", snapA),
      buildPlanningLayoutCompareSlot("B", snapB),
    ]);

    expect(result.warnings).toEqual([
      {
        warning_id: "duplicate_geometry",
        message:
          "slots A and B share the same planning_geometry_id (identical layout geometry).",
        slot_labels: ["A", "B"],
      },
    ]);
    expect(result.rows).toHaveLength(2);
    expect(result.rows[0]?.warnings).toEqual(result.warnings);
    expect(result.rows[1]?.warnings).toEqual(result.warnings);
  });

  it("detects extent mismatch with compatibility warnings and still compares", () => {
    const snapA = snapshotFor(RADARS, "planning_5km");
    const snapB = snapshotFor(ALT_RADARS, "planning_20km");

    const result = derivePlanningLayoutCompareRows([
      buildPlanningLayoutCompareSlot("A", snapA),
      buildPlanningLayoutCompareSlot("B", snapB),
    ]);

    expect(result.rows).toHaveLength(2);
    expect(result.rows[0]?.planning_extent_id).toBe("planning_5km");
    expect(result.rows[1]?.planning_extent_id).toBe("planning_20km");
    expect(result.warnings).toEqual([
      {
        warning_id: "extent_mismatch",
        message:
          "slots A and B use different planning_extent_id values (planning_20km (slot B); planning_5km (slot A)). Metrics remain heuristic and are not directly comparable across Planning World extents.",
        slot_labels: ["A", "B"],
      },
    ]);
  });

  it("propagates MC link status only without analytics or scoring", () => {
    const snap = snapshotFor(RADARS);
    const pkg = buildPlanningMcPackage(snap);
    const linkedRef = buildMockPlanningMcResultRef(pkg, {
      importedUtc: "2026-06-04T12:00:00Z",
      summary: { success_rate: 0.95, miss_distance_p95: 10, intercept_time_mean: 8 },
    });
    const linked = buildPlanningResultLink(pkg, linkedRef, snap.planning_geometry_id);
    const stale = buildPlanningResultLink(
      pkg,
      linkedRef,
      "rt_planning:sha256:drifted",
    );
    const mismatchRef = buildMockPlanningMcResultRef(pkg, {
      importedUtc: "2026-06-04T12:00:00Z",
    });
    mismatchRef.linked_package_id = "rt_planning_package:wrong";
    mismatchRef.linked_planning_geometry_id = "rt_planning:sha256:wrong";
    const mismatch = buildPlanningResultLink(
      pkg,
      mismatchRef,
      snap.planning_geometry_id,
    );

    const linkedRow = derivePlanningLayoutCompareRows([
      buildPlanningLayoutCompareSlot("A", snap, { resultLink: linked }),
    ]).rows[0];
    const staleRow = derivePlanningLayoutCompareRows([
      buildPlanningLayoutCompareSlot("B", snap, { resultLink: stale }),
    ]).rows[0];
    const mismatchRow = derivePlanningLayoutCompareRows([
      buildPlanningLayoutCompareSlot("C", snap, { resultLink: mismatch }),
    ]).rows[0];
    const unlinkedRow = derivePlanningLayoutCompareRows([
      buildPlanningLayoutCompareSlot("A", snap, {
        resultLink: buildPlanningResultLink(pkg, null, snap.planning_geometry_id),
      }),
    ]).rows[0];

    expect(linkedRow?.mc_link_status).toBe("linked");
    expect(staleRow?.mc_link_status).toBe("stale");
    expect(mismatchRow?.mc_link_status).toBe("mismatch");
    expect(unlinkedRow?.mc_link_status).toBe("unlinked");
    expect(linkedRow).not.toHaveProperty("success_rate");
    expect(linkedRow).not.toHaveProperty("miss_distance_p95");
    expect(linkedRow).not.toHaveProperty("intercept_time_mean");
  });

  it("preserves runtime bounds and does not invoke bridge or MC execution", () => {
    const bridgeCommand = vi.fn();
    const runtimeMutation = vi.fn();
    const monteCarloExecution = vi.fn();

    const result = derivePlanningLayoutCompareRows([
      buildPlanningLayoutCompareSlot("A", snapshotFor(RADARS)),
    ]);

    expect(planningLayoutComparisonPreservesRuntimeBounds()).toBe(true);
    expect(result.rows).toHaveLength(1);
    expect(bridgeCommand).not.toHaveBeenCalled();
    expect(runtimeMutation).not.toHaveBeenCalled();
    expect(monteCarloExecution).not.toHaveBeenCalled();
  });

  it("captures snapshots into next available slots up to three", () => {
    const snapA = snapshotFor(RADARS);
    const snapB = snapshotFor(ALT_RADARS);
    const snapC = snapshotFor(ALT_RADARS, "planning_20km", "2026-06-04T02:00:00Z");

    const first = capturePlanningLayoutCompareSlot([], snapA, "2026-06-04T00:00:00Z");
    expect(first.captured?.slot_label).toBe("A");
    expect(first.slots).toHaveLength(1);

    const second = capturePlanningLayoutCompareSlot(first.slots, snapB, "2026-06-04T01:00:00Z");
    expect(second.captured?.slot_label).toBe("B");
    expect(second.slots.map((slot) => slot.slot_label)).toEqual(["A", "B"]);

    const third = capturePlanningLayoutCompareSlot(second.slots, snapC, "2026-06-04T02:00:00Z");
    expect(third.captured?.slot_label).toBe("C");
    expect(third.slots).toHaveLength(3);

    const full = capturePlanningLayoutCompareSlot(third.slots, snapA, "2026-06-04T03:00:00Z");
    expect(full.captured).toBeNull();
    expect(full.slots).toEqual(third.slots);
  });

  it("removes a slot without mutating other captured snapshots", () => {
    const snapA = snapshotFor(RADARS);
    const snapB = snapshotFor(ALT_RADARS);
    const captured = capturePlanningLayoutCompareSlot([], snapA).slots;
    const both = capturePlanningLayoutCompareSlot(captured, snapB).slots;
    const removed = removePlanningLayoutCompareSlot(both, "A");

    expect(removed).toHaveLength(1);
    expect(removed[0]?.slot_label).toBe("B");
    expect(removed[0]?.planning_snapshot_id).toBe(snapB.planning_snapshot_id);
    expect(both).toHaveLength(2);
  });

  it("clears all compare slots", () => {
    const slots = capturePlanningLayoutCompareSlot([], snapshotFor(RADARS)).slots;
    expect(clearPlanningLayoutCompareSlots()).toEqual([]);
    expect(slots).toHaveLength(1);
  });

  it("keeps captured slot snapshots immutable when source geometry changes", () => {
    const snapA = snapshotFor(RADARS);
    const captured = capturePlanningLayoutCompareSlot([], snapA, "2026-06-04T00:00:00Z");
    const drifted = snapshotFor(ALT_RADARS);

    expect(captured.slots[0]?.planning_geometry_id).toBe(snapA.planning_geometry_id);
    expect(drifted.planning_geometry_id).not.toBe(snapA.planning_geometry_id);
    expect(captured.slots[0]?.planning_geometry_id).not.toBe(drifted.planning_geometry_id);
  });

  it("derives deterministic deltas and advisory labels vs slot A", () => {
    const slots = [
      buildPlanningLayoutCompareSlot("A", snapshotFor(RADARS)),
      buildPlanningLayoutCompareSlot("B", snapshotFor(ALT_RADARS)),
    ];
    const first = derivePlanningLayoutCompareAnalytics(slots);
    const second = derivePlanningLayoutCompareAnalytics(slots);

    expect(second).toEqual(first);
    expect(first.deltas).toEqual(
      expect.arrayContaining([
        expect.objectContaining({
          metric: "coverage_percent",
          target_slot: "B",
          baseline_slot: "A",
          label: expect.stringMatching(/^(improved|reduced|unchanged)$/),
        }),
      ]),
    );
    const coverageDelta = first.deltas.find(
      (row) => row.metric === "coverage_percent" && row.target_slot === "B",
    );
    expect(coverageDelta?.label).toBe(
      planningLayoutCompareDeltaLabel("coverage_percent", coverageDelta!.delta),
    );
  });

  it("labels percent deltas unchanged within epsilon and radar count exactly", () => {
    expect(planningLayoutCompareDeltaLabel("coverage_percent", 0.05)).toBe("unchanged");
    expect(planningLayoutCompareDeltaLabel("coverage_percent", 0.2)).toBe("improved");
    expect(planningLayoutCompareDeltaLabel("overlap_percent", -0.2)).toBe("reduced");
    expect(planningLayoutCompareDeltaLabel("radar_count", 0)).toBe("unchanged");
    expect(planningLayoutCompareDeltaLabel("radar_count", 1)).toBe("improved");
  });

  it("parses valid snapshot JSON and rejects invalid payloads", () => {
    const snap = snapshotFor(RADARS);
    const parsed = parsePlanningMcSnapshotJson(exportPlanningMcSnapshotJson(snap));
    expect(parsed.ok).toBe(true);
    if (parsed.ok) {
      expect(parsed.snapshot.planning_snapshot_id).toBe(snap.planning_snapshot_id);
    }
    expect(parsePlanningMcSnapshotJson("{")).toEqual({ ok: false, error: "Invalid JSON" });
    expect(parsePlanningMcSnapshotJson(JSON.stringify({ schema_version: "wrong" }))).toEqual({
      ok: false,
      error: "expected schema_version rt_planning_mc_snapshot_v1",
    });
  });

  it("imports snapshots into selected slots without mutating other captures", () => {
    const snapA = snapshotFor(RADARS);
    const snapB = snapshotFor(ALT_RADARS);
    const initial = capturePlanningLayoutCompareSlot([], snapA).slots;
    const imported = setPlanningLayoutCompareSlot(initial, "B", snapB, "2026-06-04T01:00:00Z");

    expect(imported).toHaveLength(2);
    expect(imported[0]?.planning_snapshot_id).toBe(snapA.planning_snapshot_id);
    expect(imported[1]?.planning_snapshot_id).toBe(snapB.planning_snapshot_id);
  });
});
