import { describe, expect, it, vi } from "vitest";
import {
  EMPTY_PLANNING_RADARS,
  PLANNING_RADAR_PRESETS,
  type PlanningPolygonState,
  type PlanningRadarState,
} from "@/cesium/planningDrawing";
import { analyzePlanningCoverage } from "@/cesium/planningCoverageAnalysis";
import { buildPlanningMcSnapshot } from "./planningMcSnapshot";
import { buildPlanningMcPackage, exportPlanningMcPackageJson } from "./planningMcPackage";
import {
  buildMockPlanningMcResultRef,
  buildPlanningResultLink,
  buildPlanningResultLinkPreview,
  exportPlanningMcResultRefJson,
  parsePlanningMcResultRefJson,
  validatePlanningResultLink,
} from "./planningMcResultLink";

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

function packageRow() {
  return buildPlanningMcPackage(
    buildPlanningMcSnapshot(
      POLYGON,
      RADARS,
      analyzePlanningCoverage(POLYGON, RADARS, 4, {
        radarPresets: PLANNING_RADAR_PRESETS,
      }),
      {
        createdUtc: "2026-06-04T00:00:00Z",
        terrainMode: "ellipsoid",
        selectedLocationPreset: "bangkok",
        sourceLayoutId: "rt_layout_source",
        sourceGeometryId: "rt_layout:sha256:source",
      },
    ),
  );
}

describe("planningMcResultLink", () => {
  it("validates linked metadata when identifiers match", () => {
    const pkg = packageRow();
    const ref = buildMockPlanningMcResultRef(pkg, {
      importedUtc: "2026-06-04T12:00:00Z",
      summary: { success_rate: 0.75 },
    });

    expect(validatePlanningResultLink(pkg, ref, pkg.planning_geometry_id)).toEqual({
      status: "linked",
      reasons: [],
    });
    expect(buildPlanningResultLink(pkg, ref, pkg.planning_geometry_id).status).toBe("linked");
  });

  it("detects stale linkage when planning geometry drifts", () => {
    const pkg = packageRow();
    const ref = buildMockPlanningMcResultRef(pkg, { importedUtc: "2026-06-04T12:00:00Z" });
    const driftedGeometryId = "rt_planning:sha256:drifted";

    expect(validatePlanningResultLink(pkg, ref, driftedGeometryId)).toEqual({
      status: "stale",
      reasons: ["Planning geometry changed since result import"],
    });
    expect(buildPlanningResultLink(pkg, ref, driftedGeometryId).status).toBe("stale");
  });

  it("detects mismatched identifiers between package and result ref", () => {
    const pkg = packageRow();
    const ref = buildMockPlanningMcResultRef(pkg, {
      importedUtc: "2026-06-04T12:00:00Z",
    });
    ref.linked_package_id = "rt_planning_package:wrong";
    ref.linked_planning_geometry_id = "rt_planning:sha256:wrong";

    expect(validatePlanningResultLink(pkg, ref, pkg.planning_geometry_id)).toEqual({
      status: "mismatch",
      reasons: [
        "linked_package_id does not match current package",
        "linked_planning_geometry_id does not match package geometry",
      ],
    });
  });

  it("parses pasted metadata JSON with optional summaries", () => {
    const pkg = packageRow();
    const ref = buildMockPlanningMcResultRef(pkg, {
      importedUtc: "2026-06-04T12:00:00Z",
      summary: {
        success_rate: 0.9,
        miss_distance_p95: 12.5,
        intercept_time_mean: 9.1,
      },
    });
    const parsed = parsePlanningMcResultRefJson(exportPlanningMcResultRefJson(ref));

    expect(parsed.ok).toBe(true);
    if (parsed.ok) {
      expect(parsed.ref).toEqual(ref);
    }
  });

  it("rejects invalid metadata import payloads", () => {
    expect(parsePlanningMcResultRefJson("{")).toEqual({ ok: false, error: "Invalid JSON" });
    expect(parsePlanningMcResultRefJson(JSON.stringify({ schema_version: "wrong" }))).toEqual({
      ok: false,
      error: "expected schema_version rt_planning_mc_result_ref_v1",
    });
  });

  it("builds read-only preview rows without comparison logic", () => {
    const pkg = packageRow();
    const link = buildPlanningResultLink(
      pkg,
      buildMockPlanningMcResultRef(pkg, {
        importedUtc: "2026-06-04T12:00:00Z",
        summary: { success_rate: 0.5, miss_distance_p95: 20, intercept_time_mean: 15 },
      }),
      pkg.planning_geometry_id,
    );
    const preview = buildPlanningResultLinkPreview(link);

    expect(preview.statusLabel).toBe("Linked");
    expect(preview.mcResultId).toBe("rt_mc_result:mock:planning-ui");
    expect(preview.successRate).toBe(0.5);
    expect(preview.missDistanceP95).toBe(20);
    expect(preview.interceptTimeMean).toBe(15);
  });

  it("does not invoke runtime, bridge, MC execution, or filesystem access", () => {
    const bridgeCommand = vi.fn();
    const runtimeMutation = vi.fn();
    const monteCarloExecution = vi.fn();
    const readFile = vi.fn();
    const pkg = packageRow();
    const ref = buildMockPlanningMcResultRef(pkg);
    const link = buildPlanningResultLink(pkg, ref, pkg.planning_geometry_id);

    expect(link.status).toBe("linked");
    expect(JSON.parse(exportPlanningMcResultRefJson(ref)).schema_version).toBe(
      "rt_planning_mc_result_ref_v1",
    );
    expect(JSON.parse(exportPlanningMcPackageJson(pkg)).schema_version).toBe(
      "rt_planning_mc_package_v1",
    );
    expect(bridgeCommand).not.toHaveBeenCalled();
    expect(runtimeMutation).not.toHaveBeenCalled();
    expect(monteCarloExecution).not.toHaveBeenCalled();
    expect(readFile).not.toHaveBeenCalled();
  });
});
