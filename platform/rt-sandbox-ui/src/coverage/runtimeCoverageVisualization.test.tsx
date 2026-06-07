import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import type { MirrorEntity } from "@/cesium/entityMarkers";
import {
  CANONICAL_VISUAL_LAYER_REGISTRY,
  defaultVisibilityFromRegistry,
  layerIdToVisibilityKey,
} from "@/cesium/visualLayerRegistry";
import { BANNER_RUNTIME_COVERAGE } from "@/governance/banners";
import { deriveCorridorCoveragePolylines } from "./runtimeCorridorCoverage";
import {
  RUNTIME_COVERAGE_ENTITY_PREFIX,
  clearRuntimeCoverageLayer,
  syncRuntimeCoverageLayer,
} from "./runtimeCoverageLayer";
import {
  deriveRuntimeCoverageRenderModel,
  plannedRuntimeCoverageEntityIds,
  runtimeCoverageLayerRegistered,
} from "./runtimeCoverageRenderModel";
import {
  RuntimeCoverageSvgOverlay,
  RuntimeCoverageSvgOverlayView,
} from "./RuntimeCoverageSvgOverlay";

const fixturePath = join(
  dirname(fileURLToPath(import.meta.url)),
  "../../../../fixtures/rt_sandbox/runtime_coverage_golden_v1.json",
);

const centerEntity: MirrorEntity = {
  entity_id: "center-1",
  entity_type: "waypoint_marker",
  pose: { x: 0, y: 0, z: 10 },
};

const radarAtCenter: MirrorEntity = {
  entity_id: "radar-a",
  entity_type: "radar",
  pose: { x: 0, y: 0, z: 10 },
};

const radarOffset: MirrorEntity = {
  entity_id: "radar-b",
  entity_type: "radar",
  pose: { x: -250, y: 0, z: 10 },
};

function loadGoldenCase(caseId: string) {
  const fixture = JSON.parse(readFileSync(fixturePath, "utf8")) as {
    cases: Record<
      string,
      {
        entities: MirrorEntity[];
        corridor_points?: Array<{ x: number; y: number; z: number }>;
        radar_dome_config?: { detectionM: number };
      }
    >;
  };
  return fixture.cases[caseId];
}

describe("runtime coverage visualization", () => {
  it("registers runtime_coverage_cells in the visual layer registry", () => {
    expect(
      runtimeCoverageLayerRegistered(CANONICAL_VISUAL_LAYER_REGISTRY.layers),
    ).toBe(true);
    expect(layerIdToVisibilityKey("runtime_coverage_cells")).toBe(
      "showRuntimeCoverageCells",
    );
    expect(defaultVisibilityFromRegistry().showRuntimeCoverageCells).toBe(false);
  });

  it("plans stable rt-runtime-coverage entity ids", () => {
    const model = deriveRuntimeCoverageRenderModel({
      entities: [centerEntity, radarOffset],
      protectedCenterEntityId: "center-1",
      radarDomeConfig: { detectionM: 120 },
    });
    expect(model.ready).toBe(true);
    if (!model.ready) return;

    const ids = plannedRuntimeCoverageEntityIds(model);
    expect(ids.length).toBeGreaterThan(0);
    expect(ids.every((id) => id.startsWith(RUNTIME_COVERAGE_ENTITY_PREFIX))).toBe(true);
    expect(ids).toContain(`${RUNTIME_COVERAGE_ENTITY_PREFIX}covered-0`);
    expect(ids.some((id) => id.includes("blind-spot"))).toBe(true);
  });

  it("includes blind spot hints and sector labels when coverage gaps exist", () => {
    const model = deriveRuntimeCoverageRenderModel({
      entities: [centerEntity, radarOffset],
      protectedCenterEntityId: "center-1",
      radarDomeConfig: { detectionM: 120 },
    });
    expect(model.ready).toBe(true);
    if (!model.ready) return;

    expect(model.blindSpotHints.length).toBeGreaterThan(0);
    expect(model.majorUncoveredSectors.length).toBeGreaterThan(0);
    const ids = plannedRuntimeCoverageEntityIds(model);
    expect(ids.some((id) => id.includes("sector-label"))).toBe(true);
  });

  it("does not render svg overlay when protected center is unavailable", () => {
    const markup = renderToStaticMarkup(
      <RuntimeCoverageSvgOverlay
        entities={[radarAtCenter]}
        cellSize={24}
        protectedCenterEntityId={null}
        visible
      />,
    );
    expect(markup).not.toContain("runtime-coverage-svg-overlay");
  });

  it("renders svg parity for covered and uncovered cells", () => {
    const model = deriveRuntimeCoverageRenderModel({
      entities: [centerEntity, radarAtCenter],
      protectedCenterEntityId: "center-1",
      radarDomeConfig: { detectionM: 300 },
    });
    expect(model.ready).toBe(true);
    if (!model.ready) return;

    const markup = renderToStaticMarkup(
      <RuntimeCoverageSvgOverlayView model={model} cellSize={24} />,
    );
    expect(markup).toContain('data-testid="runtime-coverage-svg-overlay"');
    expect(markup).toContain('data-testid="runtime-coverage-svg-covered"');
    expect(markup).toContain('data-testid="runtime-coverage-svg-uncovered"');
  });

  it("derives corridor covered and uncovered polylines for half-covered golden case", () => {
    const golden = loadGoldenCase("half_covered_corridor");
    const polylines = deriveCorridorCoveragePolylines(
      golden.corridor_points!,
      [{ id: "radar-a", position: { x: 250, y: 0 }, detectionRangeM: 250 }],
    );
    expect(polylines.coveredPolylines.length).toBeGreaterThan(0);
    expect(polylines.uncoveredPolylines.length).toBeGreaterThan(0);
  });

  it("clears runtime coverage entities when layer sync is disabled", () => {
    const removed: string[] = [];
    const viewer = {
      entities: {
        values: [
          { id: `${RUNTIME_COVERAGE_ENTITY_PREFIX}covered-0` },
          { id: "other-entity" },
        ],
        remove: (entity: { id?: string }) => {
          if (entity.id) removed.push(entity.id);
        },
      },
      isDestroyed: () => false,
      scene: { canvas: {}, globe: { ellipsoid: {} } },
      camera: {},
    } as unknown as import("cesium").Viewer;

    syncRuntimeCoverageLayer(viewer, {
      enabled: false,
      params: {
        entities: [centerEntity, radarAtCenter],
        protectedCenterEntityId: "center-1",
      },
    });

    expect(removed).toEqual([`${RUNTIME_COVERAGE_ENTITY_PREFIX}covered-0`]);
    clearRuntimeCoverageLayer(viewer);
  });

  it("exposes governance banner copy for runtime coverage", () => {
    expect(BANNER_RUNTIME_COVERAGE).toContain("heuristic");
    expect(BANNER_RUNTIME_COVERAGE.toLowerCase()).toContain("geometry");
    expect(BANNER_RUNTIME_COVERAGE).toContain("not sensor truth");
    expect(BANNER_RUNTIME_COVERAGE).toContain("detection probability");
  });
});
