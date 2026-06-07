import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import type { MirrorEntity } from "@/cesium/entityMarkers";
import { DEFAULT_DEFENSE_ZONE_CONFIG } from "@/cesium/defenseZoneConfig";
import { BANNER_RUNTIME_COVERAGE } from "@/governance/banners";
import { deriveRuntimeCoverageStatus } from "./runtimeCoverageSelectors";
import {
  RuntimeCoverageStatusStrip,
  RuntimeCoverageStatusStripView,
} from "./RuntimeCoverageStatusStrip";

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

describe("deriveRuntimeCoverageStatus", () => {
  it("reports covered center when radar overlaps protected center", () => {
    const model = deriveRuntimeCoverageStatus({
      entities: [centerEntity, radarAtCenter],
      protectedCenterEntityId: "center-1",
      radarDomeConfig: { detectionM: 300 },
    });

    expect(model.availability).toBe("ready");
    expect(model.protectedCenterCovered).toBe("yes");
    expect(model.coveringRadarCount).toBe(1);
    expect(model.nearestEdgeDistanceM).toBe(0);
    expect(model.coveragePercent).toBeGreaterThan(90);
  });

  it("reports uncovered center when no radar covers protected center", () => {
    const model = deriveRuntimeCoverageStatus({
      entities: [centerEntity, radarOffset],
      protectedCenterEntityId: "center-1",
      radarDomeConfig: { detectionM: 120 },
    });

    expect(model.protectedCenterCovered).toBe("no");
    expect(model.coveringRadarCount).toBe(0);
    expect(model.nearestEdgeDistanceM).toBeGreaterThan(0);
    expect(model.topBlindSpotSector).not.toBeNull();
  });

  it("fails closed when protected center is missing", () => {
    const model = deriveRuntimeCoverageStatus({
      entities: [radarAtCenter],
      protectedCenterEntityId: null,
    });

    expect(model.availability).toBe("protected_center_unavailable");
    expect(model.protectedCenterCovered).toBe("unavailable");
    expect(model.coveringRadarCount).toBeNull();
    expect(model.coveragePercent).toBeNull();
    expect(model.guidance).toContain("Designate a protected center");
  });

  it("handles no radar entities with zero coverage guidance", () => {
    const model = deriveRuntimeCoverageStatus({
      entities: [centerEntity],
      protectedCenterEntityId: "center-1",
    });

    expect(model.availability).toBe("ready");
    expect(model.protectedCenterCovered).toBe("no");
    expect(model.coveringRadarCount).toBe(0);
    expect(model.coveragePercent).toBe(0);
    expect(model.nearestEdgeDistanceM).toBeNull();
    expect(model.guidance).toContain("No runtime radar entities");
  });

  it("leaves corridor uncovered percent null when corridor is absent", () => {
    const model = deriveRuntimeCoverageStatus({
      entities: [centerEntity, radarAtCenter],
      protectedCenterEntityId: "center-1",
      tacticalState: null,
    });

    expect(model.corridorUncoveredPercent).toBeNull();
  });

  it("rejects rectangle defense shape", () => {
    const model = deriveRuntimeCoverageStatus({
      entities: [centerEntity, radarAtCenter],
      protectedCenterEntityId: "center-1",
      defenseZoneConfig: { ...DEFAULT_DEFENSE_ZONE_CONFIG, shape: "rectangle" },
    });

    expect(model.availability).toBe("unsupported_defense_shape");
    expect(model.protectedCenterCovered).toBe("unavailable");
    expect(model.guidance).toContain("circle defense zones");
  });
});

describe("RuntimeCoverageStatusStrip", () => {
  it("renders governance banner and covered center chips", () => {
    const model = deriveRuntimeCoverageStatus({
      entities: [centerEntity, radarAtCenter],
      protectedCenterEntityId: "center-1",
      radarDomeConfig: { detectionM: 300 },
    });
    const markup = renderToStaticMarkup(<RuntimeCoverageStatusStripView model={model} />);

    expect(markup).toContain('data-testid="runtime-coverage-status-strip"');
    expect(markup).toContain('data-testid="runtime-coverage-banner"');
    expect(markup).toContain(BANNER_RUNTIME_COVERAGE);
    expect(markup).toContain("heuristic 2D geometry only");
    expect(markup).toContain("not sensor truth");
    expect(markup).toContain("detection probability");
    expect(markup).toContain('data-testid="runtime-coverage-center-covered"');
    expect(markup).toContain("Center covered: Yes");
  });

  it("renders unavailable center state", () => {
    const markup = renderToStaticMarkup(
      <RuntimeCoverageStatusStrip
        entities={[]}
        protectedCenterEntityId={null}
      />,
    );

    expect(markup).toContain("Center covered: Unavailable");
    expect(markup).toContain('data-testid="runtime-coverage-guidance"');
    expect(markup).toContain("Designate a protected center");
  });

  it("renders corridor absent as em dash", () => {
    const model = deriveRuntimeCoverageStatus({
      entities: [centerEntity, radarAtCenter],
      protectedCenterEntityId: "center-1",
      tacticalState: null,
    });
    const markup = renderToStaticMarkup(<RuntimeCoverageStatusStripView model={model} />);

    expect(markup).toContain('data-testid="runtime-coverage-corridor"');
    expect(markup).toContain("Corridor uncovered: —");
  });

  it("renders no-radar guidance and zero covering count", () => {
    const markup = renderToStaticMarkup(
      <RuntimeCoverageStatusStrip
        entities={[centerEntity]}
        protectedCenterEntityId="center-1"
      />,
    );

    expect(markup).toContain("Covering radars: 0");
    expect(markup).toContain("Center covered: No");
    expect(markup).toContain("No runtime radar entities");
  });
});
