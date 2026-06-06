import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { DefenseZoneSvgOverlay } from "./DefenseZoneSvgOverlay";
import { PROTECTED_CENTER_ZONE_LABEL } from "@/cesium/defenseZoneVisualState";

const waypoints = [
  {
    entity_id: "center-a",
    entity_type: "waypoint_marker",
    pose: { x: 0, y: 0, z: 5 },
  },
  {
    entity_id: "wp-b",
    entity_type: "waypoint_marker",
    pose: { x: 120, y: 40, z: 5 },
  },
];

describe("DefenseZoneSvgOverlay", () => {
  it("renders candidate zones without designation emphasis", () => {
    const markup = renderToStaticMarkup(
      <DefenseZoneSvgOverlay entities={waypoints} cellSize={14} visible />,
    );
    expect(markup).toContain('data-testid="defense-zone-svg-overlay"');
    expect(markup).not.toContain('data-designated-protected-center="true"');
  });

  it("emphasizes designated protected center", () => {
    const markup = renderToStaticMarkup(
      <DefenseZoneSvgOverlay
        entities={waypoints}
        cellSize={14}
        protectedCenterEntityId="center-a"
        visible
      />,
    );
    expect(markup).toContain('data-designated-protected-center="true"');
    expect(markup).toContain('data-defense-zone-emphasis="designated"');
    expect(markup).toContain('data-defense-zone-emphasis="candidate"');
    expect(markup).toContain(PROTECTED_CENTER_ZONE_LABEL);
  });

  it("keeps non-designated waypoint visible when center is designated", () => {
    const markup = renderToStaticMarkup(
      <DefenseZoneSvgOverlay
        entities={waypoints}
        cellSize={14}
        protectedCenterEntityId="center-a"
        selectedEntityId="wp-b"
        visible
      />,
    );
    expect(markup).toContain('data-defense-zone-emphasis="candidate"');
    expect(markup).toContain('data-designated-protected-center="true"');
  });
});
