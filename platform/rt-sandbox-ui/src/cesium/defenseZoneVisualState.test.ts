import { describe, expect, it } from "vitest";
import {
  isDesignatedProtectedCenter,
  resolveDefenseZoneEntityVisual,
} from "./defenseZoneVisualState";

describe("defenseZoneVisualState", () => {
  it("does not infer designation from selection alone", () => {
    expect(
      resolveDefenseZoneEntityVisual({
        entityId: "wp-a",
        selectedEntityId: "wp-a",
        protectedCenterEntityId: null,
        selectedDefensePresent: true,
        showAllDefenseZones: true,
        labelsEnabled: true,
      }).isDesignated,
    ).toBe(false);
  });

  it("emphasizes designated center over non-designated waypoints", () => {
    const designated = resolveDefenseZoneEntityVisual({
      entityId: "center-a",
      selectedEntityId: "wp-b",
      protectedCenterEntityId: "center-a",
      selectedDefensePresent: true,
      showAllDefenseZones: true,
      labelsEnabled: true,
    });
    const candidate = resolveDefenseZoneEntityVisual({
      entityId: "wp-b",
      selectedEntityId: "wp-b",
      protectedCenterEntityId: "center-a",
      selectedDefensePresent: true,
      showAllDefenseZones: true,
      labelsEnabled: true,
    });

    expect(designated.isDesignated).toBe(true);
    expect(candidate.isDesignated).toBe(false);
    expect(designated.fade).toBeGreaterThan(candidate.fade);
    expect(designated.emphasis).toBeGreaterThan(candidate.emphasis);
    expect(designated.showZoneLabels).toBe(true);
  });

  it("supports selection and designation on different entities", () => {
    const designatedSelected = resolveDefenseZoneEntityVisual({
      entityId: "center-a",
      selectedEntityId: "center-a",
      protectedCenterEntityId: "center-a",
      selectedDefensePresent: true,
      showAllDefenseZones: true,
      labelsEnabled: true,
    });
    expect(designatedSelected.isDesignated).toBe(true);
    expect(designatedSelected.isSelected).toBe(true);
    expect(designatedSelected.emphasis).toBeGreaterThan(1.5);
  });

  it("keeps legacy fade when no protected center is designated", () => {
    const unselected = resolveDefenseZoneEntityVisual({
      entityId: "wp-a",
      selectedEntityId: "wp-b",
      protectedCenterEntityId: null,
      selectedDefensePresent: true,
      showAllDefenseZones: true,
      labelsEnabled: true,
    });
    expect(unselected.fade).toBe(0.34);
    expect(unselected.emphasis).toBe(1);
  });

  it("identifies designated entity id explicitly", () => {
    expect(isDesignatedProtectedCenter("a", "a")).toBe(true);
    expect(isDesignatedProtectedCenter("b", "a")).toBe(false);
    expect(isDesignatedProtectedCenter("a", null)).toBe(false);
  });
});
