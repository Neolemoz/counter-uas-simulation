import { describe, expect, it } from "vitest";
import {
  DEFAULT_PLANNING_LOCATION_PRESET_ID,
  PLANNING_LOCATION_GOVERNANCE_COPY,
  PLANNING_LOCATION_PRESETS,
  planningLocationPreset,
  validatePlanningLatitude,
  validatePlanningLongitude,
  validatedPlanningCoordinates,
} from "./planningLocations";

describe("planningLocations", () => {
  it("defines initial Planning Mode location presets", () => {
    expect(DEFAULT_PLANNING_LOCATION_PRESET_ID).toBe("bangkok");
    expect(PLANNING_LOCATION_PRESETS.map((preset) => preset.label)).toEqual([
      "Bangkok",
      "Chiang Mai",
      "Phuket",
      "Custom Coordinates",
    ]);
    expect(planningLocationPreset("chiang_mai")).toMatchObject({
      label: "Chiang Mai",
      latitudeDeg: 18.7883,
      longitudeDeg: 98.9853,
    });
  });

  it("validates custom latitude and longitude ranges", () => {
    expect(validatePlanningLatitude("13.7563")).toBe(13.7563);
    expect(validatePlanningLatitude("91")).toBeNull();
    expect(validatePlanningLatitude("abc")).toBeNull();
    expect(validatePlanningLongitude("100.5018")).toBe(100.5018);
    expect(validatePlanningLongitude("181")).toBeNull();
    expect(validatePlanningLongitude("")).toBeNull();
    expect(validatedPlanningCoordinates("7.8804", "98.3923")).toEqual({
      latitudeDeg: 7.8804,
      longitudeDeg: 98.3923,
    });
    expect(validatedPlanningCoordinates("-91", "98.3923")).toBeNull();
  });

  it("documents presentation-only governance boundaries", () => {
    expect(PLANNING_LOCATION_GOVERNANCE_COPY).toContain("presentation-only");
    expect(PLANNING_LOCATION_GOVERNANCE_COPY).toContain("runtime simulation");
    expect(PLANNING_LOCATION_GOVERNANCE_COPY).toContain("sensors");
    expect(PLANNING_LOCATION_GOVERNANCE_COPY).toContain("LOS");
    expect(PLANNING_LOCATION_GOVERNANCE_COPY).toContain("MC");
  });
});
