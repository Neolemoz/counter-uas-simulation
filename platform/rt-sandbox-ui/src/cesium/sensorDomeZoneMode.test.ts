import { describe, expect, it } from "vitest";
import { shouldShowDefenseZones, shouldShowRadarZones } from "./sensorDomeZoneMode";

describe("sensorDomeZoneMode terrain gating", () => {
  it("gates radar and defense independently", () => {
    expect(shouldShowRadarZones("radar")).toBe(true);
    expect(shouldShowDefenseZones("radar")).toBe(false);
    expect(shouldShowRadarZones("defense")).toBe(false);
    expect(shouldShowDefenseZones("defense")).toBe(true);
    expect(shouldShowRadarZones("both")).toBe(true);
    expect(shouldShowDefenseZones("both")).toBe(true);
  });
});
