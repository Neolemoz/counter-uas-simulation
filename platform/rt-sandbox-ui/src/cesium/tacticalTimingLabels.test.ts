import { describe, expect, it } from "vitest";
import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import {
  deriveTacticalTimingSeconds,
  formatTacticalTimingBlock,
  pathLengthM,
} from "./tacticalTimingLabels";

describe("tacticalTimingLabels", () => {
  it("sums 3D path length for timing fallback", () => {
    const len = pathLengthM([
      { x: 0, y: 0, z: 0 },
      { x: 100, y: 0, z: 0 },
    ]);
    expect(len).toBeCloseTo(100, 5);
  });

  it("prefers telemetry tti_s and eta_s when present", () => {
    const state: TacticalStatePayload = {
      tti_s: 4.2,
      eta_s: 5.1,
      interceptor_speed_cap_m_s: 25,
    };
    const timing = deriveTacticalTimingSeconds(state, [
      { x: 0, y: 0, z: 0 },
      { x: 50, y: 0, z: 0 },
    ]);
    expect(timing.ttiS).toBe(4.2);
    expect(timing.etaS).toBe(5.1);
    expect(formatTacticalTimingBlock(timing)).toBe("TTI 4.2s\nETA 5.1s");
  });

  it("derives timing from path distance when fields are absent", () => {
    const timing = deriveTacticalTimingSeconds(null, [
      { x: 0, y: 0, z: 0 },
      { x: 125, y: 0, z: 0 },
    ]);
    expect(timing.ttiS).toBeCloseTo(5, 5);
    expect(timing.etaS).toBeCloseTo(5, 5);
  });
});
