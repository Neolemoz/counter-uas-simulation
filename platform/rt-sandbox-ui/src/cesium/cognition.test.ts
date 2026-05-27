import { describe, expect, it } from "vitest";
import {
  cesiumViewSummary,
  containsForbiddenLexicon,
  markerStyleForHealth,
} from "./cognition";
import { CESIUM_SCENARIO_CAVEAT } from "./constants";

describe("cesium cognition", () => {
  it("summarizes mirror view copy", () => {
    const s = cesiumViewSummary({
      pendingReconcile: true,
      entityCount: 3,
      editingEnabled: true,
    });
    expect(s.dualSurfaceNote).toContain("SVG grid");
    expect(s.caveat).toBe(CESIUM_SCENARIO_CAVEAT);
    expect(s.reconcileNote).toContain("reconcile");
    expect(s.entityCountLabel).toContain("3");
  });

  it("detects forbidden lexicon", () => {
    expect(containsForbiddenLexicon("tactical readiness score")).toBe(true);
    expect(containsForbiddenLexicon("entity mirror")).toBe(false);
  });

  it("maps health to marker style", () => {
    expect(markerStyleForHealth("ok", "ok")).toBe("ok");
    expect(markerStyleForHealth("ok", "stale")).toBe("stale");
    expect(markerStyleForHealth("mismatch", "ok")).toBe("warn");
  });
});
