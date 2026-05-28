import { describe, expect, it } from "vitest";
import {
  clearHorizonHintLayer,
  horizonRingVertexCount,
  syncHorizonHintLayer,
} from "./horizonHintLayer";

describe("horizonHintLayer", () => {
  it("uses closed ring with 5 vertices", () => {
    expect(horizonRingVertexCount()).toBe(5);
  });

  it("noops on null viewer", () => {
    expect(() => syncHorizonHintLayer(null, true)).not.toThrow();
    expect(() => clearHorizonHintLayer(null)).not.toThrow();
  });
});
