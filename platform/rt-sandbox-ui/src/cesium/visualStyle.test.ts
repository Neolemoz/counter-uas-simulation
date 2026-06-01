import { describe, expect, it } from "vitest";
import { labelFontSizePx, labelText, markerPixelSize, shortEntityId } from "@/cesium/visualStyle";

describe("visualStyle", () => {
  it("shortens entity ids", () => {
    expect(shortEntityId("abcdefgh-1234")).toBe("abcdefgh");
    expect(shortEntityId("ab")).toBe("ab");
  });

  it("builds label with type and id", () => {
    expect(labelText("drone", "entity-uuid-99")).toContain("Drone");
    expect(labelText("drone", "entity-uuid-99")).toContain("entity-u");
  });

  it("scales marker size when selected", () => {
    expect(markerPixelSize(true)).toBeGreaterThan(markerPixelSize(false));
  });

  it("scales label font with camera distance", () => {
    expect(labelFontSizePx(5000, false, false)).toBeLessThan(labelFontSizePx(600, true, false));
  });
});
