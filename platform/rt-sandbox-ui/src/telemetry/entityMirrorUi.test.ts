import { describe, expect, it } from "vitest";
import {
  compactTargetStateChip,
  runtimeStateChipLabel,
  runtimeStateChipTone,
} from "./entityMirrorUi";

describe("entityMirrorUi", () => {
  it("normalizes state labels for chips", () => {
    expect(runtimeStateChipLabel("assigned")).toBe("ASSIGNED");
    expect(runtimeStateChipLabel("tracking")).toBe("TRACKING");
    expect(runtimeStateChipLabel("none")).toBeNull();
  });

  it("maps chip tones", () => {
    expect(runtimeStateChipTone("tracking", "target")).toBe("active");
    expect(runtimeStateChipTone("assigned", "assignment")).toBe("assigned");
    expect(runtimeStateChipTone("idle", "target")).toBe("idle");
  });

  it("builds compact cesium target chip", () => {
    expect(compactTargetStateChip("assigned")).toBe("TGT");
    expect(compactTargetStateChip("tracking")).toBe("TRK");
    expect(compactTargetStateChip("none")).toBeNull();
  });
});
