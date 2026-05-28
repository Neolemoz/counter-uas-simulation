import { describe, expect, it } from "vitest";
import {
  deriveRowCompareStatus,
  formatCompareStatus,
} from "./compareStatusVocabulary";

describe("compareStatusVocabulary", () => {
  it("formats status labels", () => {
    expect(formatCompareStatus("aligned")).toBe("aligned");
    expect(formatCompareStatus("not_comparable")).toBe("not comparable");
  });

  it("derives aligned and divergent", () => {
    expect(deriveRowCompareStatus("a", "a")).toBe("aligned");
    expect(deriveRowCompareStatus("a", "b")).toBe("divergent");
    expect(deriveRowCompareStatus("—", "b")).toBe("missing");
    expect(deriveRowCompareStatus("x", "y", { explanatory: true })).toBe("explanatory");
  });
});
