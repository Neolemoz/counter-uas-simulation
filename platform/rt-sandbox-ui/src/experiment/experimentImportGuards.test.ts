import { describe, expect, it } from "vitest";
import demoAnnex from "./fixtures/tactical_annex_demo_v1.json";
import {
  formatImportError,
  safeParseAnalyticsReport,
  safeParseAnnex,
  safeParseAnnexBundle,
  safeParseManifest,
} from "./experimentImportGuards";
import { createEmptyManifest } from "./experimentStore";

describe("experimentImportGuards", () => {
  it("safeParseManifest rejects invalid JSON", () => {
    const result = safeParseManifest("{not json");
    expect(result.ok).toBe(false);
    if (!result.ok) {
      expect(formatImportError(result.error).length).toBeGreaterThan(0);
    }
  });

  it("safeParseManifest accepts valid manifest", () => {
    const manifest = createEmptyManifest("exp-guard");
    const result = safeParseManifest(JSON.stringify(manifest));
    expect(result.ok).toBe(true);
    if (result.ok) {
      expect(result.data.experiment_id).toBe("exp-guard");
    }
  });

  it("safeParseAnnex accepts demo fixture", () => {
    const result = safeParseAnnex(JSON.stringify(demoAnnex));
    expect(result.ok).toBe(true);
  });

  it("safeParseAnnexBundle rejects wrong schema", () => {
    const result = safeParseAnnexBundle(JSON.stringify({ schema: "wrong" }));
    expect(result.ok).toBe(false);
  });

  it("safeParseAnalyticsReport rejects empty object", () => {
    const result = safeParseAnalyticsReport("{}");
    expect(result.ok).toBe(false);
  });
});
