import { describe, expect, it } from "vitest";
import demoAnnex from "./fixtures/tactical_annex_demo_v1.json";
import {
  parseTacticalAnnexJson,
  rtTacticalCaptureAnnexSchema,
  timelineCounts,
} from "./tacticalAnnexSchema";

describe("tacticalAnnexSchema", () => {
  it("parses demo fixture", () => {
    const annex = rtTacticalCaptureAnnexSchema.parse(demoAnnex);
    expect(annex.authority_label).toBe("replay_boundary_scoped");
    expect(annex.final_tactical_mode).toBe("manual");
  });

  it("rejects wrong authority", () => {
    const bad = { ...demoAnnex, authority_label: "command_authoritative" };
    expect(() => rtTacticalCaptureAnnexSchema.parse(bad)).toThrow();
  });

  it("timelineCounts sums lists", () => {
    const annex = rtTacticalCaptureAnnexSchema.parse(demoAnnex);
    const counts = timelineCounts(annex);
    expect(counts.mode_switches).toBe(1);
    expect(counts.selected_timeline).toBe(1);
  });

  it("parseTacticalAnnexJson round-trips", () => {
    const text = JSON.stringify(demoAnnex);
    const annex = parseTacticalAnnexJson(text);
    expect(annex.schema).toBe("rt_tactical_capture_annex_v1");
  });
});
