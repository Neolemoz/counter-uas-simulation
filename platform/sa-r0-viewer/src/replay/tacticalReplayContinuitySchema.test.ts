import { describe, expect, it } from "vitest";
import {
  hasTacticalContinuity,
  rtTacticalReplayContinuitySchema,
} from "./tacticalReplayContinuitySchema";

describe("rtTacticalReplayContinuitySchema", () => {
  it("parses valid continuity block", () => {
    const block = rtTacticalReplayContinuitySchema.parse({
      schema: "rt_tactical_replay_continuity_v1",
      source: "rt_sandbox_capture_v1",
      continuity_available: true,
      capture_candidate_id: "cap-1",
      governance_banner: "RT tactical continuity — explanatory replay only; not operational authority",
      provenance: {
        imported_from_rt_capture: true,
        authority_stopped_at: "replay_sa_bundle_pack",
      },
      tactical_annex: {
        schema: "rt_tactical_capture_annex_v1",
        final_tactical_mode: "manual",
        governance_banner: "annex banner",
        authority_label: "replay_boundary_scoped",
      },
    });
    expect(hasTacticalContinuity(block)).toBe(true);
  });

  it("rejects wrong annex authority", () => {
    expect(() =>
      rtTacticalReplayContinuitySchema.parse({
        schema: "rt_tactical_replay_continuity_v1",
        source: "rt_sandbox_capture_v1",
        continuity_available: true,
        capture_candidate_id: "cap-1",
        governance_banner: "banner",
        provenance: {},
        tactical_annex: {
          schema: "rt_tactical_capture_annex_v1",
          authority_label: "command_authoritative",
          governance_banner: "x",
        },
      }),
    ).toThrow();
  });
});
