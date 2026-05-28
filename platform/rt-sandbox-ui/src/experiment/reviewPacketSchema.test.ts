import { describe, expect, it } from "vitest";
import { REVIEW_PACKET_GOVERNANCE_BANNER, reviewPacketSchema } from "./reviewPacketSchema";

describe("reviewPacketSchema", () => {
  it("parses minimal valid packet", () => {
    const parsed = reviewPacketSchema.parse({
      schema: "rt_experiment_review_packet_v1",
      packet_id: "review-test",
      created_at_utc: "2026-05-28T12:00:00Z",
      governance_banner: REVIEW_PACKET_GOVERNANCE_BANNER,
      scope: {},
    });
    expect(parsed.schema).toBe("rt_experiment_review_packet_v1");
  });

  it("rejects forbidden top-level keys", () => {
    expect(() =>
      reviewPacketSchema.parse({
        schema: "rt_experiment_review_packet_v1",
        packet_id: "x",
        created_at_utc: "2026-05-28T12:00:00Z",
        governance_banner: REVIEW_PACKET_GOVERNANCE_BANNER,
        scope: {},
        sa_bundle_ref: "bad",
      }),
    ).toThrow();
  });
});
