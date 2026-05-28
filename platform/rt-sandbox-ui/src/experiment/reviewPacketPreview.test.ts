import { describe, expect, it } from "vitest";
import { createEmptyManifest } from "./experimentStore";
import { buildReviewPacketPreview } from "./reviewPacketPreview";
import { defaultWorkbenchV2State } from "./workbenchV2State";

describe("reviewPacketPreview", () => {
  it("builds advisory packet from v2 state", () => {
    const manifest = createEmptyManifest("exp-preview");
    manifest.runs.push({
      run_id: "run-1",
      label: "one",
      session_id: "s1",
      recorded_at_utc: "2026-05-28T12:00:00Z",
      snapshot: {},
    });
    const v2State = {
      ...defaultWorkbenchV2State(),
      compare_run_a: "run-1",
      steps_completed: ["select_scope", "f1_analytics"],
    };
    const packet = buildReviewPacketPreview({
      v2State,
      manifest,
      presence: { f1_analytics: true, f3_annex: false, f5_metrics: false, f5b_fidelity: false },
    });
    expect(packet.schema).toBe("rt_experiment_review_packet_v1");
    expect(packet.compare_run_ids).toEqual(["run-1"]);
    expect(packet.review_steps_completed).toContain("f1_analytics");
  });
});
