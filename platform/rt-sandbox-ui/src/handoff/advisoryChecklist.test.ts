import { describe, expect, it } from "vitest";
import { buildAdvisoryChecklist, CHECKLIST_IDS } from "./advisoryChecklist";
import type { AdvisoryDeriveInput } from "./advisoryTypes";

describe("buildAdvisoryChecklist", () => {
  it("returns eight checklist items in stable order", () => {
    const inp: AdvisoryDeriveInput = {
      candidate: {
        normalization_status: "normalized",
        origin: "rt_sandbox_capture_v1",
      },
      normalization_validation: { valid: true },
      export_events: ["capture_validated", "capture_normalized", "export_pose_normalized"],
      session_lifecycle_state: "running",
      pose_attested: true,
    };
    const checklist = buildAdvisoryChecklist(inp);
    expect(checklist.map((c) => c.id)).toEqual([...CHECKLIST_IDS]);
    expect(checklist.filter((c) => c.status === "pass").length).toBeGreaterThanOrEqual(5);
  });
});
