import { describe, expect, it } from "vitest";
import { classifyBlockerGroups, computeQueuePriority, readinessCohort } from "./advisoryQueue";
import type { AdvisoryStatus } from "./advisoryTypes";
import { ADVISORY_GOVERNANCE_BANNER } from "./advisoryTypes";

function baseStatus(overrides: Partial<AdvisoryStatus> = {}): AdvisoryStatus {
  return {
    schema: "rt_sa_workflow_advisory_status_v1",
    capture_candidate_id: "cap-1",
    advisory_state: "capture_ready",
    advisory_state_label: "Capture ready (advisory)",
    blocked: false,
    block_reasons: [],
    upstream: {},
    governance_banner: ADVISORY_GOVERNANCE_BANNER,
    ...overrides,
  };
}

describe("advisoryQueue", () => {
  it("prioritizes blocked over import_ready", () => {
    const blocked = computeQueuePriority(
      baseStatus({ blocked: true, block_reasons: ["handoff_rejected"], advisory_state: "blocked" }),
    );
    const ready = computeQueuePriority(baseStatus({ advisory_state: "import_ready" }));
    expect(blocked.rank).toBeLessThan(ready.rank);
  });

  it("maps approval_ready cohort", () => {
    expect(readinessCohort(baseStatus({ advisory_state: "approval_ready" }))).toBe("needs_approve");
  });

  it("adds experiment_warn group", () => {
    const groups = classifyBlockerGroups(baseStatus({ advisory_state: "approval_ready" }), {
      experimentWarn: true,
    });
    expect(groups).toContain("experiment_warn");
  });
});
