import { describe, expect, it } from "vitest";
import {
  adapterModeLabel,
  feedbackEntityRows,
  shouldClearPendingReconcile,
} from "./cognition";

describe("sync cognition", () => {
  it("clears pending reconcile when poll advances", () => {
    const ws = {
      last_poll_utc: "2026-05-25T12:00:02+00:00",
      last_command_utc: "2026-05-25T12:00:01+00:00",
    };
    expect(shouldClearPendingReconcile(true, ws, undefined)).toBe(true);
  });

  it("parses feedback entity drift rows", () => {
    const rows = feedbackEntityRows({
      feedback_entities: [
        { entity_id: "abc", drift_m: 1.5, sync_revision: 2 },
      ],
    });
    expect(rows).toHaveLength(1);
    expect(rows[0]?.driftM).toBe(1.5);
  });

  it("labels adapter mode", () => {
    expect(adapterModeLabel({ adapter_mode: "mock" })).toBe("mock");
    expect(adapterModeLabel(undefined)).toBe("off");
  });
});
