import { describe, expect, it } from "vitest";
import {
  extractWorldSummaryFromApplyResult,
  patchSnapshotsWithApplyWorldSummary,
  refreshSessionAfterApply,
  worldSummarySnapshotFromApplyResponse,
} from "./applyRefresh";

describe("applyRefresh", () => {
  it("builds world_summary snapshot from apply command response", () => {
    const snap = worldSummarySnapshotFromApplyResponse("sid", {
      entity_count: 2,
      revision: 1,
    });
    expect(snap.channel).toBe("world_summary");
    expect(snap.payload.entity_count).toBe(2);
  });

  it("extracts world_summary from apply result", () => {
    const ws = extractWorldSummaryFromApplyResult({
      ok: true,
      world_summary: { entity_count: 3 },
    });
    expect(ws?.entity_count).toBe(3);
  });

  it("patches slot snapshots with command world_summary", () => {
    const merged = patchSnapshotsWithApplyWorldSummary({}, "sid", {
      entity_count: 1,
    });
    expect(merged.world_summary?.payload.entity_count).toBe(1);
  });

  it("refreshSessionAfterApply pulls until mirror or empty world", async () => {
    let pulls = 0;
    let entityCount = 2;
    await refreshSessionAfterApply({
      pull: async () => {
        pulls += 1;
        if (pulls >= 2) entityCount = 0;
      },
      readWorldSummary: () => ({
        entity_count: entityCount,
        last_poll_utc: "2026-01-01T00:00:01Z",
        last_command_utc: "2026-01-01T00:00:00Z",
      }),
      readEntityMirror: () =>
        entityCount === 0
          ? { channel: "entity_pose_mirror", payload: { entities: [] }, timestamp_utc: "" }
          : undefined,
    });
    expect(pulls).toBeGreaterThanOrEqual(1);
  });
});
