import { describe, expect, it } from "vitest";
import {
  livePreflightRows,
  livePreflightSummary,
  parseLivePreflight,
} from "./livePreflight";

describe("livePreflight", () => {
  it("parses bridge preflight payload", () => {
    const parsed = parseLivePreflight({
      schema: "rt_live_runtime_preflight_v1",
      ok: false,
      checks: {
        ros2_available: true,
        gz_available: false,
        rt_sandbox_gz_available: false,
      },
      blockers: ["gz (Gazebo Sim) not found on PATH"],
      message: "gz (Gazebo Sim) not found on PATH",
    });
    expect(parsed?.ok).toBe(false);
    expect(parsed?.checks.gz_available).toBe(false);
    expect(livePreflightSummary(parsed)).toContain("gz");
    expect(livePreflightRows(parsed).find((r) => r.id === "ros2_available")?.ok).toBe(
      true,
    );
  });

  it("returns unknown summary when null", () => {
    expect(livePreflightSummary(null)).toContain("not checked");
  });
});
