import { describe, expect, it } from "vitest";
import { TELEMETRY_CHANNELS } from "@/telemetry/constants";
import {
  LIVE_MAINTAINER_SMOKE_HINT,
  LIVE_STOP_GOVERNANCE,
  isLiveRuntimeProfile,
  liveStopCommandType,
} from "./liveSessionUx";

describe("liveSessionUx", () => {
  it("identifies live runtime profile", () => {
    expect(isLiveRuntimeProfile("live")).toBe(true);
    expect(isLiveRuntimeProfile("stub")).toBe(false);
    expect(isLiveRuntimeProfile("mock_adapter")).toBe(false);
  });

  it("routes live stop to stop_sim terminate path", () => {
    expect(liveStopCommandType("live")).toBe("stop_sim");
    expect(liveStopCommandType("stub")).toBe("stop_session");
    expect(liveStopCommandType("mock_adapter")).toBe("stop_session");
  });

  it("surfaces governance copy for live stop and maintainer smoke", () => {
    expect(LIVE_STOP_GOVERNANCE).toContain("terminates the Gazebo adapter");
    expect(LIVE_MAINTAINER_SMOKE_HINT).toContain("rt_live_smoke.py");
    expect(LIVE_MAINTAINER_SMOKE_HINT).toContain("not run from UI");
  });

  it("does not add perception or parser telemetry channels", () => {
    expect(TELEMETRY_CHANNELS).not.toContain("/tracks/state");
    expect(TELEMETRY_CHANNELS).not.toContain("tracks_state");
    expect(TELEMETRY_CHANNELS).toContain("entity_pose_mirror");
  });
});
