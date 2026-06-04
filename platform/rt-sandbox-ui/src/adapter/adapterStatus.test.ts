import { describe, expect, it } from "vitest";
import {
  deriveAdapterStatus,
  deriveRuntimeProfile,
  pickAdapterFields,
} from "./adapterStatus";

describe("deriveRuntimeProfile", () => {
  it("classifies stub when adapter is off", () => {
    expect(
      deriveRuntimeProfile(
        { stub_alive: true, adapter_alive: false },
        { adapter_mode: undefined },
      ),
    ).toBe("stub");
  });

  it("classifies mock adapter", () => {
    expect(
      deriveRuntimeProfile(
        { stub_alive: false, adapter_alive: true, adapter_mode: "mock" },
        { adapter_mode: "mock" },
      ),
    ).toBe("mock_adapter");
  });

  it("classifies live adapter", () => {
    expect(
      deriveRuntimeProfile(
        { adapter_alive: true, adapter_mode: "live", adapter_pid: 4242 },
        { adapter_mode: "live" },
      ),
    ).toBe("live_adapter");
  });
});

describe("pickAdapterFields", () => {
  it("merges nested adapter_health with top-level session_health", () => {
    const fields = pickAdapterFields({
      stub_alive: false,
      adapter_alive: false,
      adapter_pid: 100,
      adapter_health: { alive: true, mode: "mock", entity_count: 2 },
      adapter_entity_count: 2,
    });
    expect(fields.adapterAlive).toBe(true);
    expect(fields.adapterMode).toBe("mock");
    expect(fields.adapterPid).toBe("100");
  });
});

describe("deriveAdapterStatus", () => {
  const nowMs = Date.parse("2026-06-03T12:00:10.000Z");

  it("surfaces telemetry and sync freshness from world_summary", () => {
    const view = deriveAdapterStatus({
      sessionHealthPayload: {
        stub_alive: false,
        adapter_alive: true,
        adapter_mode: "mock",
        adapter_pid: 9001,
      },
      worldSummaryPayload: {
        adapter_mode: "mock",
        sync_health: "ok",
        telemetry_health: "stale",
        last_poll_utc: "2026-06-03T12:00:00.000Z",
        telemetry_revision: 3,
      },
      lastPullUtc: "2026-06-03T12:00:09.000Z",
      pullHz: 1,
      nowMs,
    });
    expect(view.profile).toBe("mock_adapter");
    expect(view.adapterPid).toBe("9001");
    expect(view.telemetryHealth).toBe("stale");
    expect(view.syncHealth).toBe("ok");
    expect(view.telemetryFreshness.tone).toBe("warn");
    expect(view.syncFreshness.tone).toBe("ok");
    expect(view.uiPullFreshness.tone).toBe("ok");
  });

  it("marks UI pull stale when last pull is old", () => {
    const view = deriveAdapterStatus({
      sessionHealthPayload: { stub_alive: true, adapter_alive: false },
      worldSummaryPayload: {},
      lastPullUtc: "2026-06-03T11:00:00.000Z",
      pullHz: 1,
      nowMs,
    });
    expect(view.profile).toBe("stub");
    expect(view.uiPullFreshness.tone).toBe("warn");
  });

  it("includes requested session start profile in view", () => {
    const view = deriveAdapterStatus({
      sessionHealthPayload: { adapter_alive: true, adapter_mode: "mock" },
      worldSummaryPayload: { adapter_mode: "mock" },
      requestedRuntimeProfile: "mock_adapter",
    });
    expect(view.requestedProfileLabel).toBe("Mock adapter");
    expect(view.requestedProfileGovernance).toContain("simulation-only");
  });

  it("surfaces live poll freshness and maintainer smoke hint", () => {
    const view = deriveAdapterStatus({
      sessionHealthPayload: {
        adapter_alive: true,
        adapter_mode: "live",
        runtime_profile: "live",
        live_background_poll_hz: 1,
        last_live_poll_utc: "2026-06-03T12:00:09.000Z",
      },
      worldSummaryPayload: { adapter_mode: "live" },
      requestedRuntimeProfile: "live",
      nowMs,
    });
    expect(view.liveProfileActive).toBe(true);
    expect(view.liveBackgroundPollHz).toBe(1);
    expect(view.lastLivePollUtc).toBe("2026-06-03T12:00:09.000Z");
    expect(view.livePollFreshness.tone).toBe("ok");
    expect(view.maintainerSmokeHint).toContain("rt_live_smoke.py");
  });

  it("marks live poll stale when bridge poll is old", () => {
    const view = deriveAdapterStatus({
      sessionHealthPayload: {
        adapter_alive: true,
        adapter_mode: "live",
        runtime_profile: "live",
        live_background_poll_hz: 1,
        last_live_poll_utc: "2026-06-03T11:00:00.000Z",
      },
      worldSummaryPayload: { adapter_mode: "live" },
      nowMs,
    });
    expect(view.livePollFreshness.tone).toBe("warn");
    expect(view.livePollFreshness.label).toContain("stale");
  });

  it("derives mirror freshness fresh/stale/unavailable", () => {
    const fresh = deriveAdapterStatus({
      connected: true,
      sessionHealthPayload: { adapter_alive: true, adapter_mode: "live" },
      worldSummaryPayload: { adapter_mode: "live", telemetry_health: "ok" },
      entityPoseMirrorSnapshot: {
        channel: "entity_pose_mirror",
        timestamp_utc: "2026-06-03T12:00:09.000Z",
        payload: { entities: [{ entity_id: "d1" }], telemetry_health: "ok" },
      },
      nowMs,
    });
    expect(fresh.mirrorFreshness.state).toBe("fresh");

    const stale = deriveAdapterStatus({
      connected: true,
      entityPoseMirrorSnapshot: {
        channel: "entity_pose_mirror",
        timestamp_utc: "2026-06-03T11:00:00.000Z",
        payload: { entities: [], telemetry_health: "stale" },
      },
      nowMs,
    });
    expect(stale.mirrorFreshness.state).toBe("stale");

    const unavailable = deriveAdapterStatus({
      connected: true,
      nowMs,
    });
    expect(unavailable.mirrorFreshness.state).toBe("unavailable");
  });

  it("derives live command health ready and unavailable", () => {
    const ready = deriveAdapterStatus({
      connected: true,
      requestedRuntimeProfile: "live",
      sessionHealthPayload: {
        state: "running",
        runtime_profile: "live",
        adapter_alive: true,
        adapter_mode: "live",
      },
      editingEnabled: true,
    });
    expect(ready.commandHealth?.state).toBe("command_ready");

    const unavailable = deriveAdapterStatus({
      connected: true,
      requestedRuntimeProfile: "live",
      sessionHealthPayload: {
        state: "running",
        runtime_profile: "live",
        adapter_alive: false,
      },
      editingEnabled: true,
    });
    expect(unavailable.commandHealth?.state).toBe("command_unavailable");
  });
});
