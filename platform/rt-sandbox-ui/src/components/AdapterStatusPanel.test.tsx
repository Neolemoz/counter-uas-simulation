import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { AdapterStatusPanel } from "./AdapterStatusPanel";

describe("AdapterStatusPanel", () => {
  it("renders stub profile distinctly", () => {
    const markup = renderToStaticMarkup(
      <AdapterStatusPanel
        sessionHealth={{
          channel: "session_health",
          timestamp_utc: "2026-06-03T12:00:00.000Z",
          payload: { stub_alive: true, adapter_alive: false, state: "running" },
        }}
        worldSummary={{
          channel: "world_summary",
          timestamp_utc: "2026-06-03T12:00:00.000Z",
          payload: { entity_count: 0, revision: 0 },
        }}
        lastPullUtc="2026-06-03T12:00:00.000Z"
      />,
    );
    expect(markup).toContain('data-runtime-profile="stub"');
    expect(markup).toContain("Stub runtime");
    expect(markup).toContain(ADAPTER_VISIBILITY_SNIPPET);
  });

  it("renders live adapter profile and pid", () => {
    const markup = renderToStaticMarkup(
      <AdapterStatusPanel
        sessionHealth={{
          channel: "session_health",
          timestamp_utc: "2026-06-03T12:00:05.000Z",
          payload: {
            stub_alive: false,
            adapter_alive: true,
            adapter_mode: "live",
            adapter_pid: 5511,
            telemetry_health: "ok",
          },
        }}
        worldSummary={{
          channel: "world_summary",
          timestamp_utc: "2026-06-03T12:00:05.000Z",
          payload: {
            adapter_mode: "live",
            sync_health: "ok",
            telemetry_health: "ok",
          },
        }}
        lastPullUtc="2026-06-03T12:00:05.000Z"
      />,
    );
    expect(markup).toContain('data-runtime-profile="live_adapter"');
    expect(markup).toContain("Live adapter");
    expect(markup).toContain("5511");
  });

  it("shows pending start selection before connect", () => {
    const markup = renderToStaticMarkup(
      <AdapterStatusPanel
        sessionHealth={undefined}
        worldSummary={undefined}
        pendingRuntimeProfile="mock_adapter"
      />,
    );
    expect(markup).toContain('data-testid="adapter-status-requested-profile"');
    expect(markup).toContain("Mock adapter");
    expect(markup).toContain("Not live Gazebo");
  });

  it("shows pending live preflight before connect", () => {
    const markup = renderToStaticMarkup(
      <AdapterStatusPanel
        sessionHealth={undefined}
        worldSummary={undefined}
        pendingRuntimeProfile="live"
        livePreflight={{
          ok: false,
          checks: {
            ros2_available: true,
            gz_available: false,
            rt_sandbox_gz_available: false,
          },
          blockers: ["gz (Gazebo Sim) not found on PATH"],
        }}
      />,
    );
    expect(markup).toContain('data-testid="adapter-live-preflight"');
    expect(markup).toContain("gz (Gazebo Sim) on PATH");
    expect(markup).toContain("missing");
  });

  it("shows launch health for live adapter", () => {
    const markup = renderToStaticMarkup(
      <AdapterStatusPanel
        sessionHealth={{
          channel: "session_health",
          timestamp_utc: "2026-06-03T12:00:05.000Z",
          payload: {
            adapter_alive: true,
            adapter_mode: "live",
            adapter_health: { alive: true, mode: "live" },
          },
        }}
        worldSummary={{
          channel: "world_summary",
          timestamp_utc: "2026-06-03T12:00:05.000Z",
          payload: { adapter_mode: "live" },
        }}
        lastPullUtc="2026-06-03T12:00:05.000Z"
      />,
    );
    expect(markup).toContain("launch health");
    expect(markup).toContain("launched");
  });

  it("renders mock adapter with freshness badges", () => {
    const markup = renderToStaticMarkup(
      <AdapterStatusPanel
        sessionHealth={{
          channel: "session_health",
          timestamp_utc: "2026-06-03T12:00:08.000Z",
          payload: {
            adapter_alive: true,
            adapter_mode: "mock",
            adapter_health: { alive: true, mode: "mock" },
          },
        }}
        worldSummary={{
          channel: "world_summary",
          timestamp_utc: "2026-06-03T12:00:08.000Z",
          payload: {
            adapter_mode: "mock",
            sync_health: "feedback_lost",
            telemetry_health: "ok",
          },
        }}
        lastPullUtc="2026-06-03T12:00:08.000Z"
      />,
    );
    expect(markup).toContain('data-runtime-profile="mock_adapter"');
    expect(markup).toContain("Mock adapter");
    expect(markup).toContain("adapter-status-freshness");
  });

  it("renders live poll freshness and maintainer smoke hint", () => {
    const markup = renderToStaticMarkup(
      <AdapterStatusPanel
        sessionHealth={{
          channel: "session_health",
          timestamp_utc: "2026-06-03T12:00:09.000Z",
          payload: {
            adapter_alive: true,
            adapter_mode: "live",
            runtime_profile: "live",
            state: "running",
            live_background_poll_hz: 1,
            last_live_poll_utc: "2026-06-03T12:00:09.000Z",
            adapter_health: { alive: true, mode: "live" },
          },
        }}
        worldSummary={{
          channel: "world_summary",
          timestamp_utc: "2026-06-03T12:00:09.000Z",
          payload: { adapter_mode: "live" },
        }}
        requestedRuntimeProfile="live"
        connected
        lastPullUtc="2026-06-03T12:00:09.000Z"
        entityPoseMirror={{
          channel: "entity_pose_mirror",
          timestamp_utc: "2026-06-03T12:00:09.000Z",
          payload: {
            entities: [{ entity_id: "d1" }],
            telemetry_health: "ok",
          },
        }}
        editingEnabled
      />,
    );
    expect(markup).toContain('data-testid="adapter-live-status"');
    expect(markup).toContain("live profile active");
    expect(markup).toContain("live poll rate");
    expect(markup).toContain('data-testid="adapter-live-poll-freshness"');
    expect(markup).toContain('data-testid="adapter-maintainer-smoke-hint"');
    expect(markup).toContain("rt_live_smoke.py");
    expect(markup).toContain("Mirror:");
    expect(markup).toContain("adapter-mirror-freshness");
    expect(markup).toContain("Command path: ready");
    expect(markup).toContain("adapter-command-health");
  });
});

const ADAPTER_VISIBILITY_SNIPPET = "maintainer-only";
