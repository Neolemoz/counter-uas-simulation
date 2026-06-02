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
});

const ADAPTER_VISIBILITY_SNIPPET = "maintainer-only";
