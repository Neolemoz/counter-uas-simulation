import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import { SessionWorkflowStrip } from "./SessionWorkflowStrip";

describe("SessionWorkflowStrip live command health", () => {
  it("shows live connected and command ready for active live session", () => {
    const markup = renderToStaticMarkup(
      <SessionWorkflowStrip
        connected
        sessionState="running"
        simPaused={false}
        editingAllowed
        lastError={null}
        requestedRuntimeProfile="live"
        sessionHealth={{
          channel: "session_health",
          timestamp_utc: "2026-06-05T12:00:00.000Z",
          payload: {
            state: "running",
            runtime_profile: "live",
            adapter_alive: true,
            adapter_mode: "live",
          },
        }}
      />,
    );
    expect(markup).toContain('data-testid="workflow-live-connected"');
    expect(markup).toContain('data-testid="workflow-command-health"');
    expect(markup).toContain("Command path: ready");
  });

  it("shows command unavailable when adapter not running", () => {
    const markup = renderToStaticMarkup(
      <SessionWorkflowStrip
        connected
        sessionState="running"
        simPaused={false}
        editingAllowed
        lastError={null}
        requestedRuntimeProfile="live"
        sessionHealth={{
          channel: "session_health",
          timestamp_utc: "2026-06-05T12:00:00.000Z",
          payload: {
            state: "running",
            runtime_profile: "live",
            adapter_alive: false,
          },
        }}
      />,
    );
    expect(markup).toContain("Command path: unavailable");
  });

  it("omits live command badges for stub profile", () => {
    const markup = renderToStaticMarkup(
      <SessionWorkflowStrip
        connected
        sessionState="running"
        simPaused={false}
        editingAllowed
        lastError={null}
        requestedRuntimeProfile="stub"
      />,
    );
    expect(markup).not.toContain('data-testid="workflow-command-health"');
    expect(markup).not.toContain('data-testid="workflow-live-connected"');
  });
});
