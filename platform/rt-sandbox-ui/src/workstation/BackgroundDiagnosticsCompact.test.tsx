import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import type { SessionSlot } from "@/hooks/useRtSessionWorkspace";
import { BackgroundDiagnosticsCompact } from "./BackgroundDiagnosticsCompact";

function backgroundSlot(sessionId: string): SessionSlot {
  return {
    sessionId,
    subscriptionId: "sub-bg",
    role: "background",
    snapshots: {
      lifecycle_state: { payload: { state: "running" } },
      session_health: { payload: { telemetry_health: "ok" } },
      world_summary: { payload: { entity_count: 1 } },
    },
    lastError: null,
    connected: true,
    lastPullUtc: "2026-05-27T10:00:00.000Z",
    drainedCount: 0,
    pulling: false,
  };
}

describe("BackgroundDiagnosticsCompact", () => {
  it("renders compact chips for background slots", () => {
    const markup = renderToStaticMarkup(
      <BackgroundDiagnosticsCompact
        slots={[backgroundSlot("bg-a"), backgroundSlot("bg-b")]}
        orderedSessionIds={["bg-a", "bg-b"]}
        labelFor={(id) => id}
      />,
    );
    expect(markup).toContain("background-diagnostics-compact");
    expect(markup).toContain("last pull:");
    expect(markup).toContain("Expand diagnostics");
  });

  it("returns null when no background slots", () => {
    const markup = renderToStaticMarkup(
      <BackgroundDiagnosticsCompact
        slots={[]}
        orderedSessionIds={[]}
        labelFor={(id) => id}
      />,
    );
    expect(markup).toBe("");
  });

  it("shows poll paused badge when paused", () => {
    const markup = renderToStaticMarkup(
      <BackgroundDiagnosticsCompact
        slots={[backgroundSlot("bg-a")]}
        orderedSessionIds={["bg-a"]}
        pollPaused
        labelFor={(id) => id}
      />,
    );
    expect(markup).toContain("poll paused");
  });
});
