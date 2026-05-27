import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import type { SessionSlot } from "@/hooks/useRtSessionWorkspace";
import { BackgroundDiagnostics } from "./BackgroundDiagnostics";

function backgroundSlot(
  sessionId: string,
  overrides: Partial<SessionSlot> = {},
): SessionSlot {
  return {
    sessionId,
    subscriptionId: "sub-bg",
    role: "background",
    snapshots: {
      lifecycle_state: { payload: { state: "running" } },
      session_health: {
        payload: {
          telemetry_health: "stale",
          source: "runtime_stub",
        },
      },
      world_summary: { payload: { entity_count: 2 } },
    },
    lastError: null,
    connected: true,
    lastPullUtc: "2026-05-27T10:00:00.000Z",
    drainedCount: 0,
    pulling: true,
    ...overrides,
  };
}

const labelFor = (id: string) =>
  id === "bg-session-aaaa" ? "Ridge BG" : "Valley BG";

describe("BackgroundDiagnostics", () => {
  it("shows richer row fields and sorted order", () => {
    const markup = renderToStaticMarkup(
      <BackgroundDiagnostics
        slots={[
          backgroundSlot("bg-session-bbbb"),
          backgroundSlot("bg-session-aaaa"),
        ]}
        handoffBySession={new Map()}
        orderedSessionIds={["bg-session-aaaa", "bg-session-bbbb"]}
        editingSessionId="bg-session-aaaa"
        labelFor={labelFor}
      />,
    );
    const ridgePos = markup.indexOf("Ridge BG");
    const valleyPos = markup.indexOf("Valley BG");
    expect(ridgePos).toBeGreaterThanOrEqual(0);
    expect(valleyPos).toBeGreaterThan(ridgePos);
    expect(markup).toContain("lifecycle: running");
    expect(markup).toContain("editing lock");
    expect(markup).toContain("stale: telemetry");
    expect(markup).toContain("last pull:");
  });

  it("shows poll paused on summary when collapsed", () => {
    const markup = renderToStaticMarkup(
      <BackgroundDiagnostics
        slots={[backgroundSlot("bg-session-aaaa")]}
        handoffBySession={new Map()}
        orderedSessionIds={["bg-session-aaaa"]}
        editingSessionId={null}
        pollPaused
        labelFor={labelFor}
      />,
    );
    expect(markup).toContain("background poll paused");
  });

  it("shows pull fault and stale pull age chips", () => {
    const markup = renderToStaticMarkup(
      <BackgroundDiagnostics
        slots={[
          backgroundSlot("bg-session-aaaa", { lastError: "PULL_FAILED" }),
        ]}
        handoffBySession={new Map()}
        orderedSessionIds={["bg-session-aaaa"]}
        editingSessionId={null}
        labelFor={labelFor}
      />,
    );
    expect(markup).toContain("pull fault: PULL_FAILED");
    expect(markup).toContain("stale: pull fault");
  });
});
