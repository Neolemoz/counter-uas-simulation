import { renderToStaticMarkup } from "react-dom/server";
import { describe, expect, it } from "vitest";
import type { SessionSlot } from "@/hooks/useRtSessionWorkspace";
import { SessionTabBar } from "./SessionTabBar";

function slot(sessionId: string, role: SessionSlot["role"]): SessionSlot {
  return {
    sessionId,
    subscriptionId: "sub-1",
    role,
    snapshots: {},
    lastError: null,
    connected: true,
    lastPullUtc: null,
    drainedCount: 0,
    pulling: false,
  };
}

const labelFor = (id: string) =>
  id === "aaaa-1111-0000-0000-0000" ? "Alpha run" : id.slice(0, 8);

function reorderIds(
  orderedSessionIds: readonly string[],
  dragId: string,
  dropId: string,
): string[] {
  if (dragId === dropId) return [...orderedSessionIds];
  const next = orderedSessionIds.filter((id) => id !== dragId);
  const dropIdx = next.indexOf(dropId);
  if (dropIdx < 0) return [...orderedSessionIds];
  next.splice(dropIdx, 0, dragId);
  return next;
}

describe("SessionTabBar", () => {
  it("disables new session button at capacity", () => {
    const markup = renderToStaticMarkup(
      <SessionTabBar
        slots={[slot("aaaa-1111", "active"), slot("bbbb-2222", "background")]}
        orderedSessionIds={["aaaa-1111", "bbbb-2222"]}
        selectedSessionId="aaaa-1111"
        editingSessionId="aaaa-1111"
        atCapacity
        busy={false}
        handoffBySession={new Map()}
        labelFor={labelFor}
        onRename={() => undefined}
        onSelect={() => undefined}
        onReorder={() => undefined}
        onNew={() => undefined}
        onClose={() => undefined}
      />,
    );
    expect(markup).toContain("+ New");
    expect(markup).toContain("disabled");
    expect(markup).toContain("Maximum 3 concurrent sessions");
  });

  it("renders tabs in orderedSessionIds order", () => {
    const markup = renderToStaticMarkup(
      <SessionTabBar
        slots={[slot("bbbb-2222", "background"), slot("aaaa-1111", "active")]}
        orderedSessionIds={["aaaa-1111", "bbbb-2222"]}
        selectedSessionId="aaaa-1111"
        editingSessionId="aaaa-1111"
        atCapacity={false}
        busy={false}
        handoffBySession={new Map()}
        labelFor={labelFor}
        onRename={() => undefined}
        onSelect={() => undefined}
        onReorder={() => undefined}
        onNew={() => undefined}
        onClose={() => undefined}
      />,
    );
    const alphaPos = markup.indexOf("aaaa-111");
    const betaPos = markup.indexOf("bbbb-222");
    expect(alphaPos).toBeGreaterThanOrEqual(0);
    expect(betaPos).toBeGreaterThan(alphaPos);
  });

  it("shows display label and drag hint in title", () => {
    const sid = "aaaa-1111-0000-0000-0000";
    const markup = renderToStaticMarkup(
      <SessionTabBar
        slots={[slot(sid, "active")]}
        orderedSessionIds={[sid]}
        selectedSessionId={sid}
        editingSessionId={sid}
        atCapacity={false}
        busy={false}
        handoffBySession={new Map()}
        labelFor={labelFor}
        onRename={() => undefined}
        onSelect={() => undefined}
        onReorder={() => undefined}
        onNew={() => undefined}
        onClose={() => undefined}
      />,
    );
    expect(markup).toContain("Alpha run");
    expect(markup).toContain("Drag to reorder");
  });
});

describe("tab reorder ids", () => {
  it("moves drag id before drop target", () => {
    expect(reorderIds(["a", "b", "c"], "c", "a")).toEqual(["c", "a", "b"]);
  });
});
