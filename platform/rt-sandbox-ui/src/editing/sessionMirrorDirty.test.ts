import { describe, expect, it } from "vitest";
import { hasUnsyncedLocalMirror } from "./sessionMirrorDirty";

describe("hasUnsyncedLocalMirror", () => {
  const empty = {
    localEntities: {},
    locallyDeletedIds: new Set<string>(),
  };

  it("returns false when mirror is clean", () => {
    expect(hasUnsyncedLocalMirror(empty, false)).toBe(false);
  });

  it("returns true when pending reconcile", () => {
    expect(hasUnsyncedLocalMirror(empty, true)).toBe(true);
  });

  it("returns true when local entities present", () => {
    expect(
      hasUnsyncedLocalMirror(
        {
          localEntities: {
            e1: {
              entity_id: "e1",
              entity_type: "drone",
              pose: {},
            },
          },
          locallyDeletedIds: new Set(),
        },
        false,
      ),
    ).toBe(true);
  });

  it("returns true when locally deleted ids present", () => {
    expect(
      hasUnsyncedLocalMirror(
        {
          ...empty,
          locallyDeletedIds: new Set(["e1"]),
        },
        false,
      ),
    ).toBe(true);
  });
});
