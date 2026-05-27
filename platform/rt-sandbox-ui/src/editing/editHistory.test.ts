import { describe, expect, it } from "vitest";
import {
  appendEditHistory,
  clearEditHistory,
  createEditHistoryEntry,
} from "./editHistory";

describe("editHistory", () => {
  it("prepends and caps at 32 entries", () => {
    let history = clearEditHistory();
    for (let i = 0; i < 40; i++) {
      history = appendEditHistory(
        history,
        createEditHistoryEntry({
          commandType: "spawn_entity",
          ok: true,
        }),
      );
    }
    expect(history.length).toBe(32);
  });

  it("clear returns empty", () => {
    expect(clearEditHistory()).toEqual([]);
  });
});
