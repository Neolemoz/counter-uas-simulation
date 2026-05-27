import { describe, expect, it } from "vitest";
import { sortSlotsBySessionOrder } from "./sortSessionSlots";

describe("sortSlotsBySessionOrder", () => {
  it("orders slots by workspace tab order", () => {
    const slots = [
      { sessionId: "c", role: "background" as const },
      { sessionId: "a", role: "active" as const },
      { sessionId: "b", role: "background" as const },
    ];
    const sorted = sortSlotsBySessionOrder(slots, ["a", "b", "c"]);
    expect(sorted.map((s) => s.sessionId)).toEqual(["a", "b", "c"]);
  });

  it("appends unknown ids at end", () => {
    const slots = [{ sessionId: "z" }, { sessionId: "a" }];
    const sorted = sortSlotsBySessionOrder(slots, ["a"]);
    expect(sorted.map((s) => s.sessionId)).toEqual(["a", "z"]);
  });
});
