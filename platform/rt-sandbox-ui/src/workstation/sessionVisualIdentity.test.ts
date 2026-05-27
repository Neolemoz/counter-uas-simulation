import { describe, expect, it } from "vitest";
import {
  sessionAccentClass,
  sessionSlotIndex,
  shortSessionId,
} from "@/workstation/sessionVisualIdentity";

describe("sessionVisualIdentity", () => {
  const ordered = ["session-a", "session-b", "session-c"];

  it("maps stable slot index", () => {
    expect(sessionSlotIndex("session-b", ordered)).toBe(1);
    expect(sessionSlotIndex("session-a", ordered)).toBe(0);
  });

  it("returns distinct accent classes per slot", () => {
    expect(sessionAccentClass("session-a", ordered)).not.toBe(
      sessionAccentClass("session-b", ordered),
    );
  });

  it("shortens session id for display", () => {
    expect(shortSessionId("12345678-abcd")).toBe("12345678");
  });
});
