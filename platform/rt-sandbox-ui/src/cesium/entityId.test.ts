import { describe, expect, it } from "vitest";
import { parseRtEntityId, toCesiumEntityId } from "./entityId";

describe("entityId", () => {
  it("roundtrips rt entity prefix", () => {
    const id = "abc-123";
    expect(parseRtEntityId(toCesiumEntityId(id))).toBe(id);
  });

  it("rejects non-prefixed ids", () => {
    expect(parseRtEntityId("other-id")).toBeNull();
    expect(parseRtEntityId(undefined)).toBeNull();
  });
});
