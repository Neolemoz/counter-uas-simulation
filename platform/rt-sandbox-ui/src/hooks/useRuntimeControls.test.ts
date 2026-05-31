import { describe, expect, it } from "vitest";
import {
  resolveSelectedDefenderId,
  resolveSelectedTargetId,
} from "./useRuntimeControls";

describe("useRuntimeControls selection helpers", () => {
  const entities = [
    { entity_id: "d1", entity_type: "interceptor", pose: {} },
    { entity_id: "t1", entity_type: "drone", pose: {} },
  ];

  it("prefers selected interceptor entity for defender", () => {
    expect(resolveSelectedDefenderId("d1", entities, null)).toBe("d1");
  });

  it("falls back to tactical interceptor id", () => {
    expect(resolveSelectedDefenderId(null, entities, "d1")).toBe("d1");
  });

  it("prefers tactical target id", () => {
    expect(resolveSelectedTargetId("t1", entities, "t2")).toBe("t2");
  });

  it("uses selected drone when tactical target unset", () => {
    expect(resolveSelectedTargetId("t1", entities, null)).toBe("t1");
  });
});
