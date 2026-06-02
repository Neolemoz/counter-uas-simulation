import { describe, expect, it } from "vitest";
import { entityRegistryCommandsAllowed } from "./entityControlStates";

describe("entityRegistryCommandsAllowed", () => {
  it("allows registry commands when running or paused", () => {
    expect(entityRegistryCommandsAllowed("running")).toBe(true);
    expect(entityRegistryCommandsAllowed("paused")).toBe(true);
  });

  it("blocks registry commands for ineligible lifecycle states", () => {
    for (const state of ["stopped", "captured", "discarded", "failed", "runtime_crashed"]) {
      expect(entityRegistryCommandsAllowed(state)).toBe(false);
    }
  });
});
