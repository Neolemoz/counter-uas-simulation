import { describe, expect, it } from "vitest";
import { scenarioControlStates } from "./scenarioControlStates";

describe("scenarioControlStates", () => {
  it("enables apply only when running or paused", () => {
    expect(scenarioControlStates("running").applyScenario).toBe(true);
    expect(scenarioControlStates("paused").applyScenario).toBe(true);
  });

  it("disables apply for terminal and blocked lifecycle states", () => {
    for (const state of [
      "stopped",
      "captured",
      "discarded",
      "failed",
      "runtime_crashed",
      "unknown",
    ]) {
      expect(scenarioControlStates(state).applyScenario).toBe(false);
    }
  });
});
