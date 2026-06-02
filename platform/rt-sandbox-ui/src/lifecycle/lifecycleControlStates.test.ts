import { describe, expect, it } from "vitest";
import { lifecycleControlStates } from "./lifecycleControlStates";

describe("lifecycleControlStates", () => {
  it("enables pause only when running", () => {
    expect(lifecycleControlStates("running")).toEqual({
      pause: true,
      resume: false,
      reset: true,
      stopSession: true,
    });
  });

  it("enables resume only when paused", () => {
    expect(lifecycleControlStates("paused")).toEqual({
      pause: false,
      resume: true,
      reset: true,
      stopSession: true,
    });
  });

  it("disables lifecycle actions for stopped and unknown states", () => {
    for (const state of ["stopped", "discarded", "unknown", "starting"]) {
      expect(lifecycleControlStates(state)).toEqual({
        pause: false,
        resume: false,
        reset: false,
        stopSession: false,
      });
    }
  });
});
