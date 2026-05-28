import { beforeEach, describe, expect, it, vi } from "vitest";
import {
  advanceReviewStep,
  defaultWorkbenchV2State,
  loadWorkbenchV2State,
  markStepCompleted,
  saveWorkbenchV2State,
  setCompareRuns,
  WORKBENCH_V2_STATE_KEY,
} from "./workbenchV2State";

const memory: Record<string, string> = {};

vi.stubGlobal("localStorage", {
  getItem: (key: string) => memory[key] ?? null,
  setItem: (key: string, value: string) => {
    memory[key] = value;
  },
  removeItem: (key: string) => {
    delete memory[key];
  },
  clear: () => {
    for (const k of Object.keys(memory)) delete memory[k];
  },
});

describe("workbenchV2State", () => {
  beforeEach(() => {
    for (const k of Object.keys(memory)) delete memory[k];
  });

  it("round-trips extended fields", () => {
    const state = {
      ...defaultWorkbenchV2State(),
      active_run_id: "run-a",
      compare_run_a: "run-a",
      compare_run_b: "run-b",
      steps_completed: ["select_scope"] as const,
    };
    saveWorkbenchV2State({
      ...state,
      steps_completed: ["select_scope"],
    });
    const loaded = loadWorkbenchV2State();
    expect(loaded.active_run_id).toBe("run-a");
    expect(loaded.compare_run_a).toBe("run-a");
    expect(memory[WORKBENCH_V2_STATE_KEY]).toBeDefined();
  });

  it("advances review step and marks completed", () => {
    let state = defaultWorkbenchV2State();
    state = markStepCompleted(state, "select_scope");
    state = advanceReviewStep(state);
    expect(state.review_step).toBe("f1_analytics");
    expect(state.steps_completed).toContain("select_scope");
  });

  it("setCompareRuns updates both ids", () => {
    const next = setCompareRuns(defaultWorkbenchV2State(), "a", "b");
    expect(next.compare_run_a).toBe("a");
    expect(next.compare_run_b).toBe("b");
  });
});
