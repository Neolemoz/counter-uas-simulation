import { describe, expect, it } from "vitest";
import { buildStepCompletionMap, resolveStepCompletion } from "./reviewStepCompletion";
import { defaultWorkbenchV2State } from "./workbenchV2State";
import type { ExperimentManifest } from "./experimentSchema";

const manifest: ExperimentManifest = {
  schema: "rt_experiment_manifest_v1",
  experiment_id: "exp-test",
  created_at_utc: "2026-05-28T12:00:00Z",
  governance_banner: "RT EXPERIMENT — explanatory compare only; not operational authority",
  runs: [{ run_id: "r1", label: "r1", session_id: "s1", recorded_at_utc: "2026-05-28T12:00:00Z" }],
};

describe("reviewStepCompletion", () => {
  it("marks select_scope complete when manifest has runs", () => {
    const state = resolveStepCompletion("select_scope", {
      v2State: defaultWorkbenchV2State(),
      manifest,
      presence: { f1_analytics: false, f3_annex: false, f5_metrics: false, f5b_fidelity: false },
    });
    expect(state).toBe("complete");
  });

  it("marks f1_analytics missing without presence", () => {
    const state = resolveStepCompletion("f1_analytics", {
      v2State: defaultWorkbenchV2State(),
      manifest,
      presence: { f1_analytics: false, f3_annex: false, f5_metrics: false, f5b_fidelity: false },
    });
    expect(state).toBe("missing");
  });

  it("marks f1_analytics imported when present", () => {
    const state = resolveStepCompletion("f1_analytics", {
      v2State: defaultWorkbenchV2State(),
      manifest,
      presence: { f1_analytics: true, f3_annex: false, f5_metrics: false, f5b_fidelity: false },
    });
    expect(state).toBe("imported");
  });

  it("builds full completion map", () => {
    const map = buildStepCompletionMap({
      v2State: { ...defaultWorkbenchV2State(), review_step: "compare" },
      manifest,
      presence: { f1_analytics: true, f3_annex: false, f5_metrics: false, f5b_fidelity: false },
    });
    expect(map.compare).toBe("active");
    expect(map.f1_analytics).toBe("imported");
  });
});
