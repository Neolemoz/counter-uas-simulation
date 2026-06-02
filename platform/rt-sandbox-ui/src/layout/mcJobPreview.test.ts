import { describe, expect, it } from "vitest";
import {
  buildMcJobPreview,
  estimateMcJobDurationSec,
  formatEstimatedDuration,
  scenarioLabelFromSuggestion,
} from "./mcJobPreview";
import type { McProfilePreview } from "./rtLayoutMcProfile";
import { presetById } from "./scenarioEvaluationPresets";

const SAMPLE_PROFILE: McProfilePreview = {
  schema_version: "rt_layout_mc_profile_v1",
  source_layout_id: "rt_layout_mc_golden",
  geometry_id: "rt_layout:sha256:abc123",
  scenario_suggestion: "single",
  launch_args: "target_start_x_m:=-1500 interceptor_ic_layout:=custom:-5,0",
  launch_args_fields: {},
  entity_counts: { drone: 1, interceptor: 3, radar: 1, waypoint_marker: 0 },
  warnings: ["radar entities are retained as metadata only for MC preview"],
  unsupported_fields: [],
  source: { artifact_schema: "rt_layout_scenario_v1", layout_source: {} },
  generated_utc: "2026-06-02T00:00:00Z",
};

describe("mcJobPreview", () => {
  it("estimates duration from run count", () => {
    expect(estimateMcJobDurationSec(10)).toBe(900);
    expect(estimateMcJobDurationSec(50)).toBe(4500);
    expect(estimateMcJobDurationSec(100)).toBe(9000);
  });

  it("formats duration labels", () => {
    expect(formatEstimatedDuration(45)).toBe("~45s");
    expect(formatEstimatedDuration(900)).toBe("~15 min");
    expect(formatEstimatedDuration(4500)).toBe("~1h 15m");
  });

  it("maps scenario suggestions to labels", () => {
    expect(scenarioLabelFromSuggestion("single")).toBe("single-target");
    expect(scenarioLabelFromSuggestion("multi")).toBe("multi-target");
  });

  it("builds job preview from profile and preset", () => {
    const job = buildMcJobPreview(
      SAMPLE_PROFILE,
      presetById("standard"),
      "2026-06-02T12:00:00Z",
    );

    expect(job.schema_version).toBe("rt_mc_job_preview_v1");
    expect(job.geometry_id).toBe(SAMPLE_PROFILE.geometry_id);
    expect(job.preset).toBe("standard");
    expect(job.preset_label).toBe("Standard");
    expect(job.run_count).toBe(50);
    expect(job.estimated_duration_sec).toBe(4500);
    expect(job.estimated_duration).toBe("~1h 15m");
    expect(job.scenario_label).toBe("single-target");
    expect(job.launch_args).toBe(SAMPLE_PROFILE.launch_args);
    expect(job.warnings).toEqual(SAMPLE_PROFILE.warnings);
    expect(job.prepared_utc).toBe("2026-06-02T12:00:00Z");
  });
});
