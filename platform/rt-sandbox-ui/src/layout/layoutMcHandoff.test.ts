import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import {
  buildLayoutMcHandoffBundle,
  copyMcJobPreview,
  copyRtLayoutScenario,
  downloadMcJobPreview,
  downloadRtLayoutScenario,
  exportMcJobPreviewJson,
  exportRtLayoutScenarioJson,
  isPreparedJobStale,
  suggestedMcJobPreviewFilename,
  suggestedRtLayoutFilename,
} from "./layoutMcHandoff";
import type { McJobPreview } from "./mcJobPreview";
import type { RtLayoutScenarioV1 } from "./rtLayoutMcProfile";

const LAYOUT: RtLayoutScenarioV1 = {
  schema_version: "rt_layout_scenario_v1",
  layout_id: "rt_layout_sess_1",
  terrain_preset: "rt_sandbox_flat",
  entities: [],
  source: { authority: "rt_ui_preview", origin: "test", ui_flow: "test" },
};

const JOB: McJobPreview = {
  schema_version: "rt_mc_job_preview_v1",
  geometry_id: "rt_layout:sha256:abc",
  preset: "standard",
  preset_label: "Standard",
  run_count: 50,
  estimated_duration_sec: 4500,
  estimated_duration: "~1h 15m",
  scenario_label: "single-target",
  launch_args: "target_start_x_m:=-1",
  warnings: [],
  source_layout_id: "rt_layout_sess_1",
  prepared_utc: "2026-06-02T12:00:00Z",
};

describe("layoutMcHandoff", () => {
  beforeEach(() => {
    vi.stubGlobal(
      "URL",
      Object.assign(URL, {
        createObjectURL: vi.fn(() => "blob:test"),
        revokeObjectURL: vi.fn(),
      }),
    );
  });

  afterEach(() => {
    vi.unstubAllGlobals();
  });

  it("exports layout and job JSON with schema versions", () => {
    const layoutJson = exportRtLayoutScenarioJson(LAYOUT);
    const jobJson = exportMcJobPreviewJson(JOB);
    expect(JSON.parse(layoutJson).schema_version).toBe("rt_layout_scenario_v1");
    expect(JSON.parse(jobJson).schema_version).toBe("rt_mc_job_preview_v1");
  });

  it("suggests safe filenames", () => {
    expect(suggestedRtLayoutFilename(LAYOUT)).toBe("rt_layout_sess_1.json");
    expect(suggestedMcJobPreviewFilename(JOB)).toBe("rt_layout_sess_1_mc_job_preview.json");
  });

  it("builds handoff bundle", () => {
    const bundle = buildLayoutMcHandoffBundle(LAYOUT, JOB);
    expect(bundle.schema_version).toBe("rt_layout_mc_handoff_v1");
    expect(bundle.layout.layout_id).toBe("rt_layout_sess_1");
    expect(bundle.mc_job_preview.run_count).toBe(50);
  });

  it("detects stale prepared job geometry", () => {
    expect(isPreparedJobStale(JOB, "rt_layout:sha256:other")).toBe(true);
    expect(isPreparedJobStale(JOB, JOB.geometry_id)).toBe(false);
    expect(isPreparedJobStale(null, JOB.geometry_id)).toBe(false);
  });

  it("triggers layout download", () => {
    const click = vi.fn();
    vi.stubGlobal("document", {
      createElement: vi.fn(() => ({ href: "", download: "", click })),
    });
    downloadRtLayoutScenario(LAYOUT);
    expect(URL.createObjectURL).toHaveBeenCalled();
    expect(click).toHaveBeenCalled();
  });

  it("triggers job download", () => {
    const click = vi.fn();
    vi.stubGlobal("document", {
      createElement: vi.fn(() => ({ href: "", download: "", click })),
    });
    downloadMcJobPreview(JOB);
    expect(click).toHaveBeenCalled();
  });

  it("copies layout json via clipboard", async () => {
    const writeText = vi.fn().mockResolvedValue(undefined);
    vi.stubGlobal("navigator", { clipboard: { writeText } });
    const result = await copyRtLayoutScenario(LAYOUT);
    expect(result.ok).toBe(true);
    expect(writeText.mock.calls[0]![0]).toContain("rt_layout_scenario_v1");
  });

  it("copies job json via clipboard", async () => {
    const writeText = vi.fn().mockResolvedValue(undefined);
    vi.stubGlobal("navigator", { clipboard: { writeText } });
    const result = await copyMcJobPreview(JOB);
    expect(result.ok).toBe(true);
    expect(writeText.mock.calls[0]![0]).toContain("rt_mc_job_preview_v1");
  });
});
