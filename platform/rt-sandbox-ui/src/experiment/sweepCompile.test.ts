import { describe, expect, it } from "vitest";
import { DEFAULT_SWEEP_CATALOG } from "./sweepCatalogData";
import { batchSpecToYaml, compileSweepGroup, parseSweepCatalog } from "./sweepCompile";

describe("sweepCompile", () => {
  it("loads default catalog fixture", () => {
    expect(DEFAULT_SWEEP_CATALOG.sweep_groups.length).toBeGreaterThanOrEqual(3);
    expect(DEFAULT_SWEEP_CATALOG.catalog_id).toBe("rt_sweep_catalog_v1");
  });

  it("parses catalog object", () => {
    const catalog = parseSweepCatalog(DEFAULT_SWEEP_CATALOG);
    expect(catalog.schema).toBe("rt_experiment_sweep_catalog_v1");
  });

  it("compiles tactical_mode_comparison to three runs", () => {
    const group = DEFAULT_SWEEP_CATALOG.sweep_groups.find(
      (g) => g.group_id === "tactical_mode_comparison",
    );
    expect(group).toBeTruthy();
    const spec = compileSweepGroup(group!, "sweep-tactical-test");
    expect(spec.runs).toHaveLength(3);
    expect(spec.runs.every((r) => r.template_id === "interceptor_ready_pair_v1")).toBe(true);
  });

  it("emits yaml with schema and runs", () => {
    const group = DEFAULT_SWEEP_CATALOG.sweep_groups[0];
    const spec = compileSweepGroup(group, "sweep-test");
    const yaml = batchSpecToYaml(spec);
    expect(yaml).toContain("schema: rt_experiment_batch_v1");
    expect(yaml).toContain("runs:");
  });
});
