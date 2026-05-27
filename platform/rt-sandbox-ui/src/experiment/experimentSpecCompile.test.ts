import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import {
  compileExperimentSpec,
  compileSpecToYaml,
  computeSpecFingerprint,
  parseExperimentSpec,
} from "./experimentSpecCompile";

const FIXTURE_ROOT = join(
  import.meta.dirname,
  "../../../../fixtures/rt_experiments/f5_spec_examples",
);
const GOLDEN_ROOT = join(
  import.meta.dirname,
  "../../../../fixtures/rt_experiments/f5_compile_goldens",
);

function loadSpec(name: string) {
  const raw = readFileSync(join(FIXTURE_ROOT, name), "utf8");
  return parseExperimentSpec(JSON.parse(raw));
}

describe("experimentSpecCompile", () => {
  it("compiles all five example specs with expected run counts", () => {
    const cases: Array<[string, number]> = [
      ["terrain_comparison.json", 2],
      ["sensor_range_comparison.json", 2],
      ["tactical_mode_comparison.json", 3],
      ["repeatability_sweep.json", 3],
      ["parameter_matrix.json", 4],
    ];
    for (const [file, count] of cases) {
      const spec = loadSpec(file);
      const { batch } = compileExperimentSpec(spec);
      expect(batch.runs).toHaveLength(count);
      expect(batch.schema).toBe("rt_experiment_batch_v1");
      expect(batch.runs.every((r) => r.spec_fingerprint?.length === 16)).toBe(true);
    }
  });

  it("deterministic fingerprint for same spec body", () => {
    const spec = loadSpec("parameter_matrix.json");
    const a = computeSpecFingerprint(spec);
    const b = computeSpecFingerprint(spec);
    expect(a).toBe(b);
  });

  it("fingerprint matches Python golden", () => {
    const spec = loadSpec("parameter_matrix.json");
    const golden = readFileSync(
      join(GOLDEN_ROOT, "parameter_matrix.fingerprint.txt"),
      "utf8",
    ).trim();
    expect(computeSpecFingerprint(spec)).toBe(golden);
  });

  it("matrix cartesian run_ids stable", () => {
    const spec = loadSpec("parameter_matrix.json");
    const { batch } = compileExperimentSpec(spec);
    const ids = batch.runs.map((r) => r.run_id).sort();
    expect(ids).toEqual([
      "m-radar-north-arc-v1-1",
      "m-radar-north-arc-v1-2",
      "m-radar-valley-pair-v1-1",
      "m-radar-valley-pair-v1-2",
    ]);
  });

  it("matches parameter_matrix golden yaml", () => {
    const spec = loadSpec("parameter_matrix.json");
    const yaml = compileSpecToYaml(spec);
    const golden = readFileSync(join(GOLDEN_ROOT, "parameter_matrix.batch.yaml"), "utf8");
    expect(yaml).toBe(golden);
  });

  it("matches repeatability golden yaml", () => {
    const spec = loadSpec("repeatability_sweep.json");
    const yaml = compileSpecToYaml(spec);
    const golden = readFileSync(join(GOLDEN_ROOT, "repeatability_sweep.batch.yaml"), "utf8");
    expect(yaml).toBe(golden);
  });

  it("rejects non-zero jitter", () => {
    const spec = loadSpec("repeatability_sweep.json");
    spec.repeat_config!.jitter_s = 1;
    expect(() => compileExperimentSpec(spec)).toThrow(/jitter_s/);
  });

  it("rejects forbidden matrix axis", () => {
    const spec = loadSpec("parameter_matrix.json");
    spec.matrix_axes!.push({ axis_id: "scenario_pack_ref", values: ["x"] });
    expect(() => compileExperimentSpec(spec)).toThrow(/forbidden matrix axis/);
  });

  it("rejects blocked template path", () => {
    const spec = loadSpec("sensor_range_comparison.json");
    spec.spec_entries![0].template_id = "fixtures/scenarios/foo";
    expect(() => compileExperimentSpec(spec)).toThrow(/blocked/);
  });
});
